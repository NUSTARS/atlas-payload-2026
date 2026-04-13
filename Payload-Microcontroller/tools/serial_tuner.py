#!/usr/bin/env python3
import argparse
import csv
import datetime as dt
import queue
import socket
import sys
import threading
import time

import serial


TEL_FIELDS = [
    "time_s",
    "roll_heading",
    "roll_velocity",
    "roll_accel_est",
    "pid",
    "ff",
    "motor_cmd",
    "p",
    "i",
    "d",
    "ff_gain",
]


def make_csv_path(out_dir: str) -> str:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{out_dir.rstrip('/\\')}/teensy_tune_{stamp}.csv"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactive Teensy gain tuner + CSV logger"
    )
    parser.add_argument("--port", default=None, help="Serial port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--csv", default=".", help="Output folder for CSV log")
    parser.add_argument(
        "--cmd-port",
        type=int,
        default=None,
        metavar="PORT",
        help="Listen on localhost:PORT for commands; this window shows output only",
    )
    parser.add_argument(
        "--connect",
        type=int,
        default=None,
        metavar="PORT",
        help="Commander mode: connect to the server on localhost:PORT and relay commands",
    )
    return parser.parse_args()


def send_line(ser: serial.Serial, msg: str) -> None:
    payload = (msg.strip() + "\n").encode("utf-8")
    ser.write(payload)
    ser.flush()


def print_help() -> None:
    print("Commands:")
    print("  gains                   -> GET current gains")
    print("  set p <v>               -> SET P gain")
    print("  set i <v>               -> SET I gain")
    print("  set d <v>               -> SET D gain")
    print("  set ff <v>              -> SET FF gain")
    print("  reset i                 -> zero integrator")
    print("  raw <text>              -> send raw command directly")
    print("  help                    -> show this help")
    print("  quit                    -> exit")


def cmd_server_loop(
    port: int,
    ser: serial.Serial,
    stop_evt: threading.Event,
) -> None:
    """Accept one TCP client at a time; relay received lines to the serial port."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    srv.settimeout(1.0)
    print(f"[CMD] Listening for commander on 127.0.0.1:{port}", flush=True)

    while not stop_evt.is_set():
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        except OSError:
            break

        print(f"[CMD] Commander connected from {addr}", flush=True)
        try:
            rfile = conn.makefile("r", encoding="utf-8")
            while not stop_evt.is_set():
                try:
                    line = rfile.readline()
                except OSError:
                    break
                if not line:
                    break
                cmd = line.strip()
                if not cmd:
                    continue
                send_line(ser, cmd)
                try:
                    conn.sendall(f"[ACK] {cmd}\n".encode("utf-8"))
                except OSError:
                    break
        finally:
            conn.close()
            print("[CMD] Commander disconnected.", flush=True)

    srv.close()


def commander_main(port: int) -> int:
    """Commander mode: connect to a running serial_tuner server and relay commands."""
    try:
        sock = socket.create_connection(("127.0.0.1", port), timeout=5)
    except OSError as exc:
        print(f"Cannot connect to 127.0.0.1:{port}: {exc}")
        return 1

    sock.settimeout(None)
    print(f"Connected to serial_tuner server on port {port}.")

    def recv_loop() -> None:
        try:
            for line in sock.makefile("r", encoding="utf-8"):
                print(line.rstrip(), flush=True)
        except OSError:
            pass

    threading.Thread(target=recv_loop, daemon=True).start()
    print_help()

    try:
        while True:
            try:
                cmd = input("> ").strip()
            except EOFError:
                break
            if not cmd:
                continue

            low = cmd.lower()
            if low in ("quit", "exit"):
                break
            if low == "help":
                print_help()
                continue
            if low == "gains":
                wire_cmd = "GET GAINS"
            elif low == "reset i":
                wire_cmd = "RESET I"
            elif low.startswith("set "):
                tokens = cmd.split()
                if len(tokens) != 3:
                    print("Expected: set <p|i|d|ff> <value>")
                    continue
                gain_key = tokens[1].upper()
                if gain_key not in ("P", "I", "D", "FF"):
                    print("Gain must be one of: p, i, d, ff")
                    continue
                try:
                    float(tokens[2])
                except ValueError:
                    print("Value must be numeric")
                    continue
                wire_cmd = f"SET {gain_key} {tokens[2]}"
            elif low.startswith("raw "):
                wire_cmd = cmd[4:]
            else:
                print("Unknown command. Type 'help'.")
                continue

            try:
                sock.sendall((wire_cmd + "\n").encode("utf-8"))
            except OSError as exc:
                print(f"Connection lost: {exc}")
                break
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    print("Exited cleanly.")
    return 0


def reader_loop(
    ser: serial.Serial,
    stop_evt: threading.Event,
    csv_writer: csv.DictWriter,
    csv_file,
    line_queue: queue.Queue,
) -> None:
    telem_count = 0
    last_print = time.time()

    while not stop_evt.is_set():
        try:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            if line.startswith("TEL,"):
                parts = line.split(",")
                if len(parts) != len(TEL_FIELDS) + 1:
                    continue

                row = {}
                ok = True
                for i, field in enumerate(TEL_FIELDS, start=1):
                    try:
                        row[field] = float(parts[i])
                    except ValueError:
                        ok = False
                        break

                if not ok:
                    continue

                row["pc_time_s"] = time.time()
                csv_writer.writerow(row)
                csv_file.flush()

                telem_count += 1
                now = time.time()
                if now - last_print >= 1.0:
                    print(
                        f"[TEL] {telem_count} samples | "
                        f"roll={row['roll_heading']:.3f} rate={row['roll_velocity']:.3f} "
                        f"cmd={row['motor_cmd']:.3f}",
                        flush=True,
                    )
                    last_print = now
            else:
                line_queue.put(line)
        except serial.SerialException as exc:
            line_queue.put(f"[SERIAL ERROR] {exc}")
            stop_evt.set()
            return


def main(args: argparse.Namespace) -> int:
    if args.port is None:
        print("--port is required. Use --connect PORT for commander mode.")
        return 1

    csv_path = make_csv_path(args.csv)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as exc:
        print(f"Failed to open serial port: {exc}")
        return 1

    print(f"Connected: {args.port} @ {args.baud}")
    print(f"Logging to: {csv_path}")

    csv_file = open(csv_path, "w", newline="", encoding="utf-8")
    fieldnames = ["pc_time_s"] + TEL_FIELDS
    csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    csv_writer.writeheader()
    csv_file.flush()

    stop_evt = threading.Event()
    line_queue: queue.Queue = queue.Queue()

    t = threading.Thread(
        target=reader_loop,
        args=(ser, stop_evt, csv_writer, csv_file, line_queue),
        daemon=True,
    )
    t.start()

    try:
        if args.cmd_port is not None:
            # Server mode: commands come from a second terminal via TCP
            cmd_srv_thread = threading.Thread(
                target=cmd_server_loop,
                args=(args.cmd_port, ser, stop_evt),
                daemon=True,
            )
            cmd_srv_thread.start()
            print("Output window active. In a second terminal run:")
            print(f"  python3 serial_tuner.py --connect {args.cmd_port}")
            while not stop_evt.is_set():
                while True:
                    try:
                        msg = line_queue.get_nowait()
                        print(f"[MCU] {msg}", flush=True)
                    except queue.Empty:
                        break
                time.sleep(0.05)
        else:
            # Normal mode: commands come from stdin in this window
            print_help()
            send_line(ser, "GET GAINS")

            while not stop_evt.is_set():
                while True:
                    try:
                        msg = line_queue.get_nowait()
                        print(f"[MCU] {msg}")
                    except queue.Empty:
                        break

                try:
                    cmd = input("> ").strip()
                except EOFError:
                    cmd = "quit"

                if not cmd:
                    continue

                low = cmd.lower()
                if low in ("quit", "exit"):
                    break
                if low == "help":
                    print_help()
                    continue
                if low == "gains":
                    send_line(ser, "GET GAINS")
                    continue
                if low == "reset i":
                    send_line(ser, "RESET I")
                    continue
                if low.startswith("set "):
                    tokens = cmd.split()
                    if len(tokens) != 3:
                        print("Expected: set <p|i|d|ff> <value>")
                        continue
                    gain_key = tokens[1].upper()
                    if gain_key not in ("P", "I", "D", "FF"):
                        print("Gain must be one of: p, i, d, ff")
                        continue
                    try:
                        float(tokens[2])
                    except ValueError:
                        print("Value must be numeric")
                        continue
                    send_line(ser, f"SET {gain_key} {tokens[2]}")
                    continue
                if low.startswith("raw "):
                    send_line(ser, cmd[4:])
                    continue

                print("Unknown command. Type 'help'.")

    except KeyboardInterrupt:
        pass
    finally:
        stop_evt.set()
        t.join(timeout=1.0)
        try:
            ser.close()
        except Exception:
            pass
        csv_file.close()

    print("Exited cleanly.")
    return 0


if __name__ == "__main__":
    _args = parse_args()
    if _args.connect is not None:
        raise SystemExit(commander_main(_args.connect))
    raise SystemExit(main(_args))
