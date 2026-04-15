#!/usr/bin/env python3
import argparse
import csv
import datetime as dt
import queue
import socket
import sys
import threading
import time
from typing import Optional

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

HEARTBEAT_PERIOD_S = 0.05

 
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
        help="Command mode: connect to the server on localhost:PORT and relay commands",
    )
    return parser.parse_args()


def send_line(ser: serial.Serial, msg: str, lock: Optional[threading.Lock] = None) -> None:
    payload = (msg.strip() + "\n").encode("utf-8")
    if lock is not None:
        with lock:
            ser.write(payload)
            ser.flush()
        return
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
    print("  newcsv                  -> rotate to a new CSV log file")
    print("  raw <text>              -> send raw command directly")
    print("  help                    -> show this help")
    print("  quit                    -> exit")


class CsvLogger:
    def __init__(self, out_dir: str) -> None:
        self.out_dir = out_dir
        self._lock = threading.Lock()
        self._fieldnames = ["pc_time_s"] + TEL_FIELDS
        self._csv_file = None
        self._csv_writer = None
        self._csv_path = ""
        self.rotate()

    @property
    def path(self) -> str:
        with self._lock:
            return self._csv_path

    def rotate(self) -> str:
        new_path = make_csv_path(self.out_dir)
        new_file = open(new_path, "w", newline="", encoding="utf-8")
        new_writer = csv.DictWriter(new_file, fieldnames=self._fieldnames)
        new_writer.writeheader()
        new_file.flush()

        old_file = None
        with self._lock:
            old_file = self._csv_file
            self._csv_file = new_file
            self._csv_writer = new_writer
            self._csv_path = new_path

        if old_file is not None:
            old_file.close()

        return new_path

    def write_row(self, row: dict) -> None:
        with self._lock:
            if self._csv_writer is None or self._csv_file is None:
                return
            self._csv_writer.writerow(row)
            self._csv_file.flush()

    def close(self) -> None:
        with self._lock:
            file_to_close = self._csv_file
            self._csv_file = None
            self._csv_writer = None
        if file_to_close is not None:
            file_to_close.close()


def heartbeat_loop(
    ser: serial.Serial,
    stop_evt: threading.Event,
    serial_lock: threading.Lock,
) -> None:
    while not stop_evt.is_set():
        try:
            send_line(ser, "HB", serial_lock)
        except serial.SerialException:
            stop_evt.set()
            return
        stop_evt.wait(HEARTBEAT_PERIOD_S)


def cmd_server_loop(
    port: int,
    ser: serial.Serial,
    stop_evt: threading.Event,
    serial_lock: threading.Lock,
    csv_logger: CsvLogger,
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

                if cmd.lower() in ("newcsv", "new csv"):
                    new_path = csv_logger.rotate()
                    try:
                        conn.sendall(f"[LOG] {new_path}\n".encode("utf-8"))
                    except OSError:
                        break
                    continue

                send_line(ser, cmd, serial_lock)
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
            elif low in ("newcsv", "new csv"):
                wire_cmd = "NEWCSV"
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
    csv_logger: CsvLogger,
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
                csv_logger.write_row(row)

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

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as exc:
        print(f"Failed to open serial port: {exc}")
        return 1

    csv_logger = CsvLogger(args.csv)

    print(f"Connected: {args.port} @ {args.baud}")
    print(f"Logging to: {csv_logger.path}")

    stop_evt = threading.Event()
    line_queue: queue.Queue = queue.Queue()
    serial_lock = threading.Lock()

    hb_thread = threading.Thread(
        target=heartbeat_loop,
        args=(ser, stop_evt, serial_lock),
        daemon=True,
    )
    hb_thread.start()

    t = threading.Thread(
        target=reader_loop,
        args=(ser, stop_evt, csv_logger, line_queue),
        daemon=True,
    )
    t.start()

    try:
        if args.cmd_port is not None:
            # Server mode: commands come from a second terminal via TCP
            cmd_srv_thread = threading.Thread(
                target=cmd_server_loop,
                args=(args.cmd_port, ser, stop_evt, serial_lock, csv_logger),
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
            send_line(ser, "GET GAINS", serial_lock)

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
                    send_line(ser, "GET GAINS", serial_lock)
                    continue
                if low in ("newcsv", "new csv"):
                    new_path = csv_logger.rotate()
                    print(f"[LOG] Now writing to: {new_path}")
                    continue
                if low == "reset i":
                    send_line(ser, "RESET I", serial_lock)
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
                    send_line(ser, f"SET {gain_key} {tokens[2]}", serial_lock)
                    continue
                if low.startswith("raw "):
                    send_line(ser, cmd[4:], serial_lock)
                    continue

                print("Unknown command. Type 'help'.")

    except KeyboardInterrupt:
        pass
    finally:
        stop_evt.set()
        t.join(timeout=1.0)
        hb_thread.join(timeout=1.0)
        try:
            ser.close()
        except Exception:
            pass
        csv_logger.close()

    print("Exited cleanly.")
    return 0


if __name__ == "__main__":
    _args = parse_args()
    if _args.connect is not None:
        raise SystemExit(commander_main(_args.connect))
    raise SystemExit(main(_args))
