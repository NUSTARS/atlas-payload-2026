import spidev, time, struct

MAGIC0, MAGIC1 = 0xA5, 0x5A
PKT_LEN = 49
PAYLOAD_LEN = 40

spi = spidev.SpiDev()
spi.open(0, 0)
spi.mode = 0
spi.max_speed_hz = 500_000

def xor_checksum(buf: bytes) -> int:
    c = 0
    for b in buf:
        c ^= b
    return c

def try_parse_at(buf: bytes, i: int):
    if i + PKT_LEN > len(buf):
        return None

    pkt = buf[i:i+PKT_LEN]
    if pkt[0] != MAGIC0 or pkt[1] != MAGIC1:
        return None
    if pkt[3] != PAYLOAD_LEN:
        return None
    if xor_checksum(pkt[:48]) != pkt[48]:
        return None

    seq = struct.unpack_from("<I", pkt, 4)[0]
    f = struct.unpack_from("<10f", pkt, 8)
    return {
        "ver": pkt[2],
        "seq": seq,
        "accel": f[0:3],
        "gyro":  f[3:6],
        "quat":  f[6:10],
    }

def pop_packets(buf: bytearray):
    out = []
    i = 0
    while i <= len(buf) - PKT_LEN:
        if buf[i] == MAGIC0 and buf[i+1] == MAGIC1:
            pkt = try_parse_at(buf, i)
            if pkt:
                out.append(pkt)
                del buf[:i+PKT_LEN]
                i = 0
                continue
        i += 1

    if len(buf) > 4096:
        del buf[:-4096]
    return out

READ_N = 256
spi.xfer2([0] * READ_N)  # prime

stream = bytearray()
last_seq = None

while True:
    stream += bytes(spi.xfer2([0] * READ_N))

    pkts = pop_packets(stream)
    if not pkts:
        continue

    pkt = pkts[-1]

    if last_seq is not None and pkt["seq"] != last_seq + 1:
        print(f"[seq jump] last={last_seq} now={pkt['seq']}")
    last_seq = pkt["seq"]

    ax, ay, az = pkt["accel"]
    gx, gy, gz = pkt["gyro"]
    qw, qx, qy, qz = pkt["quat"]

    print(f"seq={pkt['seq']} v={pkt['ver']}")
    print(f"  accel: {ax:+.4f} {ay:+.4f} {az:+.4f}")
    print(f"  gyro : {gx:+.4f} {gy:+.4f} {gz:+.4f}")
    print(f"  quat : {qw:+.4f} {qx:+.4f} {qy:+.4f} {qz:+.4f}")

    time.sleep(0.01)  # optional: avoid spamming / reduce CPU
