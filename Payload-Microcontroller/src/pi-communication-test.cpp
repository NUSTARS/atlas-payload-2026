#include "SPISlave_T4.h"

// SPI slave instance (Teensy 4.0 default SPI pins)
SPISlave_T4<&SPI, SPI_8_BITS> mySPI;

static constexpr uint8_t MAGIC0  = 0xA5;
static constexpr uint8_t MAGIC1  = 0x5A;
static constexpr uint8_t VERSION = 0x01;

static constexpr size_t PKT_LEN = 49;
static constexpr uint8_t PAYLOAD_LEN = 40;

// Packet buffers (double-buffered)
static uint8_t packet_active[PKT_LEN];   // what SPI sends
static uint8_t packet_build[PKT_LEN];    // what loop builds

static volatile bool packet_ready = false;
static uint32_t seqno = 0;

static uint8_t checksum_xor(const uint8_t* p, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) c ^= p[i];
  return c;
}

// Build a fresh packet and atomically publish it for SPI to transmit
void spi_set_imu_packet(const float a[3], const float g[3], const float q[4]) {
  uint8_t *p = packet_build;

  p[0] = MAGIC0;
  p[1] = MAGIC1;
  p[2] = VERSION;
  p[3] = PAYLOAD_LEN;

  uint32_t s = seqno++;
  p[4] = (uint8_t)(s & 0xFF);
  p[5] = (uint8_t)((s >> 8) & 0xFF);
  p[6] = (uint8_t)((s >> 16) & 0xFF);
  p[7] = (uint8_t)((s >> 24) & 0xFF);

  // payload: accel xyz (12), gyro xyz (12), quat wxyz (16) = 40 bytes
  memcpy(&p[8],       a, 3 * sizeof(float));
  memcpy(&p[8 + 12],  g,  3 * sizeof(float));
  memcpy(&p[8 + 24],  q,  4 * sizeof(float));

  p[48] = checksum_xor(p, 48);

  // Atomically swap into active buffer so SPI never sees a half-written packet
  noInterrupts();
  memcpy(packet_active, packet_build, PKT_LEN);
  packet_ready = true;
  interrupts();
}

// Called when the master performs a transaction.
// We queue data for the master to clock out.
static void on_spi_rx() {
  // Drain any bytes master wrote (we don't use MOSI here)
  while (mySPI.available()) (void)mySPI.popr();

  if (!packet_ready) return;

  // Push one packet worth of bytes into TX FIFO.
  // Rolling-buffer receiver on Pi tolerates if reads start mid-stream.
  for (size_t i = 0; i < PKT_LEN; i++) {
    mySPI.pushr(packet_active[i]);
  }
}

void setup_slave() {
  mySPI.onReceive(on_spi_rx);
  mySPI.begin();

  // Optional: preload recognizable bytes before first real packet is ready
  mySPI.pushr(MAGIC0);
  mySPI.pushr(MAGIC1);
}
