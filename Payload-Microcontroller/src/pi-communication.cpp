#include <pi-communication.h>
#include "SPISlave_T4.h"


static struct Packet packet = {
    .magic_number = 0,
    .orientation = 0,
    .angular_velocity = 0,
    .gps_long = 0,
    .gps_lat = 0,
    .state = 0,
    .altitude_m = 0,
    .battery_v = 0,
};

// Define ONCE in a .cpp (not in a header)
SPISlave_T4<&SPI, SPI_8_BITS> mySPI;

static constexpr uint8_t MAGIC[8]    = {0xA5,0x5A,0xC3,0x3C,0x9E,0xE9,0x11,0x42};
static constexpr uint8_t VERSION     = 0x01;
static constexpr uint8_t PAYLOAD_LEN = 40;
static constexpr size_t  PKT_LEN     = 64;

static uint8_t packet_active[PKT_LEN];
static uint8_t packet_build[PKT_LEN];
static volatile bool     packet_ready = false;
static volatile uint32_t seqno        = 0;

// sets the voltage field of the spi packet struct
void spi_packet_set_voltage(const float voltage) {
    packet.battery_v = voltage;
}

// sets the altitude field of the spi packet struct
void spi_packet_set_altitude(const float altitude) {
    packet.altitude_m = altitude;
}

// sets the state field fo the spi packet struct
void spi_packet_set_state(const uint8_t state) {
    packet.state = state;
}

// sets the orientation and angular velocity fields of the packet struct
void spi_packet_set_imu(const float (&orientation)[3], const float (&angular_velocity)[3]) {
    for (uint8_t i = 0; i < 3; i++) {
        packet.orientation[i] = orientation[i];
        packet.angular_velocity[i] = angular_velocity[i];
    }
}

static uint32_t crc32_update(uint32_t crc, uint8_t data) {
  crc ^= data;
  for (int k = 0; k < 8; k++) {
    crc = (crc >> 1) ^ (0xEDB88320u & (-(int)(crc & 1)));
  }
  return crc;
}

static uint32_t crc32_compute(const uint8_t* p, size_t n) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; i++) crc = crc32_update(crc, p[i]);
  return ~crc;
}

static void build_blank_frame(uint8_t out[PKT_LEN]) {
  memset(out, 0, PKT_LEN);
  memcpy(out, MAGIC, 8);
  out[8] = VERSION;
  out[9] = PAYLOAD_LEN;

  uint32_t crc = crc32_compute(out, 60);
  memcpy(&out[60], &crc, 4); // little-endian on Teensy
}

static inline const uint8_t* current_frame_ptr() {
  static uint8_t blank[PKT_LEN];
  static bool init = false;
  if (!init) { 
    build_blank_frame(blank); 
    init = true; 
  }
  return packet_ready ? packet_active : blank;
}

void spi_set_imu_packet(const float a[3], const float g[3], const float q[4]) {
  uint8_t* p = packet_build;
  memset(p, 0, PKT_LEN);

  memcpy(p, MAGIC, 8);
  p[8] = VERSION;
  p[9] = PAYLOAD_LEN;

  uint32_t s = seqno++;
  memcpy(&p[12], &s, 4);

  memcpy(&p[16], a, 3 * sizeof(float));
  memcpy(&p[28], g, 3 * sizeof(float));
  memcpy(&p[40], q, 4 * sizeof(float));

  uint32_t crc = crc32_compute(p, 60);
  memcpy(&p[60], &crc, 4);

  noInterrupts();
  memcpy(packet_active, packet_build, PKT_LEN);
  packet_ready = true;
  interrupts();
}

// -----------------------------
// Streaming TX state (ISR-owned)
// -----------------------------
static volatile uint8_t tx_idx     = 0;
static volatile bool    in_txn_isr = false;

/*
  IMPORTANT (library behavior):
  - pushr() writes a single transmit register (TDR). There is NO queue.
  - Therefore we must write TDR once per received byte, reliably.
  - We also must reset tx_idx at the start of each CS-asserted transaction.
  - Do CS edge detect inside the ISR so we don't miss it due to loop timing.
*/
static void on_spi_rx() {
  const bool cs_active = mySPI.active(); // true while CS asserted

  // Start of transaction (CS just asserted)
  if (cs_active && !in_txn_isr) {
    in_txn_isr = true;
    tx_idx = 0;

    // Preload first byte so the *next* clock returns frame[0].
    // Pi should clock 65 bytes and drop the first received byte.
    const uint8_t* frame0 = current_frame_ptr();
    mySPI.pushr(frame0[tx_idx]);
    tx_idx = 1;
  }
  // End of transaction (CS deasserted)
  else if (!cs_active && in_txn_isr) {
    in_txn_isr = false;
    // no other action needed
  }

  // Consume one RX byte (Pi sends dummy)
  (void)mySPI.popr();

  // Queue next byte for the next clock
  const uint8_t* frame = current_frame_ptr();
  mySPI.pushr(frame[tx_idx]);
  tx_idx = (uint8_t)((tx_idx + 1) & 0x3F); // wrap 0..63
}

void setup_slave() {
  mySPI.onReceive(on_spi_rx);
  mySPI.begin();
  mySPI.swapPins();

  // Safe default before first CS edge is observed
  mySPI.pushr(MAGIC[0]);

  // Reset ISR transaction state
  tx_idx = 0;
  in_txn_isr = false;
}
