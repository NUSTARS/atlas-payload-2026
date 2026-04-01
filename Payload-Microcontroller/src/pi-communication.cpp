#include <pi-communication.h>
#include "SPISlave_T4.h"

// ----- FORMAT -------------------------------------------------------------
// struct __attribute__((packed)) Packet { // 49 bytes + 8 bytes for header
//     float orientation[3]; // 12 bytes
//     float angular_velocity[3]; // 12 bytes
//     double gps_long; // 8 bytes
//     double gps_lat;  // 8 bytes
//     uint8_t state;  // 1 byte
//     float altitude_m; // 4 bytes
//     float battery_v; // 4 bytes
// };

static Packet packet = {};

// Define ONCE in a .cpp (not in a header)
SPISlave_T4<&SPI, SPI_8_BITS> mySPI;

static constexpr uint8_t MAGIC[8]    = {0xA5,0x5A,0xC3,0x3C,0x9E,0xE9,0x11,0x42};
static constexpr uint8_t VERSION     = 0x01;

static constexpr size_t  PKT_LEN     = 72;
static constexpr size_t  OFF_PAYLOAD = 16;
static constexpr size_t  OFF_CRC = PKT_LEN - 4;

// static_assert(sizeof(Packet)==49, "...");
// static_assert(OFF_PAYLOAD + sizeof(Packet) <= OFF_CRC, "Packet too big for frame");

static constexpr uint8_t PAYLOAD_LEN = (uint8_t)sizeof(Packet);

static uint8_t packet_active[PKT_LEN];
static uint8_t packet_build[PKT_LEN];
static volatile bool     packet_ready = false;
static volatile uint32_t seqno        = 0;
volatile uint16_t tx_idx = 0;

// void isr_cs_falling() {
//   tx_idx = 0;
//   const uint8_t* frame = current_frame_ptr();
//   mySPI.pushr(frame[0]);   // preload first byte
//   tx_idx = 1;
// }

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

// sets the orientation, angular velocity, lat/lon/alt fields of the packet struct
void spi_packet_set_imu(const float (&orientation)[3], const float (&angular_velocity)[3], const double (&latLonAlt)[3]) {
    memcpy(packet.orientation,      orientation,      sizeof(packet.orientation));
    memcpy(packet.angular_velocity, angular_velocity, sizeof(packet.angular_velocity));

    packet.gps_lat   = latLonAlt[0];
    packet.gps_long  = latLonAlt[1];
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
  memcpy(&out[0], MAGIC, 8);
  out[8] = VERSION;
  out[9] = PAYLOAD_LEN;
  out[10] = 0;
  out[11] = 0;

  uint32_t s = 0;
  memcpy(&out[12], &s, 4);

  uint32_t crc = crc32_compute(out, OFF_CRC);
  memcpy(&out[OFF_CRC], &crc, 4);
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

void build_spi_buffer() {
  uint8_t* p = packet_build;
  memset(p, 0, PKT_LEN);

  memcpy(&p[0], MAGIC, 8);
  p[8] = VERSION;
  p[9] = PAYLOAD_LEN;
  p[10] = 0;
  p[11] = 0;

  uint32_t s = seqno++;
  memcpy(&p[12], &s, 4);

  memcpy(&p[OFF_PAYLOAD], &packet, sizeof(Packet));

  uint32_t crc = crc32_compute(p, OFF_CRC);
  memcpy(&p[OFF_CRC], &crc, 4);

  noInterrupts();
  memcpy(packet_active, packet_build, PKT_LEN);
  packet_ready = true;
  interrupts();
}

/*
  IMPORTANT (library behavior):
  - pushr() writes a single transmit register (TDR). There is NO queue.
  - Therefore we must write TDR once per received byte, reliably.
  - We also must reset tx_idx at the start of each CS-asserted transaction.
  - Do CS edge detect inside the ISR so we don't miss it due to loop timing.
*/
static void on_spi_rx() {
  (void)mySPI.popr();

  const uint8_t* frame = current_frame_ptr();   // active frame or blank
  mySPI.pushr(frame[tx_idx]);
  tx_idx++;
  if (tx_idx >= PKT_LEN) tx_idx = 0;
  Serial.println("pinged");
}

void setup_slave() {
  mySPI.onReceive(on_spi_rx);
  mySPI.begin();
  mySPI.swapPins();          // you said swap must be true

  tx_idx = 0;

  // preload something so first clock returns a defined byte:
  const uint8_t* frame0 = current_frame_ptr();
  mySPI.pushr(frame0[0]);
  tx_idx = 1;
}



// ------------ OLD VERSION -------------

// #include <pi-communication.h>
// #include "SPISlave_T4.h"


// static struct Packet packet = {
//     .orientation = 0,
//     .angular_velocity = 0,
//     .gps_long = 0,
//     .gps_lat = 0,
//     .state = 0,
//     .altitude_m = 0,
//     .battery_v = 0,
// };

//     // float orientation[3]; // 12 bytes
//     // float angular_velocity[3]; // 12 bytes
//     // float gps_long; // 4 bytes
//     // float gps_lat;  // 4 bytes
//     // uint8_t state;  // 1 byte
//     // float altitude_m; // 4 bytes
//     // float battery_v; // 4 bytes


// // Define ONCE in a .cpp (not in a header)
// SPISlave_T4<&SPI, SPI_8_BITS> mySPI;

// static constexpr uint8_t MAGIC[8]    = {0xA5,0x5A,0xC3,0x3C,0x9E,0xE9,0x11,0x42};
// static constexpr uint8_t VERSION     = 0x01;
// static constexpr size_t  PKT_LEN     = 64;

// static constexpr size_t  OFF_PAYLOAD = 16;
// static constexpr size_t  OFF_CRC = 60;

// static_assert(OFF_PAYLOAD + sizeof(Packet) <= OFF_CRC, "Packet too big for frame");
// static constexpr uint8_t PAYLOAD_LEN = (uint8_t)sizeof(Packet);

// static uint8_t packet_active[PKT_LEN];
// static uint8_t packet_build[PKT_LEN];
// static volatile bool     packet_ready = false;
// static volatile uint32_t seqno        = 0;

// static volatile bool cs_fell = false;   // set by CS interrupt
// static void isr_cs_falling() {
//   cs_fell = true;
// }

// // sets the voltage field of the spi packet struct
// void spi_packet_set_voltage(const float voltage) {
//     packet.battery_v = voltage;
// }

// // sets the altitude field of the spi packet struct
// void spi_packet_set_altitude(const float altitude) {
//     packet.altitude_m = altitude;
// }

// // sets the state field fo the spi packet struct
// void spi_packet_set_state(const uint8_t state) {
//     packet.state = state;
// }

// // sets the orientation and angular velocity fields of the packet struct
// void spi_packet_set_imu(const float (&orientation)[3], const float (&angular_velocity)[3]) {
//     for (uint8_t i = 0; i < 3; i++) {
//         packet.orientation[i] = orientation[i];
//         packet.angular_velocity[i] = angular_velocity[i];
//     }
// }

// static uint32_t crc32_update(uint32_t crc, uint8_t data) {
//   crc ^= data;
//   for (int k = 0; k < 8; k++) {
//     crc = (crc >> 1) ^ (0xEDB88320u & (-(int)(crc & 1)));
//   }
//   return crc;
// }

// static uint32_t crc32_compute(const uint8_t* p, size_t n) {
//   uint32_t crc = 0xFFFFFFFFu;
//   for (size_t i = 0; i < n; i++) crc = crc32_update(crc, p[i]);
//   return ~crc;
// }

// static void build_blank_frame(uint8_t out[PKT_LEN]) {
//   memset(out, 0, PKT_LEN);
//   memcpy(&out[0], MAGIC, 8);
//   out[8] = VERSION;
//   out[9] = PAYLOAD_LEN;
//   out[10] = 0;
//   out[11] = 0;

//   uint32_t s = 0;
//   memcpy(&out[12], &s, 4);

//   uint32_t crc = crc32_compute(out, OFF_CRC);
//   memcpy(&out[OFF_CRC], &crc, 4);
// }

// static inline const uint8_t* current_frame_ptr() {
//   static uint8_t blank[PKT_LEN];
//   static bool init = false;
//   if (!init) { 
//     build_blank_frame(blank); 
//     init = true; 
//   }
//   return packet_ready ? packet_active : blank;
// }

// void build_spi_buffer() {
//   uint8_t* p = packet_build;
//   memset(p, 0, PKT_LEN);

//   memcpy(&p[0], MAGIC, 8);
//   p[8] = VERSION;
//   p[9] = PAYLOAD_LEN;
//   p[10] = 0;
//   p[11] = 0;

//   uint32_t s = seqno++;
//   memcpy(&p[12], &s, 4);

//   memcpy(&p[OFF_PAYLOAD], &packet, sizeof(Packet));

//   uint32_t crc = crc32_compute(p, OFF_CRC);
//   memcpy(&p[OFF_CRC], &crc, 4);

//   noInterrupts();
//   memcpy(packet_active, packet_build, PKT_LEN);
//   packet_ready = true;
//   interrupts();
// }


// // void spi_set_imu_packet(const float a[3], const float g[3], const float q[4]) {
// //   uint8_t* p = packet_build;
// //   memset(p, 0, PKT_LEN);

// //   memcpy(p, MAGIC, 8);
// //   p[8] = VERSION;
// //   p[9] = PAYLOAD_LEN;

// //   uint32_t s = seqno++;
// //   memcpy(&p[12], &s, 4);

// //   memcpy(&p[16], a, 3 * sizeof(float));
// //   memcpy(&p[28], g, 3 * sizeof(float));
// //   memcpy(&p[40], q, 4 * sizeof(float));

// //   uint32_t crc = crc32_compute(p, 60);
// //   memcpy(&p[60], &crc, 4);

// //   noInterrupts();
// //   memcpy(packet_active, packet_build, PKT_LEN);
// //   packet_ready = true;
// //   interrupts();
// // }

// // -----------------------------
// // Streaming TX state (ISR-owned)
// // -----------------------------
// static volatile uint8_t tx_idx = 0;

// /*
//   IMPORTANT (library behavior):
//   - pushr() writes a single transmit register (TDR). There is NO queue.
//   - Therefore we must write TDR once per received byte, reliably.
//   - We also must reset tx_idx at the start of each CS-asserted transaction.
//   - Do CS edge detect inside the ISR so we don't miss it due to loop timing.
// */
// static void on_spi_rx() {
//   (void)mySPI.popr();

//   const uint8_t* frame = current_frame_ptr();   // active frame or blank
//   mySPI.pushr(frame[tx_idx]);
//   tx_idx = (uint8_t)((tx_idx + 1) & 0x3F);      // wrap 0..63
// }

// void setup_slave() {
//   mySPI.onReceive(on_spi_rx);
//   mySPI.begin();
//   mySPI.swapPins();          // you said swap must be true

//   tx_idx = 0;

//   // preload something so first clock returns a defined byte:
//   const uint8_t* frame0 = current_frame_ptr();
//   mySPI.pushr(frame0[0]);
//   tx_idx = 1;
// }