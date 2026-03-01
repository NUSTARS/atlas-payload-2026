// imu.cpp
//
// contains all logic and control
// related to setting up and getting data
// from the IMU
#include "imu.h"
#ifdef ARDUINO
 #include <HardwareSerial.h>
  static HardwareSerial IMUSerial(1);
  HardwareSerial& IMUSerial = Serial1;
#else
  // This part runs on your computer
#include <cstdint>
#include <cstring>
#include <vector>
#include <string>
#include <stdexcept>
#include <optional>
#include <iostream>
#endif



// initIMU: (int, int, int) -> (int)
// initializes the IMU by writing ASCII commands over serial to the IMU
// baudRate: the baud rate for serial communication, defaults at 115200
// serialNum: the serial number of the teensy connected to the IMU
// outputmode: the output mode for the IMU (0: asynchronous binary, 1: asynchronous ASCII, 2: synchronous binary, 3: synchronous ASCII)  
// returns 1 on successful connection, 0 otherwise
int initIMU(int baudRate = 115200){
    sendUARTCommand("$VNASY,0*4E"); //disable asynchronous data output
    if (baudRate != 115200) {
        sendUARTCommand("$VNWRG,05,baudRate*XX"); //set baud rate
    }
    sendUARTCommand("$VNWRG,06,17,0*XX"); //set output mode. rn its Yaw, Pitch, Roll, Inertial True Acceleration and Angular Rate Measurements but look into body vs inertial
    // sendUARTCommand("$VNWRG,60"); //add timestamp to data output

    



    return 0;

}



enum GroupOffset {
    GROUP_COMMON = 0,
    GROUP_TIME   = 1,
    GROUP_IMU    = 2,
    GROUP_GNSS   = 3,
    GROUP_ATT    = 4,
    GROUP_INS    = 5,
    GROUP_GNSS2  = 6
};
static size_t typeSizeBytes(int groupOffset, int typeOffset)
{
    switch (groupOffset)
    {
        case GROUP_COMMON:
            switch (typeOffset) {
                case 0:  return 8;   // TimeStartup
                case 1:  return 8;   // TimeGps
                case 2:  return 8;   // TimeSyncIn
                case 3:  return 12;  // Ypr
                case 4:  return 16;  // Quaternion
                case 5:  return 12;  // AngularRate
                case 6:  return 24;  // PosLla
                case 7:  return 12;  // VelNed
                case 8:  return 12;  // Accel
                case 9:  return 24;  // Imu
                case 10: return 20;  // MagPres
                case 11: return 28;  // Deltas
                case 12: return 2;   // InsStatus
                case 13: return 4;   // SyncInCnt
                case 14: return 8;   // TimeGpsPps
                default: return 0;
            }
        case GROUP_TIME:
            switch (typeOffset) {
                case 0: return 8;  // TimeStartup
                case 1: return 8;  // TimeGps
                case 2: return 8;  // GpsTow
                case 3: return 2;  // GpsWeek
                case 4: return 8;  // TimeSyncIn
                case 5: return 8;  // TimeGpsPps
                case 6: return 8;  // TimeUtc
                case 7: return 4;  // SyncInCnt
                case 8: return 4;  // SyncOutCnt
                case 9: return 1;  // TimeStatus
                default: return 0;
            }

        case GROUP_GNSS:
            switch (typeOffset) {
                case 0: return 8;   // TimeUtc
                case 1: return 8;   // GpsTow
                case 2: return 2;   // GpsWeek
                case 3: return 1;   // NumSats
                case 4: return 1;   // GnssFix
                case 5: return 24;  // GnssPosLla
                case 6: return 24;  // GnssPosEcef
                case 7: return 12;  // GnssVelNed
                case 8: return 12;  // GnssVelEcef
                case 9: return 12;  // GnssPosUncertainty
                case 10: return 4;  // GnssVelUncertainty
                case 11: return 4;  // GnssTimeUncertainty
                case 12: return 2;  // GnssTimeInfo
                case 13: return 28; // GnssDop
                case 14: return 10; // GnssSatInfo
                case 16: return 40; // GnssRawMeas
                case 17: return 2;  // GnssStatus
                case 18: return 8;  // GnssAltMSL
                default: return 0;
            }

        case GROUP_ATT:
            switch (typeOffset) {
                case 1: return 12;  // Ypr
                case 2: return 16;  // Quaternion
                case 3: return 36;  // Dcm
                case 4: return 12;  // MagNed
                case 5: return 12;  // AccelNed
                case 6: return 12;  // LinBodyAcc
                case 7: return 12;  // LinAccelNed
                case 8: return 12;  // YprU
                case 12: return 12; // (table shows attitude offset 12 size 12)
                case 13: return 4;  // (table shows attitude offset 13 size 4)
                default: return 0;  // offsets 0,9,10 are "-" in your table snippet
            }

        case GROUP_GNSS2:
            // same sizes as GNSS2 column in your table
            switch (typeOffset) {
                case 0: return 8;
                case 1: return 8;
                case 2: return 2;
                case 3: return 1;
                case 4: return 1;
                case 5: return 24;
                case 6: return 24;
                case 7: return 12;
                case 8: return 12;
                case 9: return 12;
                case 10: return 4;
                case 11: return 4;
                case 12: return 2;
                case 13: return 28;
                case 14: return 10; // variable
                case 16: return 40; // variable
                case 17: return 2;
                case 18: return 8;
                default: return 0;
            }

        case GROUP_INS:
            switch (typeOffset) {
                case 0:  return 2;   // InsStatus
                case 1:  return 24;  // PosLla
                case 2:  return 24;  // PosEcef
                case 3:  return 12;  // VelBody
                case 4:  return 12;  // VelNed
                case 5:  return 12;  // VelEcef
                case 6:  return 12;  // MagEcef
                case 7:  return 12;  // AccelEcef
                case 8:  return 12;  // LinAccelEcef
                case 9:  return 4;   // PosU
                case 10: return 4;   // VelU
                default: return 0;
            }
       
        // Optional: sizes for other groups if you want to "skip unknown groups safely"
        // For now return 0 so we throw if an unknown group/type is enabled.
        default:
            return 0;
    }
}

static uint16_t u16le(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t u32le(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t u64le(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 7; i >= 0; --i) v = (v << 8) | p[i];
    return v;
}

static float f32le(const uint8_t* p) {
    float v;
    std::memcpy(&v, p, 4);
    return v;
}

static double f64le(const uint8_t* p) {
    double v;
    std::memcpy(&v, p, 8);
    return v;
}
static std::string hex2(uint8_t b) {
    const char* d = "0123456789ABCDEF";
    std::string s;
    s += d[b >> 4];
    s += d[b & 0xF];
    return s;
}

static void dumpBytes(const std::vector<uint8_t>& pkt, size_t p, size_t n, size_t payloadEnd) {
    size_t end = p + n;
    if (end > payloadEnd) end = payloadEnd;

    for (size_t i = p; i < end; ++i) {
        std::cout << hex2(pkt[i]) << " ";
    }
}

static void requireBytes(size_t p, size_t need, size_t payloadEnd, const std::string& what) {
    if (p + need > payloadEnd) {
        throw std::runtime_error("Payload underrun in " + what + " need=" +
                                 std::to_string(need) + " p=" + std::to_string(p) +
                                 " payloadEnd=" + std::to_string(payloadEnd));
    }
}
static void skipGnssSatInfo(
    const std::vector<uint8_t>& pkt,
    size_t& p,
    size_t payloadEnd,
    const char* label)
{
    // Need at least 2 bytes for Count + Resv
    requireBytes(p, 2, payloadEnd, label);

    uint8_t count = pkt[p + 0];
    // uint8_t resv  = pkt[p + 1];

    size_t total = 2 + (size_t)count * 8; // <-- 8 bytes per sat (per your screenshot)
    requireBytes(p, total, payloadEnd, label);

    std::cout << "  Count=" << (unsigned)count
              << " totalBytes=" << total << "\n";

    // Optional: dump first few bytes
    std::cout << "  raw (first up to 64): ";
    dumpBytes(pkt, p, (total < 64 ? total : 64), payloadEnd);
    std::cout << "\n";

    p += total;
}

static void skipGnssRawMeas(
    const std::vector<uint8_t>& pkt,
    size_t& p,
    size_t payloadEnd,
    const char* label)
{

    // Need at least 12 bytes to even try the variable format
    if (p + 12 > payloadEnd) {
        throw std::runtime_error(std::string("Payload underrun in ") + label);
    }

    uint8_t numSats = pkt[p + 10];
    size_t totalVar = 12 + (size_t)numSats * 28;

    bool saneCount = (numSats <= 40);                 // pick your bound
    bool fitsVar   = (p + totalVar <= payloadEnd);

    if (saneCount && fitsVar) {
        std::cout << "  NumSats=" << (unsigned)numSats
                  << " totalBytes=" << totalVar << "\n";
        p += totalVar;
        return;
    }

    // Fallback: treat as fixed 40 bytes (the minimum size)
    size_t totalFixed = 40;
    if (p + totalFixed > payloadEnd) {
        throw std::runtime_error("Payload underrun in " + std::string(label) +
                                 " (fallback40) need=40 p=" + std::to_string(p) +
                                 " payloadEnd=" + std::to_string(payloadEnd));
    }

    std::cout << "  WARNING: RawMeas variable parse rejected "
              << "(NumSats=" << (unsigned)numSats
              << ", totalVar=" << totalVar
              << "). Falling back to fixed 40.\n";

    p += totalFixed;
}
static void printUnknownFixedInterpretations(
    const std::vector<uint8_t>& pkt,
    size_t p,
    size_t need,
    size_t payloadEnd)
{
    requireBytes(p, need, payloadEnd, "UnknownFixed");

    // Always show raw
    std::cout << "  raw: ";
    dumpBytes(pkt, p, need, payloadEnd);
    std::cout << "\n";

    // Interpret based on size
    if (need == 1) {
        uint8_t u = pkt[p];
        std::cout << "  as u8: " << (unsigned)u << "\n";
        std::cout << "  as i8: " << (int)(int8_t)u << "\n";
        return;
    }

    if (need == 2) {
        uint16_t u = u16le(&pkt[p]);
        int16_t  s = (int16_t)u;
        std::cout << "  as u16: " << u << "\n";
        std::cout << "  as i16: " << s << "\n";
        return;
    }

    if (need == 4) {
        uint32_t u = u32le(&pkt[p]);
        int32_t  s = (int32_t)u;
        float    f = f32le(&pkt[p]);
        std::cout << "  as u32: " << u << "\n";
        std::cout << "  as i32: " << s << "\n";
        std::cout << "  as f32: " << f << "\n";
        return;
    }

    if (need == 8) {
        uint64_t u = u64le(&pkt[p]);
        int64_t  s = (int64_t)u;
        double   d = f64le(&pkt[p]);
        std::cout << "  as u64: " << u << "\n";
        std::cout << "  as i64: " << s << "\n";
        std::cout << "  as f64: " << d << "\n";
        return;
    }

    if (need == 12) {
        // plausible: vec3f32 or 3x u32/i32
        float a = f32le(&pkt[p+0]);
        float b = f32le(&pkt[p+4]);
        float c = f32le(&pkt[p+8]);
        std::cout << "  as vec3f32: " << a << ", " << b << ", " << c << "\n";

        uint32_t u0 = u32le(&pkt[p+0]), u1 = u32le(&pkt[p+4]), u2 = u32le(&pkt[p+8]);
        std::cout << "  as 3x u32:  " << u0 << ", " << u1 << ", " << u2 << "\n";
        std::cout << "  as 3x i32:  " << (int32_t)u0 << ", " << (int32_t)u1 << ", " << (int32_t)u2 << "\n";
        return;
    }

    if (need == 16) {
        // plausible: quat (4*f32) or 2*f64
        float a = f32le(&pkt[p+0]);
        float b = f32le(&pkt[p+4]);
        float c = f32le(&pkt[p+8]);
        float d = f32le(&pkt[p+12]);
        std::cout << "  as 4xf32:   " << a << ", " << b << ", " << c << ", " << d << "\n";

        double d0 = f64le(&pkt[p+0]);
        double d1 = f64le(&pkt[p+8]);
        std::cout << "  as 2xf64:   " << d0 << ", " << d1 << "\n";
        return;
    }

    if (need == 24) {
        // plausible: vec3f64 or 6*f32
        double a = f64le(&pkt[p+0]);
        double b = f64le(&pkt[p+8]);
        double c = f64le(&pkt[p+16]);
        std::cout << "  as vec3f64: " << a << ", " << b << ", " << c << "\n";

        float f0 = f32le(&pkt[p+0]),  f1 = f32le(&pkt[p+4]),  f2 = f32le(&pkt[p+8]);
        float f3 = f32le(&pkt[p+12]), f4 = f32le(&pkt[p+16]), f5 = f32le(&pkt[p+20]);
        std::cout << "  as 6xf32:   " << f0 << ", " << f1 << ", " << f2 << ", "
                                  << f3 << ", " << f4 << ", " << f5 << "\n";
        return;
    }

    // Generic fallback: show as u32 words
    if (need % 4 == 0) {
        std::cout << "  as u32 words: ";
        for (size_t off = 0; off < need; off += 4) {
            std::cout << u32le(&pkt[p + off]) << (off + 4 < need ? ", " : "");
        }
        std::cout << "\n";
    }

    // Also show as i32 words if divisible by 4
    if (need % 4 == 0) {
        std::cout << "  as i32 words: ";
        for (size_t off = 0; off < need; off += 4) {
            std::cout << (int32_t)u32le(&pkt[p + off]) << (off + 4 < need ? ", " : "");
        }
        std::cout << "\n";
    }
}
struct Header {
    std::vector<uint8_t> groupBytes;                   // raw group bytes (with MSB ext)
    std::vector<int> groupOffsets;                     // selected group offsets (0=Common,1=Time,...)
    std::vector<std::vector<uint16_t>> typeWords;      // per group: N typewords (N>=1)
    size_t payloadStart = 0;
};

static std::vector<int> groupOffsetsFromBytes(const std::vector<uint8_t>& gbytes) {
    std::vector<int> out;
    for (size_t i = 0; i < gbytes.size(); ++i) {
        uint8_t b = (uint8_t)(gbytes[i] & 0x7F); // ignore ext bit
        for (int bit = 0; bit < 7; ++bit) {
            if (b & (1u << bit))
                out.push_back((int)(i * 7 + bit));
        }
    }
    return out;
}

static Header parseHeader(const std::vector<uint8_t>& pkt) {
    if (pkt.size() < 2) throw std::runtime_error("too short");
    if (pkt[0] != 0xFA) throw std::runtime_error("no sync 0xFA");

    size_t i = 1;
    Header h;

    // ---- group bytes (1..4-ish) ----
    while (true) {
        if (i >= pkt.size()) throw std::runtime_error("truncated group bytes");
        uint8_t gb = pkt[i++];
        h.groupBytes.push_back(gb);
        if ((gb & 0x80) == 0) break;           // last group byte
        if (h.groupBytes.size() >= 8) break;   // safety
    }

    h.groupOffsets = groupOffsetsFromBytes(h.groupBytes);

    // ---- type words for each present group (1..N) ----
    for (size_t g = 0; g < h.groupOffsets.size(); ++g) {
        std::vector<uint16_t> tws;
        while (true) {
            if (i + 1 >= pkt.size()) throw std::runtime_error("truncated type word");
            uint16_t w = u16le(&pkt[i]);
            i += 2;
            tws.push_back(w);
            if ((w & 0x8000) == 0) break;      // last type word for this group
            if (tws.size() >= 8) break;        // safety
        }
        h.typeWords.push_back(tws);
    }

    h.payloadStart = i;
    return h;
}
enum class FieldKind {
    U8, U16, U32, U64,
    F32, F64,
    Vec3F32, Vec3F64
};

static size_t fieldSize(FieldKind k) {
    switch (k) {
        case FieldKind::U8:      return 1;
        case FieldKind::U16:     return 2;
        case FieldKind::U32:     return 4;
        case FieldKind::U64:     return 8;
        case FieldKind::F32:     return 4;
        case FieldKind::F64:     return 8;
        case FieldKind::Vec3F32: return 12;
        case FieldKind::Vec3F64: return 24;
    }
    throw std::runtime_error("unknown FieldKind");
}
struct FieldSpec {
    int typeOffset;          // offset inside this group (0..)
    const char* name;
    FieldKind kind;
};

static std::vector<int> enabledTypeOffsets(const std::vector<uint16_t>& typeWords) {
    std::vector<int> offsets;
    for (size_t w = 0; w < typeWords.size(); ++w) {
        uint16_t word = typeWords[w] & 0x7FFF; // clear extension bit
        for (int bit = 0; bit < 15; ++bit) {
            if (word & (1u << bit)) {
                offsets.push_back((int)(w * 15 + bit));
            }
        }
    }
    return offsets;
}






static void dumpHeader(const Header& h) {
    std::cout << "Groups present: ";
    for (auto g : h.groupOffsets) std::cout << g << " ";
    std::cout << "\n";

    for (size_t i = 0; i < h.groupOffsets.size(); ++i) {
        std::cout << "Group " << h.groupOffsets[i] << " typeOffsets: ";
        auto offs = enabledTypeOffsets(h.typeWords[i]);
        for (auto o : offs) std::cout << o << " ";
        std::cout << "\n";
    }
}
static size_t remainingFixedBytesAfter(
    const Header& h,
    size_t gi_start, size_t oi_start,
    const std::vector<uint8_t>& pkt,
    size_t payloadEnd)
{
    size_t sum = 0;
    for (size_t gi = gi_start; gi < h.groupOffsets.size(); ++gi) {
        int group = h.groupOffsets[gi];
        auto offs = enabledTypeOffsets(h.typeWords[gi]);

        for (size_t oi = 0; oi < offs.size(); ++oi) {
            if (gi == gi_start && oi <= oi_start) continue; // after current field

            int typeOffset = offs[oi];

            // We only count FIXED sizes here. Variable sizes count as 0.
            // Treat these as variable:
            bool variable =
                (group == 14) ||
                (group == GROUP_GNSS  && (typeOffset == 14 || typeOffset == 16)) ||
                (group == GROUP_GNSS2 && (typeOffset == 14 || typeOffset == 16));

            if (variable) continue;

            size_t need = typeSizeBytes(group, typeOffset);
            if (need == 0) continue; // unknown -> treat as variable too
            sum += need;
        }
    }
    return sum;
}
static uint16_t vn_crc16_ccitt(const uint8_t* data, size_t length)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc = (uint8_t)(crc >> 8) | (uint16_t)(crc << 8);
        crc ^= data[i];
        crc ^= (uint8_t)(crc & 0xFF) >> 4;
        crc ^= (uint16_t)(crc << 12);
        crc ^= (uint16_t)((crc & 0x00FF) << 5);
    }
    return crc;
}
static bool vn_verify_packet_crc16(const uint8_t* pkt, size_t n,
                                   uint16_t& computed, uint16_t& expected)
{
    if (n < 4) return false;
    if (pkt[0] != 0xFA) return false;

    // CRC is last 2 bytes in packet
    expected = ((uint16_t)pkt[n - 2] << 8) | (uint16_t)pkt[n - 1];

    // Compute CRC over bytes AFTER sync (exclude 0xFA), up to end of payload (exclude CRC itself)
    // Range: pkt[1 .. n-3] (length = n - 3)
    computed = vn_crc16_ccitt(&pkt[1], n - 3);

    return computed == expected;
} 
enum class PrintKind {
    SKIP_FIXED,      // known fixed size, but we won't parse (just dump raw)
    U8, U16, U32, U64,
    F32, F64,
    VEC3F32, VEC3F64,
    VAR_SATINFO,     // variable length
    VAR_RAWMEAS,     // variable length
    UNKNOWN_VAR      // unknown/variable -> infer skip
};

struct PrintSpec {
    const char* name;
    PrintKind kind;
    size_t fixedSize;   // for fixed kinds; 0 for variable/unknown
};

static PrintSpec specFor(int group, int typeOffset) {
    // COMMON (0)
    // COMMON (0)
    if (group == GROUP_COMMON) {
        switch (typeOffset) {
            case 0: return {"TimeStartup", PrintKind::U64, 8};
            case 3: return {"Ypr",         PrintKind::VEC3F32, 12};
            case 5: return {"AngularRate", PrintKind::VEC3F32, 12};
            case 6: return {"PosLla",      PrintKind::VEC3F64, 24};
            case 7: return {"VelNed",      PrintKind::VEC3F32, 12}; // ✅ add/keep this
            case 8: return {"Accel",       PrintKind::VEC3F32, 12};
            default: return {"Common(unknown-fixed)", PrintKind::SKIP_FIXED, typeSizeBytes(group, typeOffset)};
        }
    }

    // INS (5)
    if (group == GROUP_INS) {
        switch (typeOffset) {
            case 0: return {"InsStatus", PrintKind::U16, 2};
            case 1: return {"InsPosLla", PrintKind::VEC3F64, 24};
            case 4: return {"VelNed",    PrintKind::VEC3F32, 12};
            default: return {"INS(unknown-fixed)", PrintKind::SKIP_FIXED, typeSizeBytes(group, typeOffset)};
        }
    }

        // GNSS (3)
        // GNSS (3)
    if (group == GROUP_GNSS) {
        switch (typeOffset) {
            case 4:  return {"GnssFix",     PrintKind::U8, 1};
            case 14: return {"GnssSatInfo", PrintKind::VAR_SATINFO, 0};
            case 16: return {"GnssRawMeas", PrintKind::UNKNOWN_VAR, 0};;
            default: {
                size_t n = typeSizeBytes(group, typeOffset);
                if (n == 0) return {"GNSS(variable/unknown)", PrintKind::UNKNOWN_VAR, 0};
                return {"GNSS(fixed)", PrintKind::SKIP_FIXED, n};
            }
        }
    }

    // GNSS2 (6)
    if (group == GROUP_GNSS2) {
        switch (typeOffset) {
            case 14: return {"Gnss2SatInfo", PrintKind::VAR_SATINFO, 0};
            case 16: return {"Gnss2RawMeas", PrintKind::UNKNOWN_VAR, 0};
            default: {
                size_t n = typeSizeBytes(group, typeOffset);
                if (n == 0) return {"GNSS2(variable/unknown)", PrintKind::UNKNOWN_VAR, 0};
                return {"GNSS2(fixed)", PrintKind::SKIP_FIXED, n};
            }
        }
    }
    // ATT (4), TIME (1), etc: default to fixed skip if size known
   if (group == GROUP_ATT) {
        if (typeOffset == 0) {
            return {"ATT(reserved bit0)", PrintKind::SKIP_FIXED, 0}; // consume 0 bytes
        }
        size_t n = typeSizeBytes(group, typeOffset);
        if (n == 0) return {"ATT(unknown/var)", PrintKind::UNKNOWN_VAR, 0};
        return {"ATT(fixed)", PrintKind::SKIP_FIXED, n};
    }
    
    // Generic fallback for TIME, IMU, ATT, etc.
    size_t n = typeSizeBytes(group, typeOffset);
    if (n == 0) return {"Unknown(variable/unknown)", PrintKind::UNKNOWN_VAR, 0};
    return {"Unknown(fixed)", PrintKind::SKIP_FIXED, n};

}
static void skipVariableByInference(
    const Header& h,
    size_t gi, size_t oi,
    const std::vector<uint8_t>& pkt,
    size_t& p,
    size_t payloadEnd)
{
    // leave enough bytes for all remaining fixed-size fields after this one
    size_t mustLeave = remainingFixedBytesAfter(h, gi, oi, pkt, payloadEnd);

    if (p > payloadEnd || payloadEnd - p < mustLeave)
        throw std::runtime_error("Not enough bytes left to infer variable field size");

    size_t inferred = (payloadEnd - p) - mustLeave;

    std::cout << "  inferred variable len=" << inferred << " (mustLeave=" << mustLeave << ")\n";
    // dump a little bit (cap so console doesn't explode)
    std::cout << "  raw (first up to 64 bytes): ";
    dumpBytes(pkt, p, (inferred < 64 ? inferred : 64), payloadEnd);
    std::cout << "\n";

    p += inferred;
}
static void printAndAdvance(
    const Header& h,
    size_t gi, size_t oi,
    int group, int typeOffset,
    IMUData& out,
    const std::vector<uint8_t>& pkt,
    size_t& p,
    size_t payloadEnd)
{
    PrintSpec s = specFor(group, typeOffset);

    std::cout << "Group " << group << " Offset " << typeOffset
              << " @p=" << p << "  [" << s.name << "]\n";

    if (s.kind == PrintKind::UNKNOWN_VAR) {
        skipVariableByInference(h, gi, oi, pkt, p, payloadEnd);
        return;
    }

    // inside printAndAdvance(), replace the block:
if (s.kind == PrintKind::VAR_SATINFO) {
    skipGnssSatInfo(pkt, p, payloadEnd, s.name);
    return;
}
if (s.kind == PrintKind::VAR_RAWMEAS) {
    std::cout << "  RawMeas header bytes: ";
    dumpBytes(pkt, p, 12, payloadEnd);
    std::cout << "\n";

    double tow = f64le(&pkt[p+0]);
    uint16_t week = u16le(&pkt[p+8]);
    uint8_t numSats = pkt[p+10];

    std::cout << "  tow=" << tow << " week=" << week
            << " numSats=" << (unsigned)numSats << "\n";
    skipGnssRawMeas(pkt, p, payloadEnd, s.name);
    return;
}



    // Fixed-size paths
    size_t need = s.fixedSize;
   
    if (need == 0) {
        // If size is 0, treat as "no payload" and just return.
        std::cout << "  (no payload bytes)\n";
        return;
    }

    requireBytes(p, need, payloadEnd, s.name);

    std::cout << "  raw: ";
    dumpBytes(pkt, p, need, payloadEnd);
    std::cout << "\n";

    // Interpret and optionally fill out IMUData for the ones you care about
    switch (s.kind) {
        case PrintKind::U8: {
            uint8_t v = pkt[p];
            std::cout << "  val(u8): " << (unsigned)v << "\n";
            p += 1;
        } break;

        case PrintKind::U16: {
            uint16_t v = u16le(&pkt[p]);
            std::cout << "  val(u16): " << v << "\n";
            // store if INS status
            if (group == GROUP_INS && typeOffset == 0) {
                out.insStatus = v;
                out.hasInsStatus = true;
            }
            p += 2;
        } break;

        case PrintKind::U32: {
            uint32_t v = u32le(&pkt[p]);
            std::cout << "  val(u32): " << v << "\n";
            p += 4;
        } break;

        case PrintKind::U64: {
            uint64_t v = u64le(&pkt[p]);
            std::cout << "  val(u64): " << v << "\n";
            if (group == GROUP_COMMON && typeOffset == 0) {
                out.timeStartup = v;
                out.hasTimeStartup = true;
            }
            p += 8;
        } break;

        case PrintKind::F32: {
            float v = f32le(&pkt[p]);
            std::cout << "  val(f32): " << v << "\n";
            p += 4;
        } break;

        case PrintKind::F64: {
            double v = f64le(&pkt[p]);
            std::cout << "  val(f64): " << v << "\n";
            p += 8;
        } break;

        case PrintKind::VEC3F32: {
            float a = f32le(&pkt[p+0]);
            float b = f32le(&pkt[p+4]);
            float c = f32le(&pkt[p+8]);
            std::cout << "  val(vec3f32): " << a << ", " << b << ", " << c << "\n";

            if (group == GROUP_COMMON && typeOffset == 3) {
                out.yaw = a; out.pitch = b; out.roll = c;
                out.hasYpr = true;
            }
            if (group == GROUP_COMMON && typeOffset == 5) {
                out.angRateX = a; out.angRateY = b; out.angRateZ = c;
                out.hasAngularRate = true;
            }
            if (group == GROUP_COMMON && typeOffset == 8) {
                out.accelX = a; out.accelY = b; out.accelZ = c;
                out.hasAccel = true;
            }
            if (group == GROUP_COMMON && typeOffset == 7) {
                out.velN = a; out.velE = b; out.velD = c;
                out.hasVelNed = true;
            }

            p += 12;
        } break;

        case PrintKind::VEC3F64: {
            double a = f64le(&pkt[p+0]);
            double b = f64le(&pkt[p+8]);
            double c = f64le(&pkt[p+16]);
            std::cout << "  val(vec3f64): " << a << ", " << b << ", " << c << "\n";

            if (group == GROUP_COMMON && typeOffset == 6) {
                out.lat = a; out.lon = b; out.alt = c;
                out.hasPosLla = true;
            }

            p += 24;
        } break;

        case PrintKind::SKIP_FIXED:
            printUnknownFixedInterpretations(pkt, p, need, payloadEnd);
            p += need;
            break;

        default:
            // shouldn’t happen because variable handled above
            throw std::runtime_error("Unhandled PrintKind");
    }
}
static IMUData decodePacket_CommonAndINS(const uint8_t* data, size_t len)
{
    std::vector<uint8_t> pkt(data, data + len);
    size_t trailerLen = 0;
    uint16_t crcCalc=0, crcExp=0;
    if (vn_verify_packet_crc16(pkt.data(), pkt.size(), crcCalc, crcExp)) {
        trailerLen = 2;
    }
    size_t payloadEnd = pkt.size() - trailerLen;
    Header h = parseHeader(pkt);
    dumpHeader(h);

    IMUData out;
    size_t p = h.payloadStart;

    // payloadEnd excludes CRC


    for (size_t gi = 0; gi < h.groupOffsets.size(); ++gi) {
        int group = h.groupOffsets[gi];
        auto offsets = enabledTypeOffsets(h.typeWords[gi]);

        for (size_t oi = 0; oi < offsets.size(); ++oi) {
            int typeOffset = offsets[oi];

            printAndAdvance(h, gi, oi, group, typeOffset,
                            out, pkt, p, payloadEnd);
        }
    }
    if (p != payloadEnd) {
    std::cout << "WARNING: p != payloadEnd  p=" 
              << p << "  payloadEnd=" << payloadEnd << "\n";
    } else {
        std::cout << "OK: p == payloadEnd (" << p << ")\n";
    }
    return out;
}

// Convenience wrapper for your raw buffer
static IMUData decodePacketGeneric(const uint8_t* data, size_t len)
{
    return decodePacket_CommonAndINS(data, len);
}

bool decodeVNPacket(const uint8_t* data, size_t len, IMUData& out)
{
    try {
        out = decodePacket_CommonAndINS(data, len);  // your throwing decoder
        return true;
    } catch (const std::exception&) {
        return false;
    } catch (...) {
        return false;
    }
}
void testbinary(){



    //Serial.println("Starting IMU Binary Test...");


    uint8_t debugBuffer[] = {
  0xFA, 0x01, 0xE9, 0x01, 0x28, 0x0B, 0x07, 0x4D,
  0xB9, 0x00, 0x00, 0x00, 0xE8, 0xF4, 0x9C, 0x42,
  0xB1, 0xEF, 0x0F, 0x41, 0xA9, 0x49, 0xCE, 0xC2,
  0x70, 0x0B, 0x9D, 0x39, 0xE1, 0x34, 0x0D, 0x3B,
  0x8C, 0x46, 0x9E, 0xBB, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xBF, 0x9A, 0xC6, 0x3F, 0xCE, 0x8E, 0x17, 0x41,
  0xD8, 0xD2, 0x0B, 0x40, 0x24, 0xCC
};
    uint16_t crcCalc = 0, crcExp = 0;
    bool ok = vn_verify_packet_crc16(debugBuffer, sizeof(debugBuffer), crcCalc, crcExp);

    std::cout << "CRC expected: 0x" << std::hex << crcExp
            << "  computed: 0x" << crcCalc
            << "  match=" << std::boolalpha << ok
            << std::dec << "\n";

    if (!ok) {
        std::cout << "CRC FAIL: packet may be truncated or checksum mode may not be CRC16.\n";
        // You can choose to return early here:
        // return;
    }
    // 2. Call your parsing function
    try {
        IMUData d = decodePacket_CommonAndINS(debugBuffer, sizeof(debugBuffer));

        if (d.hasTimeStartup)  std::cout << "TimeStartup: " << d.timeStartup << "\n";
        if (d.hasYpr)          std::cout << "YPR: " << d.yaw << ", " << d.pitch << ", " << d.roll << "\n";
        if (d.hasAngularRate)  std::cout << "AngularRate: " << d.angRateX << ", " << d.angRateY << ", " << d.angRateZ << "\n";
        if (d.hasAccel)        std::cout << "Accel: " << d.accelX << ", " << d.accelY << ", " << d.accelZ << "\n";
        if (d.hasPosLla)       std::cout << "PosLLA: " << d.lat << ", " << d.lon << ", " << d.alt << "\n";
        if (d.hasVelNed)       std::cout << "VelNed: " << d.velN << ", " << d.velE << ", " << d.velD << "\n";
        if (d.hasInsStatus)    std::cout << "InsStatus: " << d.insStatus << "\n";
    }
    catch (const std::exception& e) {
        std::cout << "Decode error: " << e.what() << "\n";
    }
}
void sendUARTCommand(const char* command, unsigned int timeout_ms){
    // Send the command over UART
    Serial1.print(command);
    
    // Wait for response with timeout
    unsigned long startTime = millis();
    std::string response = "";
    
    // Read response until timeout
    while(millis() - startTime < timeout_ms){
        if(Serial1.available() > 0){
            char c = Serial1.read();
            response += c;
            
            // Check if we have at least 2 characters for checksum
            if(response.length() >= 2){
                // Reset timeout on each character received
                startTime = millis();
                
                if(!Serial1.available()){
                    break; // No more data coming
                }
            }
        }
    }
}
bool attachIMUTriggerISR(uint8_t pin, void (*isr)()){
    if (digitalPinToInterrupt(pin) == NOT_AN_INTERRUPT) {
        return false;
    }

    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), isr, RISING);
    return true;
}
