#include <Arduino.h>

/*
* Aliases for uBlox defined types to make defining packets from the interface
* descriptions easier.
*/
using U1 = uint8_t;
using I1 = int8_t;
using X1 = uint8_t;
using U2 = uint16_t;
using I2 = int16_t;
using X2 = uint16_t;
using U4 = uint32_t;
using I4 = int32_t;
using X4 = uint32_t;
using R4 = float;
using R8 = double;
using CH = char;

/* Classes */
static constexpr uint8_t UBX_ACK_CLS_ = 0x05;
static constexpr uint8_t UBX_CFG_CLS_ = 0x06;
static constexpr uint8_t UBX_INF_CLS_ = 0x04;
static constexpr uint8_t UBX_LOG_CLS_ = 0x21;
static constexpr uint8_t UBX_MGA_CLS_ = 0x13;
static constexpr uint8_t UBX_MON_CLS_ = 0x0a;
static constexpr uint8_t UBX_NAV_CLS_ = 0x01;
static constexpr uint8_t UBX_RXM_CLS_ = 0x02;
static constexpr uint8_t UBX_SEC_CLS_ = 0x27;
static constexpr uint8_t UBX_TIM_CLS_ = 0x0d;
static constexpr uint8_t UBX_UPD_CLS_ = 0x09;

/* UBX-NAV IDs */
static constexpr uint8_t UBX_NAV_CLOCK_ID_ = 0x22;
static constexpr uint8_t UBX_NAV_DOP_ID_ = 0x04;
static constexpr uint8_t UBX_NAV_EOE_ID_ = 0x61;
static constexpr uint8_t UBX_NAV_GEOFENCE_ID_ = 0x39;
static constexpr uint8_t UBX_NAV_HPPOSECEF_ID_ = 0x13;
static constexpr uint8_t UBX_NAV_HPPOSLLH_ID_ = 0x14;
static constexpr uint8_t UBX_NAV_ODO_ID_ = 0x09;
static constexpr uint8_t UBX_NAV_ORB_ID_ = 0x34;
static constexpr uint8_t UBX_NAV_POSECEF_ID_ = 0x01;
static constexpr uint8_t UBX_NAV_POSLLH_ID_ = 0x02;
static constexpr uint8_t UBX_NAV_PVT_ID_ = 0x07;
static constexpr uint8_t UBX_NAV_RELPOSNED_ID_ = 0x3c;
static constexpr uint8_t UBX_NAV_RESETODO_ID_ = 0x10;
static constexpr uint8_t UBX_NAV_SAT_ID_ = 0x35;
static constexpr uint8_t UBX_NAV_SBAS_ID_ = 0x32;
static constexpr uint8_t UBX_NAV_SIG_ID_ = 0x43;
static constexpr uint8_t UBX_NAV_SLAS_ID_ = 0x42;
static constexpr uint8_t UBX_NAV_STATUS_ID_ = 0x03;
static constexpr uint8_t UBX_NAV_SVIN_ID_ = 0x3b;
static constexpr uint8_t UBX_NAV_TIMEBDS_ID_ = 0x24;
static constexpr uint8_t UBX_NAV_TIMEGAL_ID_ = 0x25;
static constexpr uint8_t UBX_NAV_TIMEGLO_ID_ = 0x23;
static constexpr uint8_t UBX_NAV_TIMEGPS_ID_ = 0x20;
static constexpr uint8_t UBX_NAV_TIMELS_ID_ = 0x26;
static constexpr uint8_t UBX_NAV_TIMEQZSS_ID_ = 0x27;
static constexpr uint8_t UBX_NAV_TIMEUTC_ID_ = 0x21;
static constexpr uint8_t UBX_NAV_VELECEF_ID_ = 0x11;
static constexpr uint8_t UBX_NAV_VELNED_ID_ = 0x12;


const unsigned char UBX_HEADER[]        = { 0xB5, 0x62 };
const unsigned char NAV_POSLLH_HEADER[] = { 0x01, 0x02 };
const unsigned char NAV_STATUS_HEADER[] = { 0x01, 0x03 };
const unsigned char NAV_PVT_HEADER[]    = { 0x01, 0x07 };

enum _ubxMsgType {
  MT_NONE,
  MT_NAV_POSLLH,
  MT_NAV_STATUS,
  MT_NAV_PVT
};

struct UbxNavPosllh {
  U1 cls;
  U1 id;
  U2 len;
  U4 i_tow;   // GPS time of week, ms
  I4 lon;     // Longitude, deg, scale 1e-7
  I4 lat;     // Latitude, deg, scale 1e-7
  I4 height;  // Height above ellipsoid, mm
  I4 h_msl;   // Height above MSL, mm
  U4 h_acc;   // Horizontal accuracy estimate, mm
  U4 v_acc;   // Vertical accuracy estimate, mm
};

struct UbxNavStatus {
  U1 cls;
  U1 id;
  U2 len;
  U4 i_tow;       // GPS time of week, ms
  U1 gps_fix;     // GPS fix type
  X1 flags;       // Navigation status flags
  X1 fix_stat;    // Fix status info
  X1 flags2;      // Further info about nav output
  U4 ttff;        // Time to first fix, ms
  U4 msss;        // Milliseconds since startup, ms
};

struct UbxNavPvt {
  U1 cls;
  U1 id;
  U2 len;
  U4 i_tow;       // GPS time of week, ms
  U2 year;        // Year (UTC)
  U1 month;       // Month (UTC)
  U1 day;         // Day (UTC)
  U1 hour;        // Hour (UTC)
  U1 min;         // Minute (UTC)
  U1 sec;         // Seconds (UTC)
  X1 valid;       // Validity flags
  U4 t_acc;       // Time accuracy estimate (UTC), ns
  I4 nano;        // Fraction of second (UTC), ns
  U1 fix_type;    // GNSS fix type
  X1 flags;       // Fix status flags
  U1 reserved1;   // Additional flags
  U1 num_sv;      // Number of satellites used in nav solution
  I4 lon;         // Longitude, deg, scale 1e-7
  I4 lat;         // Latitude, deg, scale 1e-7
  I4 height;      // Height above the ellipsoid, mm
  I4 h_msl;       // Height above MSL, mm
  U4 h_acc;       // Horizontal accuracy estimate, mm
  U4 v_acc;       // Vertical accuracy estimate, mm
  I4 vel_n;       // NED north velocity, mm/s
  I4 vel_e;       // NED east velocity, mm/s
  I4 vel_d;       // NED down velocity, mm/s
  I4 g_speed;     // Ground speed (2D), mm/s
  I4 head_mot;    // Heading of motion (2D), deg, scale 1e-5
  U4 s_acc;       // Speed accuracy estimate, mm/s
  U4 head_acc;    // Heading accuracy estimate, deg, scale 1e-5
  U2 p_dop;       // Position DOP, scale 0.01
  X2 reserved2;   // Additional flags
  U4 reserved3;   // Additional flags
  // those following lines are not in the Description of der Version 14 Values for NEO M6 / M7 NAV_PVT
  //I4 head_veh;    // Heading of vehicle (2D), deg, scale 1e-5
  //I2 mag_dec;     // Magnetic declination, deg, scale 1e-2
  //U2 mag_acc;     // Magnetic declination accuracy, deg, scale 1e-2
};

union UBXMessage {
  UbxNavPvt     NAV_PVT;    
  UbxNavPosllh  NAV_POSLLH;
  UbxNavStatus  NAV_STATUS;
};

int processGPS(HardwareSerial* SerialGps, UBXMessage* ubxMessage);
void SendMemToGps(char GpsConf[], int l, HardwareSerial* SerialGps);
String compute_Locator(double latitude, double longitude);
