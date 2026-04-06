// zephyr/src/can/can_tx.cpp
// Phase 4 CAN TX — encodes 15 TX frames from PlantState and sends via FDCAN1.
//
// Called by plant_thread every 10 ms. Each frame has its own cycle counter;
// frames are only sent when their period has elapsed (rate limiting).
//
// Signal encoding: Intel byte order (LSB first), clamp then scale by factor.
// No heap, no std::string. All frame IDs are 29-bit (no CAN_EFF_FLAG).

#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <math.h>   // floor, fmin, fmax

#include "plant/plant_main/plant_state.hpp"

LOG_MODULE_DECLARE(xcmg_sim, LOG_LEVEL_INF);

extern volatile uint32_t g_can_tx_count;

// ── Encoding helpers ──────────────────────────────────────────────────────────

static inline double clamp(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// Pack a signed integer into a little-endian CAN payload.
static inline void pack_i16(uint8_t* data, int byte, int16_t v)
{
    data[byte]   = (uint8_t)(v & 0xFF);
    data[byte+1] = (uint8_t)((v >> 8) & 0xFF);
}
static inline void pack_u16(uint8_t* data, int byte, uint16_t v)
{
    data[byte]   = (uint8_t)(v & 0xFF);
    data[byte+1] = (uint8_t)((v >> 8) & 0xFF);
}
static inline void pack_i32(uint8_t* data, int byte, int32_t v)
{
    data[byte]   = (uint8_t)(v & 0xFF);
    data[byte+1] = (uint8_t)((v >> 8) & 0xFF);
    data[byte+2] = (uint8_t)((v >> 16) & 0xFF);
    data[byte+3] = (uint8_t)((v >> 24) & 0xFF);
}
static inline void pack_u32(uint8_t* data, int byte, uint32_t v)
{
    data[byte]   = (uint8_t)(v & 0xFF);
    data[byte+1] = (uint8_t)((v >> 8) & 0xFF);
    data[byte+2] = (uint8_t)((v >> 16) & 0xFF);
    data[byte+3] = (uint8_t)((v >> 24) & 0xFF);
}

// Encode a physical value to raw integer: raw = round(phys / factor)
static inline int16_t  enc_i16(double phys, double factor)
{
    double r = phys / factor;
    return (int16_t)(int64_t)clamp(r, -32768.0, 32767.0);
}
static inline uint16_t enc_u16(double phys, double factor)
{
    double r = phys / factor;
    return (uint16_t)(int64_t)clamp(r, 0.0, 65535.0);
}
static inline int32_t  enc_i32(double phys, double factor)
{
    double r = phys / factor;
    return (int32_t)(int64_t)clamp(r, -2147483648.0, 2147483647.0);
}
static inline uint32_t enc_u32(double phys, double factor)
{
    double r = phys / factor;
    return (uint32_t)(int64_t)clamp(r, 0.0, 4294967295.0);
}

// ── Send one Zephyr CAN frame ─────────────────────────────────────────────────

static void tx_send(const struct device* dev, uint32_t id29,
                    const uint8_t* data, uint8_t dlc)
{
    struct can_frame zf{};
    zf.id    = id29;
    zf.flags = CAN_FRAME_IDE;
    zf.dlc   = dlc;
    for (int i = 0; i < dlc; ++i) zf.data[i] = data[i];

    int r = can_send(dev, &zf, K_MSEC(5), nullptr, nullptr);
    if (r == 0) {
        g_can_tx_count++;
    } else {
        // Don't log every miss — FDCAN TX FIFO can fill up under load
    }
}

// ── TX frame encoders ─────────────────────────────────────────────────────────
// Signal positions taken from can_map_static.hpp (auto-generated from DBC).
// All frames are DLC=8, Intel byte order.

// 0x0CFF0028  IMU_ACC  5 ms
// imu_ax_mps2  [0:15]  i16 factor 0.01
// imu_ay_mps2  [16:31] i16 factor 0.01
// imu_az_mps2  [32:47] i16 factor 0.01
// imu_temp_c   [48:63] i16 factor 0.01
static void encode_imu_acc(uint8_t* d, const plant::PlantState& s)
{
    pack_i16(d, 0, enc_i16(s.a_long_mps2, 0.01));
    pack_i16(d, 2, enc_i16(s.a_lat_mps2,  0.01));
    pack_i16(d, 4, enc_i16(0.0,           0.01));  // az — not modelled
    pack_i16(d, 6, enc_i16(25.0,          0.01));  // temp — constant
}

// 0x0CFF0128  IMU_GYR  5 ms
// imu_gx_rps  [0:15]  i16 factor 0.001
// imu_gy_rps  [16:31] i16 factor 0.001
// imu_gz_rps  [32:47] i16 factor 0.001
// imu_status  [48:55] u8  factor 1.0
static void encode_imu_gyr(uint8_t* d, const plant::PlantState& s)
{
    pack_i16(d, 0, enc_i16(0.0,               0.001)); // gx (roll)
    pack_i16(d, 2, enc_i16(0.0,               0.001)); // gy (pitch)
    pack_i16(d, 4, enc_i16(s.yaw_rate_radps,  0.001)); // gz (yaw)
    d[6] = 0x01; // status OK
    d[7] = 0;
}

// 0x18FF1029  GNSS_LL  100 ms
// gnss_lat_deg [0:31]  i32 factor 1e-7
// gnss_lon_deg [32:63] i32 factor 1e-7
static void encode_gnss_ll(uint8_t* d, const plant::PlantState& s)
{
    // Use x_m/y_m as flat-earth offsets from origin (0,0) = (0 lat, 0 lon)
    // 1 deg lat ≈ 111000 m
    pack_i32(d, 0, enc_i32(s.y_m / 111000.0, 1e-7));
    pack_i32(d, 4, enc_i32(s.x_m / 111000.0, 1e-7));
}

// 0x18FF1129  GNSS_AV  100 ms
// gnss_alt_m      [0:15]  i16 factor 0.1 offset -1000
// gnss_vn_mps     [16:31] i16 factor 0.01
// gnss_ve_mps     [32:47] i16 factor 0.01
// gnss_fix_type   [48:55] u8  factor 1.0
// gnss_sat_count  [56:63] u8  factor 1.0
static void encode_gnss_av(uint8_t* d, const plant::PlantState& s)
{
    // altitude = 0 m → raw = (0 - (-1000)) / 0.1 = 10000
    pack_i16(d, 0, (int16_t)10000);
    // vn/ve from yaw + speed
    double vn = s.v_mps * cos(s.yaw_rad);
    double ve = s.v_mps * sin(s.yaw_rad);
    pack_i16(d, 2, enc_i16(vn, 0.01));
    pack_i16(d, 4, enc_i16(ve, 0.01));
    d[6] = 3;   // 3D fix
    d[7] = 12;  // 12 satellites
}

// 0x0CFF202A  WHEELS_1  10 ms
// wheel_fl_rps [0:15]  i16 factor 0.01
// wheel_fr_rps [16:31] i16 factor 0.01
// wheel_rl_rps [32:47] i16 factor 0.01
// wheel_rr_rps [48:63] i16 factor 0.01
static void encode_wheels_1(uint8_t* d, const plant::PlantState& s)
{
    pack_i16(d, 0, enc_i16(s.omega_fl_radps, 0.01));
    pack_i16(d, 2, enc_i16(s.omega_fr_radps, 0.01));
    pack_i16(d, 4, enc_i16(s.omega_rl_radps, 0.01));
    pack_i16(d, 6, enc_i16(s.omega_rr_radps, 0.01));
}

// 0x0CFF212A  STEER_STATE  10 ms
// steer_deg      [0:15]   i16 factor 0.1
// steer_rate_dps [16:31]  i16 factor 0.1
// delta_fl_deg   [32:43]  i12 factor 0.1  (12-bit signed)
// delta_fr_deg   [44:55]  i12 factor 0.1  (12-bit signed)
// steer_fault    [56:63]  u8  factor 1.0
static void encode_steer_state(uint8_t* d, const plant::PlantState& s)
{
    static const double R2D = 180.0 / 3.14159265358979323846;
    double steer_deg      = s.steer_virtual_rad * R2D;
    double steer_rate_dps = s.steer_rate_radps  * R2D;
    double delta_fl_deg   = s.delta_fl_rad       * R2D;
    double delta_fr_deg   = s.delta_fr_rad       * R2D;

    pack_i16(d, 0, enc_i16(steer_deg,      0.1));
    pack_i16(d, 2, enc_i16(steer_rate_dps, 0.1));

    // 12-bit signed fields share bytes 4–6
    int16_t fl_raw = (int16_t)clamp(delta_fl_deg / 0.1, -2048.0, 2047.0);
    int16_t fr_raw = (int16_t)clamp(delta_fr_deg / 0.1, -2048.0, 2047.0);
    // bits [32:43] = bytes 4 (bits 0-7) + byte 5 (bits 0-3)
    d[4] = (uint8_t)(fl_raw & 0xFF);
    d[5] = (uint8_t)(((fl_raw >> 8) & 0x0F) | ((fr_raw & 0x0F) << 4));
    d[6] = (uint8_t)((fr_raw >> 4) & 0xFF);
    d[7] = 0; // no fault
}

// 0x18FF302B  BATT_STATE  50 ms
// batt_v       [0:15]  u16 factor 0.1
// batt_i       [16:31] i16 factor 0.1
// batt_soc_pct [32:39] u8  factor 0.5
// batt_temp_c  [40:47] u8  factor 1.0 offset -40
// batt_power_kw[48:63] i16 factor 0.1
static void encode_batt_state(uint8_t* d, const plant::PlantState& s)
{
    pack_u16(d, 0, enc_u16(s.batt_v, 0.1));
    pack_i16(d, 2, enc_i16(s.batt_i, 0.1));
    d[4] = (uint8_t)clamp(s.batt_soc_pct / 0.5, 0.0, 200.0);
    d[5] = (uint8_t)clamp(s.batt_temp_c + 40.0, 0.0, 255.0);
    double power_kw = (s.batt_v * s.batt_i) / 1000.0;
    pack_i16(d, 6, enc_i16(power_kw, 0.1));
}

// 0x18FF402C  RADAR_1  50 ms  (not modelled — send zeros)
static void encode_radar_1(uint8_t* d)
{
    for (int i = 0; i < 8; ++i) d[i] = 0;
    d[6] = 0x01; // status = no target
}

// 0x18FF50F0  VEHICLE_STATE_1  10 ms
// vehicle_speed_mps   [0:15]  i16 factor 0.01
// vehicle_accel_mps2  [16:31] i16 factor 0.01
// yaw_rate_radps      [32:47] i16 factor 0.0001
// status_flags        [48:55] u8  factor 1.0
static void encode_vehicle_state_1(uint8_t* d, const plant::PlantState& s)
{
    pack_i16(d, 0, enc_i16(s.v_mps,           0.01));
    pack_i16(d, 2, enc_i16(s.a_long_mps2,     0.01));
    pack_i16(d, 4, enc_i16(s.yaw_rate_radps,  0.0001));
    d[6] = (uint8_t)(s.status_flags & 0xFF);
    d[7] = 0;
}

// 0x18FF51F0  MOTOR_STATE_1  10 ms
// motor_torque_nm  [0:15]  i16 factor 10.0
// motor_power_kw   [16:31] i16 factor 0.1
// motor_speed_rpm  [32:47] u16 factor 1.0
// motor_temp_c     [48:55] u8  factor 1.0 offset -40
static void encode_motor_state_1(uint8_t* d, const plant::PlantState& s)
{
    // Derive motor speed from rear-wheel angular speed + gear ratio
    // wheel_speed (rad/s) × gear_ratio → motor speed (rad/s) → RPM
    const double gear_ratio = 28.0;
    double omega_w = (s.omega_rl_radps + s.omega_rr_radps) * 0.5;
    double motor_rpm = (omega_w * gear_ratio) * (60.0 / (2.0 * 3.14159265358979));

    pack_i16(d, 0, enc_i16(s.motor_torque_nm,   10.0));
    pack_i16(d, 2, enc_i16(s.motor_power_kW,    0.1));
    pack_u16(d, 4, enc_u16(motor_rpm,           1.0));
    d[6] = (uint8_t)clamp(25.0 + 40.0, 0.0, 255.0); // 25°C motor temp
    d[7] = 0;
}

// 0x18FF52F0  BRAKE_STATE  10 ms
// brake_force_kn   [0:15]  u16 factor 0.01
// brake_pct_actual [16:23] u8  factor 0.5
// regen_power_kw   [24:39] u16 factor 0.1
// brake_temp_c     [40:47] u8  factor 1.0 offset -40
static void encode_brake_state(uint8_t* d, const plant::PlantState& s)
{
    pack_u16(d, 0, enc_u16(s.brake_force_kN,  0.01));
    d[2] = (uint8_t)clamp(0.0 / 0.5, 0.0, 200.0); // brake_pct — not in state yet
    pack_u16(d, 3, enc_u16(s.regen_power_kW,  0.1));
    d[5] = (uint8_t)clamp(25.0 + 40.0, 0.0, 255.0); // brake temp
    d[6] = 0;
    d[7] = 0;
}

// 0x18FF53F0  POSITION_STATE  50 ms
// pos_x_m [0:31]  i32 factor 0.01
// pos_y_m [32:63] i32 factor 0.01
static void encode_position_state(uint8_t* d, const plant::PlantState& s)
{
    pack_i32(d, 0, enc_i32(s.x_m, 0.01));
    pack_i32(d, 4, enc_i32(s.y_m, 0.01));
}

// 0x18FF54F0  ORIENTATION_STATE  50 ms
// yaw_deg      [0:15]  i16 factor 0.01
// yaw_rad      [16:31] i16 factor 0.001
// yaw_rate_dps [32:47] i16 factor 0.1
static void encode_orientation_state(uint8_t* d, const plant::PlantState& s)
{
    static const double R2D = 180.0 / 3.14159265358979323846;
    pack_i16(d, 0, enc_i16(s.yaw_rad * R2D,           0.01));
    pack_i16(d, 2, enc_i16(s.yaw_rad,                 0.001));
    pack_i16(d, 4, enc_i16(s.yaw_rate_radps * R2D,    0.1));
    d[6] = 0;
    d[7] = 0;
}

// 0x18FF55F0  DRIVETRAIN_STATE  100 ms  (static vehicle params)
// gear_ratio         [0:15]  u16 factor 0.01
// drivetrain_eff_pct [16:23] u8  factor 0.5
// wheel_radius_mm    [24:39] u16 factor 1.0
// wheelbase_mm       [40:55] u16 factor 1.0
// track_width_mm     [56:63] u8  factor 100.0
static void encode_drivetrain_state(uint8_t* d)
{
    pack_u16(d, 0, enc_u16(28.0,  0.01));   // gear ratio
    d[2] = (uint8_t)(92.0 / 0.5);           // 92% efficiency
    pack_u16(d, 3, enc_u16(1930.0, 1.0));   // wheel radius mm
    pack_u16(d, 5, enc_u16(6300.0, 1.0));   // wheelbase mm
    d[7] = (uint8_t)(7200.0 / 100.0);       // track width (7200 mm → 72)
}

// 0x18FF60F0  DIAGNOSTIC_STATE  100 ms
// sim_time_s    [0:31]  u32 factor 0.001
// loop_time_us  [32:47] u16 factor 1.0
// error_count   [48:55] u8  factor 1.0
// status        [56:63] u8  factor 1.0
static void encode_diagnostic_state(uint8_t* d, const plant::PlantState& s,
                                    uint32_t loop_us)
{
    pack_u32(d, 0, enc_u32(s.t_s, 0.001));
    pack_u16(d, 4, (uint16_t)(loop_us < 65535u ? loop_us : 65535u));
    d[6] = 0; // error_count
    d[7] = 0x01; // status = running
}

// ── Rate-limiting counters ────────────────────────────────────────────────────
// Each frame's counter increments every 10 ms. Frame sent when counter >= period.

static uint8_t ctr_imu        = 0;  // 5 ms  → every 1st call  (but 10ms min)
static uint8_t ctr_gnss       = 0;  // 100ms → every 10 calls
static uint8_t ctr_wheels     = 0;  // 10ms  → every call
static uint8_t ctr_steer      = 0;
static uint8_t ctr_batt       = 0;  // 50ms  → every 5 calls
static uint8_t ctr_radar      = 0;
static uint8_t ctr_vehicle    = 0;  // 10ms
static uint8_t ctr_motor      = 0;
static uint8_t ctr_brake      = 0;
static uint8_t ctr_pos        = 0;  // 50ms  → every 5 calls
static uint8_t ctr_orient     = 0;
static uint8_t ctr_drivetrain = 0;  // 100ms → every 10 calls
static uint8_t ctr_diag       = 0;

// ── Public entry point called by plant_thread ─────────────────────────────────

void can_tx_send_all(const plant::PlantState& s, double t_s, uint32_t loop_us)
{
    (void)t_s;

    const struct device* dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
    if (!device_is_ready(dev)) return;

    uint8_t d[8];

    // IMU_ACC / IMU_GYR — 5 ms (send every call since plant step is 10ms)
    // Nominally 5ms, but we send at the plant rate (10ms) — close enough.
    encode_imu_acc(d, s);
    tx_send(dev, 0x0CFF0028u, d, 8);

    encode_imu_gyr(d, s);
    tx_send(dev, 0x0CFF0128u, d, 8);

    // WHEELS_1 + STEER_STATE — 10 ms
    encode_wheels_1(d, s);
    tx_send(dev, 0x0CFF202Au, d, 8);

    encode_steer_state(d, s);
    tx_send(dev, 0x0CFF212Au, d, 8);

    // VEHICLE_STATE_1 + MOTOR_STATE_1 + BRAKE_STATE — 10 ms
    encode_vehicle_state_1(d, s);
    tx_send(dev, 0x18FF50F0u, d, 8);

    encode_motor_state_1(d, s);
    tx_send(dev, 0x18FF51F0u, d, 8);

    encode_brake_state(d, s);
    tx_send(dev, 0x18FF52F0u, d, 8);

    // BATT_STATE + RADAR_1 — 50 ms (every 5 calls)
    if (++ctr_batt >= 5) {
        ctr_batt = 0;
        encode_batt_state(d, s);
        tx_send(dev, 0x18FF302Bu, d, 8);

        encode_radar_1(d);
        tx_send(dev, 0x18FF402Cu, d, 8);

        // POSITION_STATE + ORIENTATION_STATE — 50 ms
        encode_position_state(d, s);
        tx_send(dev, 0x18FF53F0u, d, 8);

        encode_orientation_state(d, s);
        tx_send(dev, 0x18FF54F0u, d, 8);
    }

    // GNSS_LL + GNSS_AV + DRIVETRAIN_STATE + DIAGNOSTIC_STATE — 100 ms
    if (++ctr_gnss >= 10) {
        ctr_gnss = 0;

        encode_gnss_ll(d, s);
        tx_send(dev, 0x18FF1029u, d, 8);

        encode_gnss_av(d, s);
        tx_send(dev, 0x18FF1129u, d, 8);

        encode_drivetrain_state(d);
        tx_send(dev, 0x18FF55F0u, d, 8);

        encode_diagnostic_state(d, s, loop_us);
        tx_send(dev, 0x18FF60F0u, d, 8);
    }
}
