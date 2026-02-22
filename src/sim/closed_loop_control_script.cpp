// src/sim/closed_loop_control_script.cpp
//
// Closed-loop speed + steer controller for the vehicle dynamics simulator.
//
// Connects to vcan0, listens to plant-state CAN frames, and sends
// ACTUATOR_CMD_1 commands following a phase state machine.
//
// Phase sequence:
//   WAIT_CAN   → first CAN frame received
//   ACCEL      → positive torque until v ≥ target*(1-TOL)            [speed-based exit]
//   CRUISE     → hold speed, steer=0, for CRUISE_DUR_S               [time-based exit]
//   STEER_L    → hold speed, steer=STEER_LEFT_DEG, for STEER_DUR_S  [time-based exit]
//   SETTLE1    → hold speed, steer=0, for SETTLE_DUR_S               [time-based exit]
//   STEER_R    → hold speed, steer=STEER_RIGHT_DEG, for STEER_DUR_S [time-based exit]
//   SETTLE2    → hold speed, steer=0, for SETTLE_DUR_S               [time-based exit]
//   BRAKE_STOP → gear=FORWARD, brake=100%, until |v| < 0.1 m/s       [speed-based exit]
//   REV_ACCEL  → gear=REVERSE, negative torque, until v ≤ -spd_rev   [speed-based exit]
//   REV_STEER  → hold reverse speed, steer=REV_STEER_DEG             [time-based exit]
//   REV_SETTLE → hold reverse speed, steer=0                          [time-based exit]
//   REV_BRAKE  → gear=REVERSE, brake=100%, until |v| < 0.1 m/s       [speed-based exit]
//   FWD2_ACCEL → gear=FORWARD, torque until v ≥ TARGET_FWD2*(1-TOL)  [speed-based exit]
//   FWD2_COAST → zero torque & brake, coast to 0 via drag+rolling    [speed or time exit]
//   STOPPED    → send zeros and exit
//
// Usage:
//   ./build/src/sim/closed_loop_control_script [vcan_iface] [can_map_path]

#include "can/can_codec.hpp"
#include "can/can_map.hpp"
#include "can/socketcan_iface.hpp"
#include "sim/controller_schedule.hpp"
#include "utils/logging.hpp"

#include <chrono>
#include <csignal>
#include <cstring>
#include <linux/can.h>
#include <thread>

// ---------------------------------------------------------------------------
// Signal handler
// ---------------------------------------------------------------------------
static volatile std::sig_atomic_t g_stop = 0;
static void on_sigint(int) { g_stop = 1; }

// ---------------------------------------------------------------------------
using Clock = std::chrono::steady_clock;

static double elapsed_s(const Clock::time_point& t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

// ---------------------------------------------------------------------------
// Phase state machine
// ---------------------------------------------------------------------------
enum class Phase {
    WAIT_CAN,    // No CAN data received yet
    ACCEL,       // Accelerate forward to target speed
    CRUISE,      // Hold speed straight (settle before first steer)
    STEER_L,     // Hold speed, steer left
    SETTLE1,     // Hold speed, steer=0 (settle after left steer)
    STEER_R,     // Hold speed, steer right
    SETTLE2,     // Hold speed, steer=0 (settle after right steer)
    BRAKE_STOP,  // Brake to full stop from forward (gear=FORWARD)
    REV_ACCEL,   // Accelerate backward (gear=REVERSE, negative torque)
    REV_STEER,   // Hold reverse speed + steer (U-turn)
    REV_SETTLE,  // Hold reverse speed, steer=0 (settle before brake)
    REV_BRAKE,   // Brake to full stop from reverse (gear=REVERSE)
    FWD2_ACCEL,  // Accelerate forward to 10 km/h target
    FWD2_COAST,  // Release everything — coast to 0 via drag + rolling resistance
    STOPPED      // Done — send zeros and exit
};

static const char* phase_name(Phase p)
{
    switch (p) {
        case Phase::WAIT_CAN:   return "WAIT_CAN";
        case Phase::ACCEL:      return "ACCEL";
        case Phase::CRUISE:     return "CRUISE";
        case Phase::STEER_L:    return "STEER_L";
        case Phase::SETTLE1:    return "SETTLE1";
        case Phase::STEER_R:    return "STEER_R";
        case Phase::SETTLE2:    return "SETTLE2";
        case Phase::BRAKE_STOP: return "BRAKE_STOP";
        case Phase::REV_ACCEL:  return "REV_ACCEL";
        case Phase::REV_STEER:  return "REV_STEER";
        case Phase::REV_SETTLE: return "REV_SETTLE";
        case Phase::REV_BRAKE:  return "REV_BRAKE";
        case Phase::FWD2_ACCEL: return "FWD2_ACCEL";
        case Phase::FWD2_COAST: return "FWD2_COAST";
        case Phase::STOPPED:    return "STOPPED";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    std::signal(SIGINT,  on_sigint);
    std::signal(SIGTERM, on_sigint);

    utils::set_level(utils::LogLevel::Info);

    const char* ifname   = (argc > 1) ? argv[1] : controller::CAN_INTERFACE;
    const char* map_path = (argc > 2) ? argv[2] : controller::CAN_MAP_PATH;

    // -----------------------------------------------------------------------
    // Load CAN map
    // -----------------------------------------------------------------------
    can::CanMap can_map;
    if (!can_map.load(map_path)) {
        LOG_ERROR("Failed to load CAN map: %s", map_path);
        return 1;
    }

    // Find TX command frame (we write to this)
    const can::FrameDef* cmd_frame_def = nullptr;
    for (const auto& kv : can_map.rx_frames()) {
        if (kv.second.frame_name == controller::CMD_FRAME_NAME) {
            cmd_frame_def = &kv.second;
            break;
        }
    }
    if (!cmd_frame_def) {
        LOG_ERROR("Frame '%s' not found in CAN map RX frames", controller::CMD_FRAME_NAME);
        return 1;
    }

    // Plant-state frames we decode
    // 0x300: vehicle_speed_mps, vehicle_accel_mps2, yaw_rate_radps
    // 0x310: motor_torque_nm
    // 0x320: brake_force_kn
    // 0x330: pos_x_m, pos_y_m
    // 0x331: yaw_deg
    const std::vector<uint32_t> rx_ids = {0x300, 0x310, 0x320, 0x330, 0x331};

    std::unordered_map<uint32_t, const can::FrameDef*> rx_defs;
    for (const auto& kv : can_map.tx_frames()) {
        for (uint32_t id : rx_ids) {
            if (kv.frame_id == id)
                rx_defs[id] = &kv;
        }
    }

    // -----------------------------------------------------------------------
    // Open SocketCAN
    // -----------------------------------------------------------------------
    can::SocketCanIface iface;
    if (!iface.open(ifname)) {
        LOG_ERROR("Failed to open CAN interface: %s", ifname);
        return 1;
    }

    // -----------------------------------------------------------------------
    // Derived constants
    // -----------------------------------------------------------------------
    static constexpr double MOTOR_TQ_MAX_NM = 9500.0;

    // Forward primary
    const double tq_fwd    = MOTOR_TQ_MAX_NM * controller::TORQUE_FRACTION;
    const double spd_high  = controller::TARGET_SPEED_MPS * (1.0 + controller::TOLERANCE);
    const double spd_low   = controller::TARGET_SPEED_MPS * (1.0 - controller::TOLERANCE);

    // Reverse (negative torque command → backward traction)
    const double tq_rev       = -MOTOR_TQ_MAX_NM * controller::TORQUE_FRACTION_REV;
    const double spd_rev_low  = controller::TARGET_SPEED_REV_MPS * (1.0 - controller::TOLERANCE);
    const double spd_rev_high = controller::TARGET_SPEED_REV_MPS * (1.0 + controller::TOLERANCE);

    // Second forward run
    const double tq_fwd2   = MOTOR_TQ_MAX_NM * controller::TORQUE_FRACTION_FWD2;
    const double spd2_low  = controller::TARGET_SPEED_FWD2_MPS * (1.0 - controller::TOLERANCE);
    const double spd2_high = controller::TARGET_SPEED_FWD2_MPS * (1.0 + controller::TOLERANCE);

    LOG_INFO("=================================================");
    LOG_INFO(" Closed-loop controller started");
    LOG_INFO("  Interface   : %s", ifname);
    LOG_INFO("  CAN map     : %s", map_path);
    LOG_INFO("  FWD target  : %.1f m/s (%.0f km/h), deadband %.1f–%.1f m/s",
             controller::TARGET_SPEED_MPS, controller::TARGET_SPEED_MPS * 3.6, spd_low, spd_high);
    LOG_INFO("  REV target  : %.1f m/s (%.0f km/h)",
             controller::TARGET_SPEED_REV_MPS, controller::TARGET_SPEED_REV_MPS * 3.6);
    LOG_INFO("  FWD2 target : %.2f m/s (10 km/h)", controller::TARGET_SPEED_FWD2_MPS);
    LOG_INFO("  Torque      : fwd=%.0f Nm  rev=%.0f Nm  fwd2=%.0f Nm",
             tq_fwd, tq_rev, tq_fwd2);
    LOG_INFO("  Brake %%     : %.0f%%", controller::BRAKE_PCT_DECEL);
    LOG_INFO("  Steer       : L=%.1f°  R=%.1f°  Rev=%.1f°",
             controller::STEER_LEFT_DEG, controller::STEER_RIGHT_DEG, controller::REV_STEER_DEG);
    LOG_INFO("=================================================");

    // -----------------------------------------------------------------------
    // Received plant state
    // -----------------------------------------------------------------------
    double v_mps          = 0.0;
    double a_mps2         = 0.0;
    double yaw_rate_radps = 0.0;
    double yaw_deg        = 0.0;
    double motor_torque   = 0.0;
    double brake_force_kn = 0.0;
    double pos_x          = 0.0;
    double pos_y          = 0.0;
    double last_rx_t      = -1e9;

    // -----------------------------------------------------------------------
    // Phase state
    // -----------------------------------------------------------------------
    Phase  phase       = Phase::WAIT_CAN;
    Phase  prev_phase  = Phase::WAIT_CAN;
    double phase_start = 0.0;

    auto t_start = Clock::now();
    auto next_tx = Clock::now();
    int  log_ctr = 0;

    const double dt_cmd_s = 1.0 / controller::CMD_HZ;

    // -----------------------------------------------------------------------
    // Main loop
    // -----------------------------------------------------------------------
    while (!g_stop) {
        // Drain all available RX frames (non-blocking)
        struct can_frame frame{};
        while (iface.read_nonblocking(frame)) {
            const uint32_t id = frame.can_id & CAN_SFF_MASK;
            auto it = rx_defs.find(id);
            if (it == rx_defs.end()) continue;

            auto sigs = can::CanCodec::decode_to_map(*it->second, frame);
            last_rx_t  = elapsed_s(t_start);

            auto get = [&](const char* name, double& out) {
                auto s = sigs.find(name);
                if (s != sigs.end()) out = s->second;
            };

            if (id == 0x300) {
                get("vehicle_speed_mps",  v_mps);
                get("vehicle_accel_mps2", a_mps2);
                get("yaw_rate_radps",     yaw_rate_radps);
            }
            if (id == 0x310) { get("motor_torque_nm", motor_torque); }
            if (id == 0x320) { get("brake_force_kn",  brake_force_kn); }
            if (id == 0x330) { get("pos_x_m", pos_x); get("pos_y_m", pos_y); }
            if (id == 0x331) { get("yaw_deg",  yaw_deg); }
        }

        // Wait until next TX slot
        auto now = Clock::now();
        if (now < next_tx) {
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            continue;
        }
        next_tx += std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(dt_cmd_s));

        const double t_wall      = elapsed_s(t_start);
        const double phase_elapsed = t_wall - phase_start;
        const bool   safe_mode   = (last_rx_t < 0.0) ||
                                   ((t_wall - last_rx_t) > controller::CAN_TIMEOUT_S);

        // -------------------------------------------------------------------
        // Phase transitions
        // -------------------------------------------------------------------
        switch (phase) {

        case Phase::WAIT_CAN:
            if (!safe_mode) { phase = Phase::ACCEL; phase_start = t_wall; }
            break;

        case Phase::ACCEL:
            if (v_mps >= spd_low) { phase = Phase::CRUISE; phase_start = t_wall; }
            break;

        case Phase::CRUISE:
            if (phase_elapsed >= controller::CRUISE_DUR_S)
                { phase = Phase::STEER_L; phase_start = t_wall; }
            break;

        case Phase::STEER_L:
            if (phase_elapsed >= controller::STEER_DUR_S)
                { phase = Phase::SETTLE1; phase_start = t_wall; }
            break;

        case Phase::SETTLE1:
            if (phase_elapsed >= controller::SETTLE_DUR_S)
                { phase = Phase::STEER_R; phase_start = t_wall; }
            break;

        case Phase::STEER_R:
            if (phase_elapsed >= controller::STEER_DUR_S)
                { phase = Phase::SETTLE2; phase_start = t_wall; }
            break;

        case Phase::SETTLE2:
            if (phase_elapsed >= controller::SETTLE_DUR_S)
                { phase = Phase::BRAKE_STOP; phase_start = t_wall; }
            break;

        case Phase::BRAKE_STOP:
            // Exit to reverse once fully stopped (or timeout 30 s)
            if (std::abs(v_mps) < 0.1 || phase_elapsed > 30.0)
                { phase = Phase::REV_ACCEL; phase_start = t_wall; }
            break;

        case Phase::REV_ACCEL:
            // v_mps becomes negative; exit when magnitude reaches spd_rev_low
            if (v_mps <= -spd_rev_low || phase_elapsed > 30.0)
                { phase = Phase::REV_STEER; phase_start = t_wall; }
            break;

        case Phase::REV_STEER:
            if (phase_elapsed >= controller::REV_STEER_DUR_S)
                { phase = Phase::REV_SETTLE; phase_start = t_wall; }
            break;

        case Phase::REV_SETTLE:
            if (phase_elapsed >= controller::REV_SETTLE_DUR_S)
                { phase = Phase::REV_BRAKE; phase_start = t_wall; }
            break;

        case Phase::REV_BRAKE:
            // Exit to second forward run once fully stopped (or timeout 30 s)
            if (std::abs(v_mps) < 0.1 || phase_elapsed > 30.0)
                { phase = Phase::FWD2_ACCEL; phase_start = t_wall; }
            break;

        case Phase::FWD2_ACCEL:
            if (v_mps >= spd2_low || phase_elapsed > 30.0)
                { phase = Phase::FWD2_COAST; phase_start = t_wall; }
            break;

        case Phase::FWD2_COAST:
            // Done when coasted to near-zero or max coast duration exceeded
            if (v_mps < 0.1 || phase_elapsed >= controller::FWD2_COAST_MAX_S)
                { phase = Phase::STOPPED; phase_start = t_wall; }
            break;

        case Phase::STOPPED:
            g_stop = 1;
            break;
        }

        // Log phase transitions
        if (phase != prev_phase) {
            LOG_INFO("[t=%5.1fs] *** Phase: %s → %s | v=%.2f m/s ***",
                     t_wall, phase_name(prev_phase), phase_name(phase), v_mps);
            prev_phase = phase;
        }

        // -------------------------------------------------------------------
        // Compute actuator commands for current phase
        // -------------------------------------------------------------------
        double drive_tq  = 0.0;
        double brake_pct = 0.0;
        double steer_deg = 0.0;
        double gear      = 1.0;   // FORWARD by default

        if (safe_mode || phase == Phase::WAIT_CAN || phase == Phase::STOPPED) {
            // All zeros — safe hold / done
            gear = 0.0;
        }
        else if (phase == Phase::BRAKE_STOP) {
            // Full brake, gear stays FORWARD — gear clamp stops vx at 0
            brake_pct = controller::BRAKE_PCT_DECEL;
        }
        else if (phase == Phase::REV_ACCEL) {
            // Switch to REVERSE gear, apply negative torque
            gear = 2.0;  // REVERSE
            if (v_mps > -spd_rev_low) {
                drive_tq = tq_rev;   // negative → backward traction
            } else if (v_mps < -spd_rev_high) {
                brake_pct = controller::BRAKE_PCT_DECEL;
            }
            // else coast
        }
        else if (phase == Phase::REV_STEER) {
            // Hold reverse speed with bang-bang + steer
            gear = 2.0;
            if (v_mps > -spd_rev_low) {
                drive_tq = tq_rev;
            } else if (v_mps < -spd_rev_high) {
                brake_pct = controller::BRAKE_PCT_DECEL;
            }
            steer_deg = controller::REV_STEER_DEG;
        }
        else if (phase == Phase::REV_SETTLE) {
            // Hold reverse speed, return steer to 0
            gear = 2.0;
            if (v_mps > -spd_rev_low) {
                drive_tq = tq_rev;
            } else if (v_mps < -spd_rev_high) {
                brake_pct = controller::BRAKE_PCT_DECEL;
            }
        }
        else if (phase == Phase::REV_BRAKE) {
            // Full brake in REVERSE — tests that gear clamp stops at 0 without going forward
            gear = 2.0;
            brake_pct = controller::BRAKE_PCT_DECEL;
        }
        else if (phase == Phase::FWD2_ACCEL) {
            // Accelerate forward to 10 km/h
            if (v_mps < spd2_low) {
                drive_tq = tq_fwd2;
            } else if (v_mps > spd2_high) {
                brake_pct = controller::BRAKE_PCT_DECEL;
            }
        }
        else if (phase == Phase::FWD2_COAST) {
            // Release everything — drag + rolling resistance take over
            // drive_tq=0, brake_pct=0
        }
        else {
            // Forward bang-bang (ACCEL, CRUISE, STEER_*, SETTLE_*)
            if (v_mps < spd_low) {
                drive_tq = tq_fwd;
            } else if (v_mps > spd_high) {
                brake_pct = controller::BRAKE_PCT_DECEL;
            }
            switch (phase) {
            case Phase::STEER_L: steer_deg = controller::STEER_LEFT_DEG;  break;
            case Phase::STEER_R: steer_deg = controller::STEER_RIGHT_DEG; break;
            default:             steer_deg = 0.0;                          break;
            }
        }

        // -------------------------------------------------------------------
        // Encode and send ACTUATOR_CMD_1
        // -------------------------------------------------------------------
        can::SignalMap vals;
        vals["system_enable"]       = (safe_mode || phase == Phase::WAIT_CAN ||
                                       phase == Phase::STOPPED) ? 0.0 : 1.0;
        vals["gear_position"]       = gear;
        vals["mode"]                = 0.0;
        vals["steer_cmd_deg"]       = steer_deg;
        vals["drive_torque_cmd_nm"] = drive_tq;
        vals["brake_cmd_pct"]       = brake_pct;

        struct can_frame tx_frame{};
        can::CanCodec::encode_from_map(*cmd_frame_def, vals, tx_frame);
        iface.write_frame(tx_frame);

        // -------------------------------------------------------------------
        // Periodic log (every 2 s)
        // -------------------------------------------------------------------
        if (++log_ctr % static_cast<int>(controller::CMD_HZ * 2.0) == 0) {
            const char* spd_phase =
                (drive_tq > 0.0)   ? "ACCEL" :
                (drive_tq < 0.0)   ? "REV"   :
                (brake_pct > 0.0)  ? "BRAKE" : "COAST";

            LOG_INFO("[t=%5.1fs|%-10s] %s | v=%+6.2f m/s (%+5.1f km/h) | "
                     "steer=%+6.1f° | yaw=%+7.2f° | r=%+.3f rad/s | "
                     "x=%6.0f y=%6.0f | tq=%+8.0f Nm | brk=%.0f%% | F=%5.1f kN%s",
                     t_wall, phase_name(phase), spd_phase,
                     v_mps, v_mps * 3.6,
                     steer_deg, yaw_deg, yaw_rate_radps,
                     pos_x, pos_y,
                     motor_torque, brake_pct, brake_force_kn,
                     safe_mode ? " [SAFE]" : "");
        }
    }

    // -----------------------------------------------------------------------
    // Final zero command
    // -----------------------------------------------------------------------
    can::SignalMap zero;
    zero["system_enable"]       = 0.0;
    zero["gear_position"]       = 0.0;
    zero["mode"]                = 0.0;
    zero["steer_cmd_deg"]       = 0.0;
    zero["drive_torque_cmd_nm"] = 0.0;
    zero["brake_cmd_pct"]       = 0.0;
    struct can_frame stop_frame{};
    can::CanCodec::encode_from_map(*cmd_frame_def, zero, stop_frame);
    for (int i = 0; i < 5; ++i) iface.write_frame(stop_frame);

    LOG_INFO("Controller stopped. Final phase: %s | v=%.3f m/s",
             phase_name(phase), v_mps);
    return 0;
}
