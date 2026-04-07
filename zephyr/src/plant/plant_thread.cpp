// zephyr/src/plant/plant_thread.cpp
// Phase 4 — 10 ms timer-driven plant loop.
//
// Timer fires every 10 ms → gives semaphore → plant_thread wakes,
// reads g_cmd, steps PlantModel, writes g_state, triggers CAN TX.
//
// Priority 5 — below CAN RX (3), above HTTP (10) and LED (12).
// Stack 16384 B — PlantModel has heavy subsystems + Dugoff tire math.
//
// CAN RX watchdog: if g_last_rx_t is stale by > CAN_RX_TIMEOUT_S,
// g_cmd is reset to safe defaults (system_enable=false, torque=0).

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "plant/plant_model.hpp"
#include "sim/actuator_cmd.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// ── Shared state (defined in main.cpp) ───────────────────────────────────────
extern plant::PlantState  g_state;
extern sim::ActuatorCmd   g_cmd;
extern struct k_mutex     g_state_mutex;
extern struct k_mutex     g_cmd_mutex;
extern volatile uint32_t  g_can_tx_count;
extern volatile double    g_last_rx_t;
extern double             g_surface_mu;

// ── CAN TX trigger (defined in can_tx.cpp) ───────────────────────────────────
extern void can_tx_send_all(const plant::PlantState& s, double t_s,
                            uint32_t loop_us);

// ── Timing ───────────────────────────────────────────────────────────────────
static constexpr double   DT_S             = 0.01;    // 10 ms step
static constexpr double   CAN_RX_TIMEOUT_S = 0.5;    // 500 ms watchdog

// ── Heavy-Duty Electric Vehicle plant params (mirrors vehicle_config.cpp without STL) ────────
static plant::PlantModelParams vehicle_params()
{
    plant::PlantModelParams p;

    // Geometry
    p.wheelbase_m             = 6.30;
    p.track_width_m           = 7.20;
    p.geometry.cg_height_m    = 3.20;
    p.geometry.cg_to_front_m  = 2.52;
    p.geometry.cg_to_rear_m   = 3.78;
    p.geometry.yaw_inertia_kgm2 = 8500000.0;

    // Drivetrain
    p.drive.mass_kg              = 218000.0;
    p.drive.wheel_radius_m       = 1.93;
    p.drive.motor_torque_max_nm  = 145000.0;
    p.drive.motor_power_max_w    = 2013000.0;
    p.drive.gear_ratio           = 28.0;
    p.drive.drivetrain_eff       = 0.92;
    p.drive.brake_torque_max_nm  = 2500000.0; // 2.5 MNm → ~0.6g at full load (218 t); tyre limit ~3 MNm
    p.drive.regen_eff_active     = 0.65;
    p.drive.regen_eff_coast      = 0.02;
    p.drive.drag_c               = 1.85;
    p.drive.roll_c               = 9500.0;
    p.drive.v_max_mps            = 17.78;
    p.drive.v_stop_eps           = 0.5;

    // Motor
    p.motor_params.max_power_kW  = 2013.0;
    p.motor_params.max_torque_nm = 145000.0;
    p.motor_params.efficiency    = 0.92;

    // Battery
    p.battery_params.capacity_kWh            = 1650.0;
    p.battery_params.nominal_voltage_v       = 1200.0;
    p.battery_params.max_charge_power_kW     = 600.0;
    p.battery_params.max_discharge_power_kW  = 2400.0;
    p.battery_params.efficiency_charge       = 0.91;
    p.battery_params.efficiency_discharge    = 0.93;
    p.battery_params.min_soc                 = 0.18;
    p.battery_params.max_soc                 = 0.88;

    // Dynamic Dugoff tire model enabled
    p.dynamic_config.enabled    = true;
    p.dynamic_config.surface_mu = 0.72;

    return p;
}

// ── Static PlantModel instance (lives in .bss, not on any thread stack) ──────
static plant::PlantModel  s_plant{vehicle_params()};

// ── 10 ms periodic timer ──────────────────────────────────────────────────────
K_SEM_DEFINE(plant_sem, 0, 1);

static void plant_timer_expiry(struct k_timer*) { k_sem_give(&plant_sem); }
K_TIMER_DEFINE(plant_timer, plant_timer_expiry, NULL);

// ── Plant thread ──────────────────────────────────────────────────────────────
static void plant_thread(void*, void*, void*)
{
    LOG_INF("[plant] Heavy-Duty Electric Vehicle plant thread started (dt=10 ms, prio=5)");

    // Apply initial surface mu from the shell-settable global
    {
        auto p = s_plant.params();
        p.dynamic_config.surface_mu = g_surface_mu;
        s_plant.set_params(p);
    }

    k_timer_start(&plant_timer, K_MSEC(10), K_MSEC(10));

    while (true) {
        k_sem_take(&plant_sem, K_FOREVER);

        const uint32_t t0_cycles = k_cycle_get_32();
        const double   t_s       = k_uptime_get_32() / 1000.0;

        // ── Propagate shell-settable surface_mu into plant ────────────────────
        // Only update when changed — set_params rebuilds all subsystems.
        {
            static double last_mu = 0.0;
            if (g_surface_mu != last_mu) {
                last_mu = g_surface_mu;
                auto p = s_plant.params();
                p.dynamic_config.surface_mu = g_surface_mu;
                s_plant.set_params(p);
            }
        }

        // ── Read actuation command ────────────────────────────────────────────
        sim::ActuatorCmd cmd;
        k_mutex_lock(&g_cmd_mutex, K_FOREVER);
        cmd = g_cmd;
        k_mutex_unlock(&g_cmd_mutex);

        // ── CAN RX watchdog — safe mode if no frame for 500 ms ────────────────
        if (g_last_rx_t > 0.0 && (t_s - g_last_rx_t) > CAN_RX_TIMEOUT_S) {
            cmd.reset();
        }

        // ── Step plant physics ────────────────────────────────────────────────
        plant::PlantState local_state;
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        local_state = g_state;
        k_mutex_unlock(&g_state_mutex);

        s_plant.step(local_state, cmd, DT_S);

        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_state = local_state;
        k_mutex_unlock(&g_state_mutex);

        // ── Brake diagnostics — log every 100 ms while braking ───────────────
        // Shows: vx, measured a_long, total Fx from tyre model, tau_brake applied,
        //        and per-wheel omega vs reference omega (vx/R).
        // Lets us verify: is slip building up? Is kinematic/dynamic switch active?
        if (cmd.brake_cmd_pct > 1.0) {
            static int brake_log_ctr = 0;
            if ((++brake_log_ctr % 10) == 0) {
                const double Fx_total = local_state.Fx_fl + local_state.Fx_fr
                                      + local_state.Fx_rl + local_state.Fx_rr;
                const double tau_applied = local_state.tau_brake_fl_nm
                                         + local_state.tau_brake_fr_nm
                                         + local_state.tau_brake_rl_nm
                                         + local_state.tau_brake_rr_nm;
                const double omega_ref = local_state.v_mps / 1.93;
                LOG_INF("[brk] t=%.2f vx=%.3f a=%.3f Fx=%.0fN tau=%.0fNm mode=%s",
                        t_s,
                        local_state.v_mps,
                        local_state.a_long_mps2,
                        Fx_total,
                        tau_applied,
                        local_state.dynamic_model_enabled ? "dyn" : "kin");
                LOG_INF("[brk] omega FL=%.2f FR=%.2f RL=%.2f RR=%.2f ref=%.2f rad/s",
                        local_state.omega_fl_radps, local_state.omega_fr_radps,
                        local_state.omega_rl_radps, local_state.omega_rr_radps,
                        omega_ref);
            }
        }

        // ── CAN TX ────────────────────────────────────────────────────────────
        const uint32_t dt_cycles = k_cycle_get_32() - t0_cycles;
        const uint32_t loop_us   = k_cyc_to_us_ceil32(dt_cycles);

        can_tx_send_all(local_state, t_s, loop_us);
    }
}

K_THREAD_DEFINE(plant_tid, 16384, plant_thread, NULL, NULL, NULL, 5, 0, 0);
