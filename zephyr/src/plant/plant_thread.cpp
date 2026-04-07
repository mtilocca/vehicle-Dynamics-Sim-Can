// zephyr/src/plant/plant_thread.cpp
// Phase 4 — 10 ms timer-driven plant loop.
//
// Timer fires every 10 ms → gives semaphore → plant_thread wakes,
// reads g_cmd, steps PlantModel, writes g_state, triggers CAN TX.
//
// Priority 5 — below CAN RX (3), above HTTP (10) and LED (12).
// Stack 16384 B — PlantModel has heavy subsystems + Dugoff tire math.
//
// CAN RX watchdog: uses cmd.last_update_t_s (written under g_cmd_mutex).
// Hardware watchdog: stroked via g_wdt_sem given to watchdog_thread every step.
// Loop overrun: warns if step takes > 9.5 ms; stores worst-case in g_sys_stats.

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "plant/plant_model.hpp"
#include "sim/actuator_cmd.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// ── Shared state (defined in main.cpp) ───────────────────────────────────────
extern plant::PlantState  g_state;
extern sim::ActuatorCmd   g_cmd;
extern struct k_mutex     g_state_mutex;
extern struct k_mutex     g_cmd_mutex;
extern atomic_t           g_can_tx_count;
extern double             g_surface_mu;

// ── System stats (defined in main.cpp) ───────────────────────────────────────
struct SysStats {
    uint32_t plant_loop_us_max;
    uint32_t can_rx_total;
    uint32_t can_timeout_total;
    size_t   heap_used;
    size_t   heap_free;
};
extern SysStats      g_sys_stats;
extern struct k_mutex g_stats_mutex;

// ── Watchdog semaphore (defined in main.cpp) ──────────────────────────────────
extern struct k_sem g_wdt_sem;

// ── CAN TX trigger (defined in can_tx.cpp) ───────────────────────────────────
extern void can_tx_send_all(const plant::PlantState& s, double t_s,
                            uint32_t loop_us);

// ── Timing constants ──────────────────────────────────────────────────────────
static constexpr double   DT_S              = 0.01;   // 10 ms step
static constexpr double   CAN_RX_TIMEOUT_S = 0.5;    // 500 ms watchdog
static constexpr uint32_t OVERRUN_WARN_US  = 9500;   // warn above 9.5 ms

// ── Vehicle params (mirrors vehicle_config.cpp without STL) ───────────────────
static plant::PlantModelParams vehicle_params()
{
    plant::PlantModelParams p;

    p.wheelbase_m             = 6.30;
    p.track_width_m           = 7.20;
    p.geometry.cg_height_m    = 3.20;
    p.geometry.cg_to_front_m  = 2.52;
    p.geometry.cg_to_rear_m   = 3.78;
    p.geometry.yaw_inertia_kgm2 = 8500000.0;

    p.drive.mass_kg              = 218000.0;
    p.drive.wheel_radius_m       = 1.93;
    p.drive.motor_torque_max_nm  = 145000.0;
    p.drive.motor_power_max_w    = 2013000.0;
    p.drive.gear_ratio           = 28.0;
    p.drive.drivetrain_eff       = 0.92;
    p.drive.brake_torque_max_nm  = 2500000.0;
    p.drive.regen_eff_active     = 0.65;
    p.drive.regen_eff_coast      = 0.02;
    p.drive.drag_c               = 1.85;
    p.drive.roll_c               = 9500.0;
    p.drive.v_max_mps            = 17.78;
    p.drive.v_stop_eps           = 0.5;

    p.motor_params.max_power_kW  = 2013.0;
    p.motor_params.max_torque_nm = 145000.0;
    p.motor_params.efficiency    = 0.92;

    p.battery_params.capacity_kWh            = 1650.0;
    p.battery_params.nominal_voltage_v       = 1200.0;
    p.battery_params.max_charge_power_kW     = 600.0;
    p.battery_params.max_discharge_power_kW  = 2400.0;
    p.battery_params.efficiency_charge       = 0.91;
    p.battery_params.efficiency_discharge    = 0.93;
    p.battery_params.min_soc                 = 0.18;
    p.battery_params.max_soc                 = 0.88;

    p.dynamic_config.enabled    = true;
    p.dynamic_config.surface_mu = 0.72;

    return p;
}

static plant::PlantModel s_plant{vehicle_params()};

// ── 10 ms periodic timer ──────────────────────────────────────────────────────
K_SEM_DEFINE(plant_sem, 0, 1);

static void plant_timer_expiry(struct k_timer*) { k_sem_give(&plant_sem); }
K_TIMER_DEFINE(plant_timer, plant_timer_expiry, NULL);

// ── Plant thread ──────────────────────────────────────────────────────────────
static void plant_thread(void*, void*, void*)
{
    LOG_INF("[plant] Heavy-Duty Electric Vehicle plant thread started (dt=10 ms, prio=5)");

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

        // ── Surface mu propagation ────────────────────────────────────────────
        {
            static double last_mu = 0.0;
            if (g_surface_mu != last_mu) {
                last_mu = g_surface_mu;
                auto p = s_plant.params();
                p.dynamic_config.surface_mu = g_surface_mu;
                s_plant.set_params(p);
            }
        }

        // ── Read actuation command (timestamp inside g_cmd under mutex) ───────
        sim::ActuatorCmd cmd;
        k_mutex_lock(&g_cmd_mutex, K_FOREVER);
        cmd = g_cmd;
        k_mutex_unlock(&g_cmd_mutex);

        // ── CAN RX watchdog — safe mode if no fresh command for 500 ms ───────
        // last_update_t_s is written under g_cmd_mutex; we read it via the
        // local copy so the check is race-free.
        if (cmd.last_update_t_s > 0.0 &&
            (t_s - cmd.last_update_t_s) > CAN_RX_TIMEOUT_S) {
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

        // ── Brake diagnostics ────────────────────────────────────────────────
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
                        t_s, local_state.v_mps, local_state.a_long_mps2,
                        Fx_total, tau_applied,
                        local_state.dynamic_model_enabled ? "dyn" : "kin");
                LOG_INF("[brk] omega FL=%.2f FR=%.2f RL=%.2f RR=%.2f ref=%.2f rad/s",
                        local_state.omega_fl_radps, local_state.omega_fr_radps,
                        local_state.omega_rl_radps, local_state.omega_rr_radps,
                        omega_ref);
            }
        }

        // ── Loop timing + overrun detection ───────────────────────────────────
        const uint32_t dt_cycles = k_cycle_get_32() - t0_cycles;
        const uint32_t loop_us   = k_cyc_to_us_ceil32(dt_cycles);

        if (loop_us > OVERRUN_WARN_US) {
            LOG_WRN("[plant] loop overrun: %u us (limit %u us)", loop_us, OVERRUN_WARN_US);
        }

        // Update worst-case in sys_stats
        k_mutex_lock(&g_stats_mutex, K_FOREVER);
        if (loop_us > g_sys_stats.plant_loop_us_max) {
            g_sys_stats.plant_loop_us_max = loop_us;
        }
        k_mutex_unlock(&g_stats_mutex);

        // ── CAN TX ────────────────────────────────────────────────────────────
        can_tx_send_all(local_state, t_s, loop_us);

        // ── Stroke hardware watchdog via watchdog_thread semaphore ────────────
        k_sem_give(&g_wdt_sem);
    }
}

K_THREAD_DEFINE(plant_tid, 16384, plant_thread, NULL, NULL, NULL, 5, 0, 0);
