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
#include "state/sim_state.hpp"
#include "state/control_bus.hpp"
#include "state/system_health.hpp"
#include "utils/mutex_guard.hpp"
#include "utils/placement_new.hpp"
#include "config/vehicle_config_zephyr.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// ── Shared state buses (defined in main.cpp) ─────────────────────────────────
extern hdv::SimStateBus      g_sim_bus;
extern hdv::ControlBus       g_ctrl_bus;
extern hdv::SystemHealthBus  g_health_bus;
extern struct k_mutex        g_sim_plant_mtx;
extern struct k_mutex        g_sim_cmd_mtx;
extern struct k_mutex        g_health_mtx;

// ── Watchdog semaphore (defined in main.cpp, used when CONFIG_WATCHDOG=y) ─────
#ifdef CONFIG_WATCHDOG
extern struct k_sem g_wdt_sem;
#endif

// ── CAN TX trigger (defined in can_tx.cpp) ───────────────────────────────────
extern void can_tx_send_all(const plant::PlantState& s, double t_s,
                            uint32_t loop_us);

// ── Timing constants ──────────────────────────────────────────────────────────
static constexpr double   DT_S              = 0.01;   // 10 ms step
static constexpr double   CAN_RX_TIMEOUT_S = 0.5;    // 500 ms watchdog
static constexpr uint32_t OVERRUN_WARN_US  = 9500;   // warn above 9.5 ms


// ── PlantModel storage — placement-new into a static buffer ──────────────────
// Construction (~95 ms) is deferred to thread start so watchdog_thread can arm
// and feed the IWDG first. File-scope construction ran before any threads and
// starved the IWDG. Function-local static requires __cxa_guard_acquire which
// Zephyr's newlib doesn't provide.
alignas(plant::PlantModel) static uint8_t s_plant_buf[sizeof(plant::PlantModel)];
static plant::PlantModel* s_plant = nullptr;

// ── 10 ms periodic timer ──────────────────────────────────────────────────────
K_SEM_DEFINE(plant_sem, 0, 1);

static void plant_timer_expiry(struct k_timer*) { k_sem_give(&plant_sem); }
K_TIMER_DEFINE(plant_timer, plant_timer_expiry, NULL);

// ── Plant thread ──────────────────────────────────────────────────────────────
static void plant_thread(void*, void*, void*)
{
    LOG_INF("[plant] Heavy-Duty Electric Vehicle plant thread started (dt=10 ms, prio=5)");

    s_plant = new (s_plant_buf) plant::PlantModel{config::hdv_default_params()};

    {
        auto p = s_plant->params();
        p.dynamic_config.surface_mu = g_ctrl_bus.surface_mu;
        s_plant->set_params(p);
    }

    k_timer_start(&plant_timer, K_MSEC(10), K_MSEC(10));

    while (true) {
        k_sem_take(&plant_sem, K_FOREVER);

        const uint32_t t0_cycles = k_cycle_get_32();
        const double   t_s       = k_uptime_get_32() / 1000.0;

        // ── Surface mu propagation ────────────────────────────────────────────
        {
            static double last_mu = 0.0;
            if (g_ctrl_bus.surface_mu != last_mu) {
                last_mu = g_ctrl_bus.surface_mu;
                auto p = s_plant->params();
                p.dynamic_config.surface_mu = g_ctrl_bus.surface_mu;
                s_plant->set_params(p);
            }
        }

        // ── Read actuation command (timestamp inside g_sim_bus.cmd under mutex) ─
        sim::ActuatorCmd cmd;
        { hdv::MutexGuard g(g_sim_cmd_mtx); cmd = g_sim_bus.cmd; }

        // ── CAN RX watchdog — safe mode if no fresh command for 500 ms ───────
        // last_update_t_s is written under g_sim_cmd_mtx; we read it via the
        // local copy so the check is race-free.
        if (cmd.last_update_t_s > 0.0 &&
            (t_s - cmd.last_update_t_s) > CAN_RX_TIMEOUT_S) {
            cmd.reset();
        }

        // ── Step plant physics ────────────────────────────────────────────────
        plant::PlantState local_state;
        { hdv::MutexGuard g(g_sim_plant_mtx); local_state = g_sim_bus.plant; }

        s_plant->step(local_state, cmd, DT_S);

        { hdv::MutexGuard g(g_sim_plant_mtx); g_sim_bus.plant = local_state; }

        // ── Loop timing + overrun detection ───────────────────────────────────
        const uint32_t dt_cycles = k_cycle_get_32() - t0_cycles;
        const uint32_t loop_us   = k_cyc_to_us_ceil32(dt_cycles);

        if (loop_us > OVERRUN_WARN_US) {
            LOG_WRN("[plant] loop overrun: %u us (limit %u us)", loop_us, OVERRUN_WARN_US);
        }

        // Update worst-case in sys_stats
        {
            hdv::MutexGuard g(g_health_mtx);
            if (loop_us > g_health_bus.stats.plant_loop_us_max) {
                g_health_bus.stats.plant_loop_us_max = loop_us;
            }
        }

        // ── CAN TX ────────────────────────────────────────────────────────────
        can_tx_send_all(local_state, t_s, loop_us);

        // ── Stroke hardware watchdog via watchdog_thread semaphore ────────────
#ifdef CONFIG_WATCHDOG
        k_sem_give(&g_wdt_sem);
#endif
    }
}

K_THREAD_DEFINE(plant_tid, 16384, plant_thread, NULL, NULL, NULL, 5, 0, 0);
