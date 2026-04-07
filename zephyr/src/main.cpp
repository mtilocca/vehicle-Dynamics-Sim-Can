// zephyr/src/main.cpp
// Heavy-Duty Electric Vehicle Plant Simulator — Zephyr RTOS entry point
//
// Phase 0: boot, print banner over USART3.
// Phase 1: logging shim active, shell commands available.
// Phase 2: static CAN map compiled in.
// Phase 3: FDCAN1 open — RX / TX verified.
// Phase 4: full plant + sensor + CAN TX/RX stack.
//
// Build: west build -b nucleo_h753zi /path/to/repo/zephyr
// Flash: west flash
// Serial: screen /dev/ttyACM0 115200

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"

LOG_MODULE_REGISTER(hdv_sim, LOG_LEVEL_INF);

// ── Shared state (written by plant thread, read by shell + CAN TX) ────────────
plant::PlantState g_state{};
sim::ActuatorCmd  g_cmd{};

K_MUTEX_DEFINE(g_state_mutex);
K_MUTEX_DEFINE(g_cmd_mutex);

// ── CAN counters (updated in Phase 3 by zephyr_can_iface.cpp) ────────────────
volatile uint32_t g_can_tx_count     = 0;
volatile uint32_t g_can_rx_count     = 0;
volatile uint32_t g_can_timeout_count = 0;
volatile double   g_last_rx_t        = 0.0;

// ── Surface friction (set via shell 'plant mu', read by plant in Phase 4) ─────
double g_surface_mu = 0.72;

// ── Plant model pointer (set in Phase 4 when PlantModel is instantiated) ──────
// Forward declaration only — no plant .cpp linked until Phase 4.
namespace plant { class PlantModel; }
plant::PlantModel* g_plant = nullptr;

int main(void)
{
    LOG_INF("========================================");
    LOG_INF("Heavy-Duty Electric Vehicle Plant Simulator");
    LOG_INF("Board : nucleo_h753zi (STM32H753ZI)");
    LOG_INF("Phase : 1 - Logging + UART Shell");
    LOG_INF("========================================");
    LOG_INF("Shell ready on USART3 — type 'help' for commands");

    // Nothing periodic — use 'system uptime' shell command to check liveness.
    while (true) {
        k_msleep(60000);
    }

    return 0;
}
