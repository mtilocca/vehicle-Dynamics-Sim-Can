// zephyr/src/main.cpp
// XCMG XDE320 Plant Simulator — Zephyr RTOS entry point
//
// Phase 0: boot, print banner over USART3, shell responds.
// Phase 1: add CAN open (FDCAN1).
// Phase 2+: full plant + sensor + CAN TX/RX stack.
//
// Build: west build -b nucleo_h753zi
// Flash: west flash
// Serial: minicom -D /dev/tty.usbmodem* -b 115200

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(xcmg_sim, LOG_LEVEL_INF);

int main(void)
{
    LOG_INF("========================================");
    LOG_INF("XCMG XDE320 Plant Simulator");
    LOG_INF("Board : nucleo_h753zi (STM32H753ZI)");
    LOG_INF("Phase : 0 - UART boot check");
    LOG_INF("========================================");

    uint32_t tick = 0;
    while (true) {
        k_msleep(5000);
        LOG_INF("heartbeat %u s", tick * 5);
        ++tick;
    }

    return 0;
}
