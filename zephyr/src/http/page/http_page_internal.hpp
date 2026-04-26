#pragma once
// Internal header for http/page/ — not a public API.
// Declares send_str, the ThreadInfo shared state, and all card-builder functions.

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/atomic.h>
#include <stdio.h>
#include <string.h>

#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"
#include "stats/sys_stats.hpp"
#include "mqtt/mqtt_client.hpp"
#include "utils/mutex_guard.hpp"
#include "state/sim_state.hpp"
#include "state/control_bus.hpp"
#include "state/system_health.hpp"

// Shared state buses (defined in main.cpp)
extern hdv::SimStateBus      g_sim_bus;
extern hdv::ControlBus       g_ctrl_bus;
extern hdv::SystemHealthBus  g_health_bus;
extern struct k_mutex        g_sim_plant_mtx;
extern struct k_mutex        g_sim_cmd_mtx;
extern struct k_mutex        g_health_mtx;

// Low-level socket writer (defined in http_page_root.cpp)
void send_str(int fd, const char* s);

// ThreadInfo — populated by send_threads_card, consumed by send_memory_card.
// Must call send_threads_card before send_memory_card.
struct ThreadInfo {
    char   name[24];
    int    priority;
    size_t stack_used;
    size_t stack_total;
};
extern ThreadInfo s_threads[20];
extern int        s_thread_count;

// Card builders — plant/actuator snapshots passed in by value from send_page().
void send_plant_card      (int fd, const plant::PlantState& s);
void send_actuator_card   (int fd, const sim::ActuatorCmd&  c);
void send_can_card        (int fd, const sim::ActuatorCmd&  c);
void send_controls_card   (int fd, const sim::ActuatorCmd&  c);
void send_ctrl_source_card(int fd);   // reads g_ctrl_bus.ctrl_source (atomic)
void send_mqtt_card       (int fd);   // reads g_mqtt_* globals + g_ctrl_bus atomics
void send_resources_card  (int fd);   // reads g_health_bus under g_health_mtx
void send_threads_card    (int fd);   // populates s_threads[] via k_thread_foreach
void send_memory_card     (int fd);   // reads s_threads[] — must follow send_threads_card
