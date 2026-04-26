// zephyr/src/state/sim_state.hpp
// SimStateBus: plant output + actuator command, grouped for semantic clarity.
//
// The mutexes are declared separately (K_MUTEX_DEFINE in main.cpp) because
// Zephyr's K_MUTEX_DEFINE generates statically-initialised linker symbols that
// cannot live inside a struct.
//
// Extern declarations for all consumers:
//   extern hdv::SimStateBus g_sim_bus;
//   extern struct k_mutex   g_sim_plant_mtx;
//   extern struct k_mutex   g_sim_cmd_mtx;
#pragma once

#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"

namespace hdv {

struct SimStateBus {
    plant::PlantState plant{};
    sim::ActuatorCmd  cmd{};
};

} // namespace hdv
