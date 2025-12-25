#pragma once

#include <string>

extern "C" {
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

#include "sim/actuator_cmd.hpp"
#include "plant/plant_state.hpp"

namespace sim {

class LuaRuntime {
public:
    LuaRuntime() = default;
    ~LuaRuntime();

    LuaRuntime(const LuaRuntime&) = delete;
    LuaRuntime& operator=(const LuaRuntime&) = delete;

    bool init(const std::string& lua_script_path,
              const std::string& scenario_json_path);

    bool get_actuator_cmd(double t_s,
                          const plant::PlantState& s,
                          sim::ActuatorCmd& out_cmd);

private:
    lua_State* L_{nullptr};

    bool call_bool_func_1str(const char* fn, const std::string& arg);
    bool push_state_table_(const plant::PlantState& s);
    bool read_cmd_table_(int idx, sim::ActuatorCmd& out_cmd);
};

} // namespace sim
