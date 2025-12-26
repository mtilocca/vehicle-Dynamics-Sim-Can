// src/sim/lua_runtime.cpp
#include "lua_runtime.hpp"
#include "utils/logging.hpp"
#include <cstdio>

namespace sim {

LuaRuntime::~LuaRuntime() {
    if (L_) {
        lua_close(L_);
        L_ = nullptr;
    }
}

bool LuaRuntime::init(const std::string& lua_script_path,
                      const std::string& scenario_json_path) {
    if (L_) {
        lua_close(L_);
        L_ = nullptr;
    }

    L_ = luaL_newstate();
    if (!L_) return false;

    luaL_openlibs(L_);

    // Load JSON helper module first if it exists
    if (luaL_dofile(L_, "config/lua/json.lua") == LUA_OK) {
        LOG_DEBUG("[Lua] Loaded JSON helper");
    } else {
        LOG_WARN("[Lua] JSON helper not loaded (optional)");
        lua_pop(L_, 1);
    }

    // Load main scenario script
    if (luaL_dofile(L_, lua_script_path.c_str()) != LUA_OK) {
        LOG_ERROR("[Lua] Failed to load script: %s", lua_tostring(L_, -1));
        lua_pop(L_, 1);
        return false;
    }

    // Call optional scenario_init(json_path) if present
    lua_getglobal(L_, "scenario_init");
    if (lua_isfunction(L_, -1)) {
        lua_pushstring(L_, scenario_json_path.c_str());
        if (lua_pcall(L_, 1, 1, 0) != LUA_OK) {
            LOG_ERROR("[Lua] scenario_init failed: %s", lua_tostring(L_, -1));
            lua_pop(L_, 1);
            return false;
        }
        const bool ok = lua_toboolean(L_, -1);
        lua_pop(L_, 1);
        if (!ok) {
            LOG_WARN("[Lua] scenario_init returned false");
            // Don't fail completely - Lua scenarios can work without JSON
        }
    } else {
        // no init function
        lua_pop(L_, 1);
    }

    return true;
}

bool LuaRuntime::push_state_table_(const plant::PlantState& s) {
    lua_newtable(L_);

    auto set_num = [&](const char* k, double v) {
        lua_pushstring(L_, k);
        lua_pushnumber(L_, v);
        lua_settable(L_, -3);
    };

    set_num("t_s", s.t_s);
    set_num("x_m", s.x_m);
    set_num("y_m", s.y_m);
    set_num("yaw_rad", s.yaw_rad);
    set_num("v_mps", s.v_mps);
    set_num("steer_virtual_rad", s.steer_virtual_rad);
    set_num("delta_fl_rad", s.delta_fl_rad);
    set_num("delta_fr_rad", s.delta_fr_rad);
    set_num("batt_soc_pct", s.batt_soc_pct);
    set_num("batt_v", s.batt_v);
    set_num("batt_i", s.batt_i);

    return true;
}

bool LuaRuntime::read_cmd_table_(int idx, sim::ActuatorCmd& out_cmd) {
    if (!lua_istable(L_, idx)) return false;

    auto get_bool = [&](const char* k, bool def) -> bool {
        lua_getfield(L_, idx, k);
        bool v = def;
        if (lua_isboolean(L_, -1)) v = lua_toboolean(L_, -1);
        lua_pop(L_, 1);
        return v;
    };

    auto get_num = [&](const char* k, double def) -> double {
        lua_getfield(L_, idx, k);
        double v = def;
        if (lua_isnumber(L_, -1)) v = lua_tonumber(L_, -1);
        lua_pop(L_, 1);
        return v;
    };

    out_cmd.system_enable = get_bool("system_enable", true);
    out_cmd.mode = static_cast<int>(get_num("mode", 0.0));
    out_cmd.drive_torque_cmd_nm = get_num("drive_torque_cmd_nm", out_cmd.drive_torque_cmd_nm);
    out_cmd.brake_cmd_pct = get_num("brake_cmd_pct", out_cmd.brake_cmd_pct);
    out_cmd.steer_cmd_deg = get_num("steer_cmd_deg", out_cmd.steer_cmd_deg);

    return true;
}

bool LuaRuntime::get_actuator_cmd(double t_s,
                                 const plant::PlantState& s,
                                 sim::ActuatorCmd& out_cmd) {
    if (!L_) return false;

    // Call scenario_cmd(t, state)
    lua_getglobal(L_, "scenario_cmd");
    if (!lua_isfunction(L_, -1)) {
        lua_pop(L_, 1);
        LOG_ERROR("[Lua] scenario_cmd() missing");
        return false;
    }

    lua_pushnumber(L_, t_s);
    push_state_table_(s);

    // returns 1 value: cmd table
    if (lua_pcall(L_, 2, 1, 0) != LUA_OK) {
        LOG_ERROR("[Lua] scenario_cmd failed: %s", lua_tostring(L_, -1));
        lua_pop(L_, 1);
        return false;
    }

    const bool ok = read_cmd_table_(-1, out_cmd);
    lua_pop(L_, 1);
    return ok;
}

} // namespace sim