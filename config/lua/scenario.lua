-- config/lua/scenario.lua
--
-- Expected by C++ LuaRuntime:
--   - scenario_cmd(t, state)  -> returns a table with fields matching ActuatorCmd
-- Optional:
--   - scenario_init(json_path) -> called once at init if your LuaRuntime does that
--
-- NOTE:
-- If you don't have a JSON decoder in Lua (cjson/dkjson), this still works
-- and simply runs with defaults.

local M = {}

-- ----------------------------
-- Defaults (fallback behavior)
-- ----------------------------
local scenario = {
  motor_torque_nm = 1200.0,
  brake_pct = 0.0,          -- 0..100
  steer_amp_deg = 10.0,
  steer_freq_hz = 0.2,

  -- Optional simple “events” timeline (override defaults in time windows)
  -- Example:
  -- events = {
  --   { t0 = 0.0, t1 = 3.0,  motor_torque_nm = 1200.0, brake_pct = 0.0 },
  --   { t0 = 7.0, t1 = 12.9, motor_torque_nm = 0.0,    brake_pct = 40.0 },
  -- }
  events = nil
}

local function file_read_all(path)
  local f = io.open(path, "r")
  if not f then return nil end
  local s = f:read("*a")
  f:close()
  return s
end

local function try_decode_json(str)
  -- Try common Lua JSON libs
  local ok, cjson = pcall(require, "cjson")
  if ok and cjson and cjson.decode then
    return cjson.decode(str)
  end

  local ok2, dkjson = pcall(require, "dkjson")
  if ok2 and dkjson and dkjson.decode then
    local obj, _, err = dkjson.decode(str)
    if err then return nil end
    return obj
  end

  return nil
end

local function apply_overrides(dst, src)
  if type(src) ~= "table" then return end
  for k, v in pairs(src) do
    dst[k] = v
  end
end

-- Optional: called once at init (only if your C++ runtime calls it)
function scenario_init(json_path)
  if type(json_path) ~= "string" or json_path == "" then
    return true
  end

  local txt = file_read_all(json_path)
  if not txt then
    print(string.format("[Lua] scenario_init: could not open JSON: %s (using defaults)", json_path))
    return true
  end

  local obj = try_decode_json(txt)
  if not obj then
    print("[Lua] scenario_init: no JSON decoder available (install lua-cjson or dkjson). Using defaults.")
    return true
  end

  -- Allow either a flat object or { params = {...}, events = [...] }
  if obj.params then
    apply_overrides(scenario, obj.params)
  else
    apply_overrides(scenario, obj)
  end

  if obj.events then
    scenario.events = obj.events
  end

  print(string.format("[Lua] scenario_init: loaded JSON scenario: %s", json_path))
  return true
end

local function find_active_event(t)
  local evs = scenario.events
  if type(evs) ~= "table" then return nil end
  for _, ev in ipairs(evs) do
    local t0 = ev.t0 or 0.0
    local t1 = ev.t1 or -1.0
    if t >= t0 and (t1 < 0.0 or t <= t1) then
      return ev
    end
  end
  return nil
end

-- REQUIRED: called every tick by C++ runtime
function scenario_cmd(t, state)
  -- base values
  local motor_torque_nm = scenario.motor_torque_nm
  local brake_pct       = scenario.brake_pct
  local steer_amp_deg   = scenario.steer_amp_deg
  local steer_freq_hz   = scenario.steer_freq_hz

  -- apply event overrides
  local ev = find_active_event(t)
  if ev then
    if ev.motor_torque_nm ~= nil then motor_torque_nm = ev.motor_torque_nm end
    if ev.brake_pct       ~= nil then brake_pct       = ev.brake_pct       end
    if ev.steer_amp_deg   ~= nil then steer_amp_deg   = ev.steer_amp_deg   end
    if ev.steer_freq_hz   ~= nil then steer_freq_hz   = ev.steer_freq_hz   end
  end

  -- steering profile: sinusoid by default
  local steer_cmd_deg = steer_amp_deg * math.sin(2.0 * math.pi * steer_freq_hz * t)

  -- Return table matching your C++ ActuatorCmd fields
  return {
    system_enable = true,
    mode = 0,
    drive_torque_cmd_nm = motor_torque_nm,
    brake_cmd_pct = brake_pct,
    steer_cmd_deg = steer_cmd_deg,
  }
end
