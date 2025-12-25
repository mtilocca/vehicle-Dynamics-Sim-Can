-- config/lua/scenario.lua
-- Choose scenario by changing ACTIVE (no rebuild required)

-- ----------------------------
-- Configuration
-- ----------------------------
local ACTIVE = nil  -- Active scenario (override from JSON or use built-in)
local JSON_PATH = "config/scenarios/full_power.json"  -- Your JSON file path
local JSON_SCENARIO = nil  -- Will be populated from JSON if loaded

-- ----------------------------
-- JSON Loading (optional)
-- ----------------------------
local json_ok, json = pcall(require, "json")
if not json_ok then
  print("[Lua] JSON module not available (this is OK - using built-in scenarios)")
  json = nil
end

local function load_json_scenario(path)
  if not json then
    print("[Lua] JSON module not loaded, skipping JSON scenario")
    return nil
  end
  
  local file = io.open(path, "r")
  if not file then
    print(string.format("[Lua] Cannot open JSON file: %s", path))
    return nil
  end
  
  local content = file:read("*a")
  file:close()
  
  local ok, scenario = pcall(json.decode, content)
  if not ok then
    print(string.format("[Lua] JSON decode error: %s", tostring(scenario)))
    return nil
  end
  
  print(string.format("[Lua] Loaded JSON scenario: %s", scenario.meta.name or "unnamed"))
  return scenario
end

-- ----------------------------
-- helpers
-- ----------------------------
local function clamp(x, lo, hi)
  if x < lo then return lo end
  if x > hi then return hi end
  return x
end

local function deg(x) return x end

local function in_window(t, t0, t1)
  return t >= t0 and (t1 < 0 or t <= t1)
end

-- ----------------------------
-- JSON scenario interpreter
-- ----------------------------
local function get_json_cmd(t, scenario)
  local cmd = {
    system_enable = true,
    mode = 0,
    drive_torque_cmd_nm = scenario.defaults.drive_torque_cmd_nm or 0.0,
    brake_cmd_pct = scenario.defaults.brake_cmd_pct or 0.0,
    steer_cmd_deg = scenario.defaults.steer_cmd_deg or 0.0,
  }
  
  -- Apply segment overrides
  for _, seg in ipairs(scenario.segments) do
    if in_window(t, seg.t0, seg.t1) then
      cmd.drive_torque_cmd_nm = seg.drive_torque_cmd_nm or cmd.drive_torque_cmd_nm
      cmd.brake_cmd_pct = seg.brake_cmd_pct or cmd.brake_cmd_pct
      cmd.steer_cmd_deg = seg.steer_cmd_deg or cmd.steer_cmd_deg
      break
    end
  end
  
  return cmd
end

-- ----------------------------
-- scenario registry
-- ----------------------------
local scenarios = {}

-- 1) baseline: accelerating + sinusoidal steering (your current)
scenarios.SIN_STEER_ACCEL = function(t, state)
  local steer = 10.0 * math.sin(2.0 * math.pi * 0.2 * t)
  return {
    system_enable = true,
    mode = 0,
    drive_torque_cmd_nm = 1200.0,
    brake_cmd_pct = 0.0,
    steer_cmd_deg = steer,
  }
end

-- 2) brake step test: accelerate then brake hard then release
scenarios.BRAKE_STEP = function(t, state)
  local motor, brake, steer = 0.0, 0.0, 0.0

  if in_window(t, 0.0, 4.0) then
    motor = 1200.0
  elseif in_window(t, 4.0, 8.0) then
    brake = 60.0
  elseif in_window(t, 8.0, 12.0) then
    motor = 600.0
  end

  return {
    system_enable = true,
    mode = 0,
    drive_torque_cmd_nm = motor,
    brake_cmd_pct = brake,
    steer_cmd_deg = steer,
  }
end

-- 3) S-curve steering: +step then -step then center
scenarios.S_CURVE = function(t, state)
  local steer = 0.0
  if in_window(t, 2.0, 5.0) then
    steer = 8.0
  elseif in_window(t, 5.0, 8.0) then
    steer = -8.0
  end

  return {
    system_enable = true,
    mode = 0,
    drive_torque_cmd_nm = 900.0,
    brake_cmd_pct = 0.0,
    steer_cmd_deg = steer,
  }
end

-- 4) Lane change: smooth left then smooth right
scenarios.LANE_CHANGE = function(t, state)
  local function bump(t, t0, t1, amp)
    if t < t0 or t > t1 then return 0.0 end
    local u = (t - t0) / (t1 - t0)
    return amp * math.sin(math.pi * u)
  end

  local steer = bump(t, 2.0, 5.0, 7.0) - bump(t, 6.0, 9.0, 7.0)

  return {
    system_enable = true,
    mode = 0,
    drive_torque_cmd_nm = 800.0,
    brake_cmd_pct = 0.0,
    steer_cmd_deg = steer,
  }
end

-- 5) Constant radius: constant steer, constant torque
scenarios.CONSTANT_RADIUS = function(t, state)
  return {
    system_enable = true,
    mode = 0,
    drive_torque_cmd_nm = 700.0,
    brake_cmd_pct = 0.0,
    steer_cmd_deg = 6.0,
  }
end

-- 6) Stop & steer: shows "turning while almost stopped"
scenarios.STOP_AND_STEER = function(t, state)
  local motor, brake, steer = 0.0, 0.0, 0.0

  if in_window(t, 0.0, 3.0) then
    motor = 800.0
  elseif in_window(t, 3.0, 6.0) then
    brake = 70.0
    steer = 10.0
  elseif in_window(t, 6.0, 9.0) then
    motor = 300.0
    steer = 10.0
  end

  return {
    system_enable = true,
    mode = 0,
    drive_torque_cmd_nm = motor,
    brake_cmd_pct = brake,
    steer_cmd_deg = steer,
  }
end

-- ----------------------------
-- lifecycle hooks
-- ----------------------------
function scenario_init(json_path_from_cpp)
  print(string.format("[Lua] scenario_init called"))
  
  -- Use hardcoded JSON_PATH if no path provided from C++
  local path = (json_path_from_cpp and json_path_from_cpp ~= "") and json_path_from_cpp or JSON_PATH
  
  -- Try to load JSON scenario if path provided
  if path then
    JSON_SCENARIO = load_json_scenario(path)
    if JSON_SCENARIO then
      print(string.format("[Lua] Using JSON scenario: %s", JSON_SCENARIO.meta.name or "unnamed"))
      return true
    end
  end
  
  -- Fallback to built-in Lua scenario
  print(string.format("[Lua] Using built-in Lua scenario: %s", ACTIVE or "BRAKE_STEP"))
  return true
end

-- REQUIRED by LuaRuntime
function scenario_cmd(t, state)
  -- Use JSON scenario if loaded, otherwise fall back to built-in
  if JSON_SCENARIO then
    return get_json_cmd(t, JSON_SCENARIO)
  end
  
  -- Fall back to built-in Lua scenarios
  local fn = scenarios[ACTIVE] or scenarios.BRAKE_STEP
  return fn(t, state)
end