-- config/lua/scenario.lua
-- JSON-driven scenario system with PID velocity controller support

-- ----------------------------
-- Load JSON module (pure Lua)
-- ----------------------------
local json = nil
local json_ok, json_module = pcall(dofile, "config/lua/json.lua")
if json_ok then
  json = json_module
  print("[Lua] JSON module (pure Lua) loaded successfully")
else
  print("[Lua] JSON module load failed: " .. tostring(json_module))
end

-- ----------------------------
-- Configuration
-- ----------------------------
local ACTIVE = nil           -- Active built-in scenario (override from JSON or use built-in)
local JSON_PATH = "config/scenarios/slalom.json"  -- Default JSON file path
local JSON_SCENARIO = nil    -- Populated from JSON if loaded

-- ----------------------------
-- PID state (module-level, persists across calls within one run)
-- ----------------------------
local pid_state = {
    integral   = 0.0,
    last_error = 0.0,
    last_t     = -1.0,
    last_gear  = 1,   -- 1=FORWARD, 2=REVERSE
}

-- ----------------------------
-- JSON Loading
-- ----------------------------
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

  print(string.format("[Lua] Loaded JSON scenario: %s",
        (scenario.meta and scenario.meta.name) or "unnamed"))
  return scenario
end

-- ----------------------------
-- Helpers
-- ----------------------------
local function clamp(x, lo, hi)
  if x < lo then return lo end
  if x > hi then return hi end
  return x
end

local function in_window(t, t0, t1)
  return t >= t0 and (t1 < 0 or t <= t1)
end

-- Safe field read: returns seg[key] when present (even if 0.0), else default.
-- Fixes the Lua `or` zero-bug where `seg.field or default` treats 0.0 as nil.
local function seg_get(seg, key, default)
  local v = seg[key]
  if v ~= nil then return v end
  return default
end

-- ----------------------------
-- Gear-change guard
-- Prevents gear switch while vehicle is still moving above guard_mps.
-- Returns the gear that should actually be commanded this step.
-- ----------------------------
local function apply_gear_guard(requested_gear, v_mps, guard_mps)
  if math.abs(v_mps) > guard_mps then
    return pid_state.last_gear  -- hold current gear until slow enough
  end
  pid_state.last_gear = requested_gear
  return requested_gear
end

-- ----------------------------
-- PID step
-- Returns raw torque output (positive = accelerate, negative = brake).
-- ----------------------------
local function pid_step(target_mps, v_eff, t, cfg)
  local dt
  if pid_state.last_t >= 0 then
    dt = t - pid_state.last_t
  else
    dt = 0.01
  end
  if dt <= 0 then dt = 0.01 end
  pid_state.last_t = t

  local err = target_mps - v_eff
  pid_state.integral = pid_state.integral + err * dt

  -- Anti-windup clamp
  local ilim = cfg.integral_limit or 5000.0
  pid_state.integral = clamp(pid_state.integral, -ilim, ilim)

  local deriv = (err - pid_state.last_error) / dt
  pid_state.last_error = err

  local out = cfg.kp * err + cfg.ki * pid_state.integral + cfg.kd * deriv
  return clamp(out, cfg.min_torque_nm, cfg.max_torque_nm)
end

-- ----------------------------
-- Find the active segment for time t
-- ----------------------------
local function find_active_segment(t, scenario)
  local active = nil
  for _, seg in ipairs(scenario.segments) do
    if in_window(t, seg.t0, seg.t1) then
      active = seg
      break
    end
  end
  -- If no segment matches (e.g. after last segment), return an empty table
  return active or {}
end

-- ----------------------------
-- PID velocity controller command generator
-- ----------------------------
local function get_pid_cmd(t, state, scenario)
  local seg = find_active_segment(t, scenario)

  -- Gear guard threshold
  local guard = 0.2
  if scenario.gear_safety and scenario.gear_safety.guard_threshold_mps then
    guard = scenario.gear_safety.guard_threshold_mps
  end

  -- Determine commanded gear (1=FORWARD, 2=REVERSE), with guard
  local seg_gear = seg_get(seg, "gear_position", 1)
  local cmd_gear = apply_gear_guard(seg_gear, state.v_mps, guard)

  -- Effective velocity: flip sign when in REVERSE so PID always works on positive magnitude
  local v_eff
  if cmd_gear == 2 then
    v_eff = -state.v_mps
  else
    v_eff = state.v_mps
  end

  local target = seg_get(seg, "target_velocity_mps", 0.0)

  -- Run PID
  local pid_cfg = scenario.pid_config
  local raw = pid_step(target, v_eff, t, pid_cfg)

  -- Output split: positive raw → motor drive; negative raw → friction brake
  local drive_torque, brake_pct
  if raw >= 0.0 then
    -- Accelerate (flip torque sign for REVERSE direction)
    if cmd_gear == 2 then
      drive_torque = -raw
    else
      drive_torque = raw
    end
    brake_pct = 0.0
  else
    drive_torque = 0.0
    -- Scale |raw| (0 → |min_torque_nm|) to 0–100 %
    local max_brake_torque = math.abs(pid_cfg.min_torque_nm)
    if max_brake_torque > 0 then
      brake_pct = clamp((-raw / max_brake_torque) * 100.0, 0.0, 100.0)
    else
      brake_pct = 0.0
    end
  end

  return {
    system_enable       = true,
    mode                = 0,
    drive_torque_cmd_nm = drive_torque,
    brake_cmd_pct       = brake_pct,
    steer_cmd_deg       = seg_get(seg, "steer_cmd_deg", 0.0),
    gear_position       = cmd_gear,
  }
end

-- ----------------------------
-- Open-loop JSON command generator (existing path, with `or` zero bug fixed)
-- ----------------------------
local function get_json_cmd(t, scenario)
  local defaults = scenario.defaults or {}
  local cmd = {
    system_enable       = true,
    mode                = 0,
    drive_torque_cmd_nm = seg_get(defaults, "drive_torque_cmd_nm", 0.0),
    brake_cmd_pct       = seg_get(defaults, "brake_cmd_pct",       0.0),
    steer_cmd_deg       = seg_get(defaults, "steer_cmd_deg",        0.0),
    gear_position       = seg_get(defaults, "gear_position",        1),
  }

  -- Apply segment overrides
  for _, seg in ipairs(scenario.segments) do
    if in_window(t, seg.t0, seg.t1) then
      cmd.drive_torque_cmd_nm = seg_get(seg, "drive_torque_cmd_nm", cmd.drive_torque_cmd_nm)
      cmd.brake_cmd_pct       = seg_get(seg, "brake_cmd_pct",       cmd.brake_cmd_pct)
      cmd.steer_cmd_deg       = seg_get(seg, "steer_cmd_deg",       cmd.steer_cmd_deg)
      cmd.gear_position       = seg_get(seg, "gear_position",       cmd.gear_position)
      break
    end
  end

  return cmd
end

-- ----------------------------
-- Built-in scenario registry
-- ----------------------------
local scenarios = {}

scenarios.SIN_STEER_ACCEL = function(t, state)
  local steer = 10.0 * math.sin(2.0 * math.pi * 0.2 * t)
  return {
    system_enable       = true,
    mode                = 0,
    drive_torque_cmd_nm = 1200.0,
    brake_cmd_pct       = 0.0,
    steer_cmd_deg       = steer,
  }
end

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
    system_enable       = true,
    mode                = 0,
    drive_torque_cmd_nm = motor,
    brake_cmd_pct       = brake,
    steer_cmd_deg       = steer,
  }
end

scenarios.S_CURVE = function(t, state)
  local steer = 0.0
  if in_window(t, 2.0, 5.0) then
    steer = 8.0
  elseif in_window(t, 5.0, 8.0) then
    steer = -8.0
  end

  return {
    system_enable       = true,
    mode                = 0,
    drive_torque_cmd_nm = 900.0,
    brake_cmd_pct       = 0.0,
    steer_cmd_deg       = steer,
  }
end

scenarios.LANE_CHANGE = function(t, state)
  local function bump(t, t0, t1, amp)
    if t < t0 or t > t1 then return 0.0 end
    local u = (t - t0) / (t1 - t0)
    return amp * math.sin(math.pi * u)
  end

  local steer = bump(t, 2.0, 5.0, 7.0) - bump(t, 6.0, 9.0, 7.0)

  return {
    system_enable       = true,
    mode                = 0,
    drive_torque_cmd_nm = 800.0,
    brake_cmd_pct       = 0.0,
    steer_cmd_deg       = steer,
  }
end

scenarios.CONSTANT_RADIUS = function(t, state)
  return {
    system_enable       = true,
    mode                = 0,
    drive_torque_cmd_nm = 700.0,
    brake_cmd_pct       = 0.0,
    steer_cmd_deg       = 6.0,
  }
end

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
    system_enable       = true,
    mode                = 0,
    drive_torque_cmd_nm = motor,
    brake_cmd_pct       = brake,
    steer_cmd_deg       = steer,
  }
end

-- ----------------------------
-- Lifecycle hooks
-- ----------------------------
function scenario_init(json_path_from_cpp)
  print("[Lua] scenario_init called")

  -- Reset PID state so repeated runs start clean
  pid_state.integral   = 0.0
  pid_state.last_error = 0.0
  pid_state.last_t     = -1.0
  pid_state.last_gear  = 1

  -- Use path from C++ if provided, else fall back to hardcoded default
  local path = (json_path_from_cpp and json_path_from_cpp ~= "") and json_path_from_cpp or JSON_PATH

  if path then
    JSON_SCENARIO = load_json_scenario(path)
    if JSON_SCENARIO then
      local ctrl_mode = (JSON_SCENARIO.meta and JSON_SCENARIO.meta.control_mode) or "open_loop"
      print(string.format("[Lua] ✓ Using JSON scenario: %s (control_mode=%s)",
            (JSON_SCENARIO.meta and JSON_SCENARIO.meta.name) or "unnamed", ctrl_mode))
      return true
    end
  end

  print(string.format("[Lua] ✓ Using built-in Lua scenario: %s", ACTIVE or "BRAKE_STEP"))
  return true
end

function scenario_cmd(t, state)
  if JSON_SCENARIO then
    local ctrl_mode = (JSON_SCENARIO.meta and JSON_SCENARIO.meta.control_mode) or "open_loop"
    if ctrl_mode == "velocity_pid" then
      return get_pid_cmd(t, state, JSON_SCENARIO)
    end
    return get_json_cmd(t, JSON_SCENARIO)
  end

  -- Fall back to built-in Lua scenarios
  local fn = scenarios[ACTIVE] or scenarios.BRAKE_STEP
  return fn(t, state)
end
