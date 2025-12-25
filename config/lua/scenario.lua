-- config/lua/scenario.lua
-- Choose scenario by changing ACTIVE (no rebuild required)

--local ACTIVE = "BRAKE_STEP"
-- local ACTIVE = "SIN_STEER_ACCEL"
-- local ACTIVE = "S_CURVE"
local ACTIVE = "LANE_CHANGE"
-- local ACTIVE = "CONSTANT_RADIUS"
-- local ACTIVE = "STOP_AND_STEER"

-- Optional JSON scenario (not required)
-- local JSON_PATH = "config/scenarios/brake_step.json"
-- local JSON_PATH = "config/scenarios/s_curve.json"
local JSON_PATH = "config/scenarios/lane_change.json"
-- local JSON_PATH = "config/scenarios/constant_radius.json"

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
-- scenario registry
-- ----------------------------
local scenarios = {}

-- 1) baseline: accelerating + sinusoidal steering
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

-- 2) brake step test
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

-- 3) S-curve steering
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

-- 4) Lane change (smooth)
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

-- 5) Constant radius
scenarios.CONSTANT_RADIUS = function(t, state)
  return {
    system_enable = true,
    mode = 0,
    drive_torque_cmd_nm = 700.0,
    brake_cmd_pct = 0.0,
    steer_cmd_deg = 6.0,
  }
end

-- 6) Stop & steer
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
function scenario_init(_ignored_from_cpp)
  -- prefer the Lua-selected path
  local ok = false
  if load_json_if_available then
    ok = load_json_if_available(JSON_PATH)
  end
  print(string.format("[Lua] Active scenario = %s", ACTIVE))
  print(string.format("[Lua] scenario_init using JSON_PATH=%s (ok=%s)", JSON_PATH, tostring(ok)))
  return true
end

-- REQUIRED by LuaRuntime
function scenario_cmd(t, state)
  local fn = scenarios[ACTIVE] or scenarios.SIN_STEER_ACCEL
  return fn(t, state)
end
