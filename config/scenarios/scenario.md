# Scenario System Guide

This document explains how to create and run scenarios for the plant-sensor-CAN simulation.

## Overview

The simulation supports two methods for defining scenarios:

1. **JSON scenarios** (recommended) - Easy to create and modify without recompiling
2. **Built-in Lua scenarios** - For complex logic and procedural generation

## Quick Start

### Running a Scenario

```bash
# Run with default scenario (brake_test.json)
./build/sim_main

# Run with specific JSON scenario
./build/sim_main config/scenarios/lane_change_smooth.json

# Run with full power test
./build/sim_main config/scenarios/full_power_test.json
```

### Output Files

The simulation produces three output files:

1. **sim_out.csv** - Time-series data (position, velocity, battery state, etc.)
2. **sim_debug.log** - Detailed debug logging (can be disabled)
3. **Console output** - Real-time progress display

## JSON Scenario Format

### Basic Structure

```json
{
  "meta": {
    "name": "my_scenario",
    "version": 1,
    "description": "What this scenario tests"
  },
  "defaults": {
    "system_enable": true,
    "mode": 0,
    "drive_torque_cmd_nm": 0.0,
    "brake_cmd_pct": 0.0,
    "steer_cmd_deg": 0.0
  },
  "segments": [
    {
      "t0": 0.0,
      "t1": 5.0,
      "drive_torque_cmd_nm": 1200.0,
      "comment": "What happens in this segment"
    }
  ]
}
```

### Fields Explained

#### Meta Section
- `name` - Scenario identifier (used in logs)
- `version` - Version number
- `description` - Human-readable description

#### Defaults Section
Default values for all control inputs. Any segment that doesn't specify a value will use the default.

- `system_enable` - Enable/disable the motor controller (true/false)
- `mode` - Operating mode (0-7, currently unused)
- `drive_torque_cmd_nm` - Motor torque in Newton-meters (-4000 to 4000)
- `brake_cmd_pct` - Brake percentage (0 to 100)
- `steer_cmd_deg` - Steering angle in degrees (-500 to 500, typically ±35)

#### Segments Section
Array of time-based segments that override defaults.

- `t0` - Start time in seconds
- `t1` - End time in seconds (use -1 for "until end of simulation")
- `comment` - Optional description

Any control field (`drive_torque_cmd_nm`, `brake_cmd_pct`, `steer_cmd_deg`) can be specified to override the default for this time window.

### Segment Behavior

Segments are evaluated in order for each timestep. Later segments override earlier ones if time windows overlap.

Example:
```json
"segments": [
  {
    "t0": 0.0,
    "t1": 10.0,
    "drive_torque_cmd_nm": 1000.0
  },
  {
    "t0": 5.0,
    "t1": 7.0,
    "steer_cmd_deg": 10.0
  }
]
```

Result:
- 0-5s: motor torque = 1000 Nm, steer = 0°
- 5-7s: motor torque = 1000 Nm, steer = 10° (both active)
- 7-10s: motor torque = 1000 Nm, steer = 0°

## Example Scenarios

### 1. Brake Test (`brake_test_v2.json`)

Tests acceleration followed by regenerative braking:
- 0-4s: Full acceleration (1200 Nm)
- 4-8s: Regenerative braking (60%)
- 8-12s: Moderate acceleration (600 Nm)
- 12-20s: Coast

**What to observe:**
- Battery SOC decreases during acceleration
- Battery SOC increases during regenerative braking
- Current flow reverses during braking

### 2. Lane Change (`lane_change_smooth.json`)

Smooth lane change maneuver:
- 0-2s: Straight
- 2-5s: Steer left (ramp up to 7°)
- 5-6s: Straighten
- 6-9s: Steer right (return to lane)
- 9-20s: Straight

**What to observe:**
- X-Y trajectory shows S-curve
- Yaw angle changes during maneuver
- Ackermann steering angles on front wheels

### 3. Full Power Test (`full_power_test.json`)

Maximum battery discharge test:
- 0-10s: Maximum torque (4000 Nm)
- 10-15s: Reduced power (2000 Nm)
- 15-20s: Coast

**What to observe:**
- Maximum current draw (~225 A at power limit)
- Rapid SOC decrease
- Power limiting at high speeds

## Built-in Lua Scenarios

If no JSON file is specified or loading fails, the simulation falls back to built-in Lua scenarios in `config/lua/scenario.lua`.

Available built-in scenarios:
- `BRAKE_STEP` - Simple brake test
- `SIN_STEER_ACCEL` - Sinusoidal steering with acceleration
- `S_CURVE` - S-curve steering maneuver
- `LANE_CHANGE` - Smooth lane change
- `CONSTANT_RADIUS` - Constant turn
- `STOP_AND_STEER` - Stop and turn

To use a built-in scenario, edit the `ACTIVE` variable in `config/lua/scenario.lua`:

```lua
local ACTIVE = "S_CURVE"  -- Change this
```

## Advanced: Creating Custom Lua Scenarios

For complex scenarios requiring logic, randomization, or procedural generation, you can add functions to the `scenarios` table in `config/lua/scenario.lua`:

```lua
scenarios.MY_SCENARIO = function(t, state)
  -- t = current time in seconds
  -- state = current vehicle state (position, velocity, battery SOC, etc.)
  
  local motor = 0.0
  local brake = 0.0
  local steer = 0.0
  
  -- Your logic here
  if state.v_mps < 10.0 then
    motor = 1500.0
  else
    motor = 500.0
  end
  
  return {
    system_enable = true,
    mode = 0,
    drive_torque_cmd_nm = motor,
    brake_cmd_pct = brake,
    steer_cmd_deg = steer,
  }
end
```

Then set `ACTIVE = "MY_SCENARIO"` to use it.

## Configuration Options

### Simulation Timing

Edit `sim_main.cpp`:

```cpp
cfg.dt_s = 0.01;          // Timestep (10ms recommended)
cfg.duration_s = 20.0;    // Total simulation time
cfg.log_hz = 10.0;        // Console/CSV log frequency
```

### Logging Control

```cpp
// Console and file logging verbosity
utils::set_level(utils::LogLevel::Debug);  // Debug, Info, Warn, Error

// Enable/disable debug log file
cfg.enable_debug_log_file = true;

// Output paths
cfg.csv_log_path = "sim_out.csv";
cfg.debug_log_path = "sim_debug.log";
```

### Log Levels

- **Debug** - Detailed plant/battery/drive calculations
- **Info** - Major events (scenario loading, simulation start/end)
- **Warn** - Warnings (missing files, fallbacks)
- **Error** - Errors only

## Visualizing Results

Use the provided Python plotter:

```bash
python3 sim_plotter.py sim_out.csv
```

This generates plots for:
- Trajectory (X-Y path)
- Speed vs time
- Steering and yaw vs time
- Motor and brake inputs
- Battery SOC vs time
- Battery power (charge/discharge)
- Regenerative braking power

## Tips

1. **Start simple** - Begin with a basic scenario and gradually add complexity
2. **Check battery limits** - SOC should stay between 5-95%
3. **Monitor power limits** - Motor power capped at 90 kW
4. **Validate results** - Use debug logs to verify control inputs are applied correctly
5. **Use comments** - Add `"comment"` fields to segments for documentation

## Troubleshooting

### Scenario not loading
- Check JSON syntax (use a JSON validator)
- Verify file path is correct
- Check debug log for error messages

### Unexpected behavior
- Enable debug logging: `utils::set_level(utils::LogLevel::Debug)`
- Check `sim_debug.log` for detailed state information
- Verify segment time windows don't conflict

### Battery SOC hits limits
- Reduce motor torque or duration
- Add more regenerative braking segments
- Increase battery capacity in `plant_model.cpp`