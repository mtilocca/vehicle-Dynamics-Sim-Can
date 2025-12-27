// src/sensors/sensor_bank.hpp
#pragma once

#include "sensors/sensor_base.hpp"
#include "sensors/sensor_out.hpp"
#include "sensors/battery_sensor.hpp"
#include "sensors/wheel_sensor.hpp"
#include "plant/plant_state.hpp"
#include <memory>
#include <vector>

namespace sensors {

/**
 * SensorBankConfig - Configuration for all sensors
 */
struct SensorBankConfig {
    bool enable_battery_sensor = true;
    bool enable_wheel_sensor = true;
    bool enable_imu_sensor = false;      // Future
    bool enable_gnss_sensor = false;     // Future
    bool enable_radar_sensor = false;    // Future

    BatterySensorParams battery_params{};
    WheelSensorParams wheel_params{};
    
    // Global random seed (0 = random)
    uint64_t random_seed = 0;
};

/**
 * SensorBank - Manages all sensors in the simulation
 * 
 * Responsibilities:
 * - Create and initialize sensors
 * - Update all sensors each timestep
 * - Aggregate sensor outputs
 * - Provide unified interface to sim loop
 */
class SensorBank {
public:
    explicit SensorBank(const SensorBankConfig& cfg = {})
        : cfg_(cfg)
    {
        initialize();
    }

    /**
     * Update all sensors with ground truth
     */
    void step(double t, const plant::PlantState& truth, double dt) {
        for (auto& sensor : sensors_) {
            sensor->step(t, truth, dt);
        }

        // Aggregate all sensor outputs into single SensorOut
        aggregate_outputs();
    }

    /**
     * Get aggregated sensor measurements
     */
    const SensorOut& get_output() const {
        return aggregated_out_;
    }

    /**
     * Reset all sensors
     */
    void reset() {
        for (auto& sensor : sensors_) {
            sensor->reset();
        }
        aggregated_out_ = SensorOut{};
    }

    /**
     * Get individual sensor by name (for debugging)
     */
    SensorBase* get_sensor(const std::string& name) {
        for (auto& sensor : sensors_) {
            if (sensor->name() == name) {
                return sensor.get();
            }
        }
        return nullptr;
    }

    /**
     * Get number of active sensors
     */
    size_t sensor_count() const { return sensors_.size(); }

private:
    void initialize() {
        sensors_.clear();

        // Propagate global seed to individual sensors
        uint64_t seed_offset = 0;

        if (cfg_.enable_battery_sensor) {
            if (cfg_.random_seed != 0) {
                cfg_.battery_params.random_seed = cfg_.random_seed + seed_offset++;
            }
            auto battery = std::make_unique<BatterySensor>(cfg_.battery_params);
            sensors_.push_back(std::move(battery));
        }

        if (cfg_.enable_wheel_sensor) {
            if (cfg_.random_seed != 0) {
                cfg_.wheel_params.random_seed = cfg_.random_seed + seed_offset++;
            }
            auto wheel = std::make_unique<WheelSensor>(cfg_.wheel_params);
            sensors_.push_back(std::move(wheel));
        }

        // Future: IMU, GNSS, Radar sensors
    }

    void aggregate_outputs() {
        // Start with clean slate
        aggregated_out_ = SensorOut{};

        // Merge outputs from all sensors
        for (const auto& sensor : sensors_) {
            auto out = sensor->get_output();

            // Copy timestamp (should be same for all)
            aggregated_out_.t_s = out.t_s;

            // Merge battery data
            if (out.batt_valid) {
                aggregated_out_.batt_v_meas = out.batt_v_meas;
                aggregated_out_.batt_i_meas = out.batt_i_meas;
                aggregated_out_.batt_soc_meas = out.batt_soc_meas;
                aggregated_out_.batt_temp_meas = out.batt_temp_meas;
                aggregated_out_.batt_valid = true;
            }

            // Merge wheel data
            if (out.wheel_valid) {
                aggregated_out_.wheel_fl_rps_meas = out.wheel_fl_rps_meas;
                aggregated_out_.wheel_fr_rps_meas = out.wheel_fr_rps_meas;
                aggregated_out_.wheel_rl_rps_meas = out.wheel_rl_rps_meas;
                aggregated_out_.wheel_rr_rps_meas = out.wheel_rr_rps_meas;
                aggregated_out_.wheel_valid = true;
            }

            // Future: IMU, GNSS, Radar
        }
    }

    SensorBankConfig cfg_;
    std::vector<std::unique_ptr<SensorBase>> sensors_;
    SensorOut aggregated_out_;
};

} // namespace sensors