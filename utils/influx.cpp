// utils/influx.cpp - IMPROVED VERSION WITH VERBOSE LOGGING
#include "influx.hpp"
#include "logging.hpp"
#include <curl/curl.h>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <stdexcept>
#include <cstring>
#include <chrono>

namespace utils {

// ============================================================================
// Private Implementation (Pimpl)
// ============================================================================

struct InfluxClient::Impl {
    CURL* curl = nullptr;
    struct curl_slist* headers = nullptr;
    std::string write_url;
    std::string auth_header;
    
    Impl() {
        curl = curl_easy_init();
        if (!curl) {
            throw std::runtime_error("Failed to initialize libcurl");
        }
    }
    
    ~Impl() {
        if (headers) {
            curl_slist_free_all(headers);
        }
        if (curl) {
            curl_easy_cleanup(curl);
        }
    }
};

// ============================================================================
// Callback for ignoring HTTP response body
// ============================================================================

static size_t write_callback(void* contents, size_t size, size_t nmemb, void* userp) {
    // Discard response body (we only care about HTTP status code)
    (void)contents;
    (void)userp;
    return size * nmemb;
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

InfluxClient::InfluxClient(const Config& config)
    : config_(config)
    , last_write_time_(-1.0)  // Force first write
    , impl_(std::make_unique<Impl>())
{
    if (!config_.enabled) {
        LOG_INFO("[InfluxDB] Client created but disabled (use --influx flag to enable)");
        return;
    }
    
    // Build write URL: http://localhost:8086/api/v2/write?org=Autonomy&bucket=vehicle-sim
    std::ostringstream url_builder;
    url_builder << config_.url << "/api/v2/write"
                << "?org=" << config_.org
                << "&bucket=" << config_.bucket
                << "&precision=ns";  // Nanosecond precision
    impl_->write_url = url_builder.str();
    
    // Setup headers
    impl_->headers = curl_slist_append(impl_->headers, "Content-Type: text/plain; charset=utf-8");
    
    // Add authentication if token provided
    if (!config_.token.empty()) {
        impl_->auth_header = "Authorization: Token " + config_.token;
        impl_->headers = curl_slist_append(impl_->headers, impl_->auth_header.c_str());
        LOG_INFO("[InfluxDB] Authentication enabled (token configured)");
    } else {
        LOG_WARN("[InfluxDB] No authentication token provided - writes may fail!");
    }
    
    // Configure curl
    curl_easy_setopt(impl_->curl, CURLOPT_URL, impl_->write_url.c_str());
    curl_easy_setopt(impl_->curl, CURLOPT_HTTPHEADER, impl_->headers);
    curl_easy_setopt(impl_->curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(impl_->curl, CURLOPT_TIMEOUT, 5L);  // 5 second timeout
    
    LOG_INFO("[InfluxDB] Client initialized: url=%s org=%s bucket=%s interval=%.0fms",
             config_.url.c_str(), config_.org.c_str(), config_.bucket.c_str(),
             config_.write_interval_s * 1000.0);
}

InfluxClient::~InfluxClient() {
    if (config_.enabled) {
        flush();
        LOG_INFO("[InfluxDB] Client shutdown");
    }
}

// ============================================================================
// Public Interface
// ============================================================================

bool InfluxClient::write_data_point(const plant::PlantState& state,
                                    const sensors::SensorOut& sensor_out,
                                    const sim::ActuatorCmd& cmd,
                                    double sim_time)
{
    if (!config_.enabled) {
        return false;
    }
    
    // Rate limiting: only write every write_interval_s
    if ((sim_time - last_write_time_) < config_.write_interval_s) {
        return false;
    }
    
    last_write_time_ = sim_time;
    
    // Use wall clock time instead of simulation time for InfluxDB
    // This makes data appear at "now" in InfluxDB UI
    int64_t timestamp_ns = wall_clock_time_ns();
    
    // Build line protocol for all measurements
    std::ostringstream line_protocol;
    
    line_protocol << build_vehicle_truth_line(state, cmd, timestamp_ns) << "\n";
    line_protocol << build_battery_sensors_line(state, sensor_out, timestamp_ns) << "\n";
    line_protocol << build_wheel_sensors_line(state, sensor_out, timestamp_ns) << "\n";
    
    // Only add sensor measurements if valid
    if (sensor_out.imu_valid) {
        line_protocol << build_imu_sensors_line(sensor_out, timestamp_ns) << "\n";
    }
    if (sensor_out.gnss_valid) {
        line_protocol << build_gnss_sensors_line(sensor_out, timestamp_ns) << "\n";
    }
    if (sensor_out.radar_valid) {
        line_protocol << build_radar_sensors_line(sensor_out, timestamp_ns) << "\n";
    }
    
    // Send to InfluxDB
    return send_to_influx(line_protocol.str());
}

void InfluxClient::flush() {
    // No buffering in current implementation
    // Future: could add batching for efficiency
}

// ============================================================================
// Line Protocol Builders (match CSV field names exactly)
// ============================================================================

std::string InfluxClient::build_vehicle_truth_line(const plant::PlantState& state,
                                                   const sim::ActuatorCmd& cmd,
                                                   int64_t timestamp_ns)
{
    std::ostringstream line;
    
    // Measurement name
    line << "vehicle_truth";
    
    // Tags (none for single vehicle sim)
    
    // Fields (match CSV exactly)
    line << " "
         << "x_m=" << state.x_m << ","
         << "y_m=" << state.y_m << ","
         << "yaw_deg=" << (state.yaw_rad * 180.0 / M_PI) << ","
         << "v_mps=" << state.v_mps << ","
         << "steer_deg=" << (state.steer_virtual_rad * 180.0 / M_PI) << ","
         << "delta_fl_deg=" << (state.delta_fl_rad * 180.0 / M_PI) << ","
         << "delta_fr_deg=" << (state.delta_fr_rad * 180.0 / M_PI) << ","
         << "drive_torque_cmd_nm=" << cmd.drive_torque_cmd_nm << ","
         << "brake_cmd_pct=" << cmd.brake_cmd_pct << ","
         << "motor_power_kW=" << state.motor_power_kW << ","
         << "regen_power_kW=" << state.regen_power_kW << ","
         << "brake_force_kN=" << state.brake_force_kN;
    
    // Timestamp
    line << " " << timestamp_ns;
    
    return line.str();
}

std::string InfluxClient::build_battery_sensors_line(const plant::PlantState& state,
                                                     const sensors::SensorOut& sensor_out,
                                                     int64_t timestamp_ns)
{
    std::ostringstream line;
    
    line << "battery_sensors";
    
    line << " "
         << "batt_soc_truth=" << state.batt_soc_pct << ","
         << "batt_v_truth=" << state.batt_v << ","
         << "batt_i_truth=" << state.batt_i << ","
         << "batt_soc_meas=" << sensor_out.batt_soc_meas << ","
         << "batt_v_meas=" << sensor_out.batt_v_meas << ","
         << "batt_i_meas=" << sensor_out.batt_i_meas << ","
         << "batt_temp_meas=" << sensor_out.batt_temp_meas;
    
    line << " " << timestamp_ns;
    
    return line.str();
}

std::string InfluxClient::build_wheel_sensors_line(const plant::PlantState& state,
                                                   const sensors::SensorOut& sensor_out,
                                                   int64_t timestamp_ns)
{
    std::ostringstream line;
    
    line << "wheel_sensors";
    
    line << " "
         << "wheel_fl_rps_truth=" << state.wheel_fl_rps << ","
         << "wheel_fr_rps_truth=" << state.wheel_fr_rps << ","
         << "wheel_rl_rps_truth=" << state.wheel_rl_rps << ","
         << "wheel_rr_rps_truth=" << state.wheel_rr_rps << ","
         << "wheel_fl_rps_meas=" << sensor_out.wheel_fl_rps_meas << ","
         << "wheel_fr_rps_meas=" << sensor_out.wheel_fr_rps_meas << ","
         << "wheel_rl_rps_meas=" << sensor_out.wheel_rl_rps_meas << ","
         << "wheel_rr_rps_meas=" << sensor_out.wheel_rr_rps_meas;
    
    line << " " << timestamp_ns;
    
    return line.str();
}

std::string InfluxClient::build_imu_sensors_line(const sensors::SensorOut& sensor_out,
                                                 int64_t timestamp_ns)
{
    std::ostringstream line;
    
    line << "imu_sensors";
    
    line << " "
         << "imu_gx_rps=" << sensor_out.imu_gx_rps << ","
         << "imu_gy_rps=" << sensor_out.imu_gy_rps << ","
         << "imu_gz_rps=" << sensor_out.imu_gz_rps << ","
         << "imu_ax_mps2=" << sensor_out.imu_ax_mps2 << ","
         << "imu_ay_mps2=" << sensor_out.imu_ay_mps2 << ","
         << "imu_az_mps2=" << sensor_out.imu_az_mps2 << ","
         << "imu_temp_c=" << sensor_out.imu_temp_c;
    
    line << " " << timestamp_ns;
    
    return line.str();
}

std::string InfluxClient::build_gnss_sensors_line(const sensors::SensorOut& sensor_out,
                                                  int64_t timestamp_ns)
{
    std::ostringstream line;
    
    line << "gnss_sensors";
    
    line << " "
         << "gnss_lat_deg=" << sensor_out.gnss_lat_deg << ","
         << "gnss_lon_deg=" << sensor_out.gnss_lon_deg << ","
         << "gnss_alt_m=" << sensor_out.gnss_alt_m << ","
         << "gnss_vn_mps=" << sensor_out.gnss_vn_mps << ","
         << "gnss_ve_mps=" << sensor_out.gnss_ve_mps << ","
         << "gnss_fix_type=" << static_cast<int>(sensor_out.gnss_fix_type) << "i,"
         << "gnss_sat_count=" << static_cast<int>(sensor_out.gnss_sat_count) << "i";
    
    line << " " << timestamp_ns;
    
    return line.str();
}

std::string InfluxClient::build_radar_sensors_line(const sensors::SensorOut& sensor_out,
                                                   int64_t timestamp_ns)
{
    std::ostringstream line;
    
    line << "radar_sensors";
    
    line << " "
         << "radar_target_range_m=" << sensor_out.radar_target_range_m << ","
         << "radar_target_rel_vel_mps=" << sensor_out.radar_target_rel_vel_mps << ","
         << "radar_target_angle_deg=" << sensor_out.radar_target_angle_deg << ","
         << "radar_status=" << static_cast<int>(sensor_out.radar_status) << "i";
    
    line << " " << timestamp_ns;
    
    return line.str();
}

// ============================================================================
// HTTP Communication - WITH VERBOSE LOGGING
// ============================================================================

bool InfluxClient::send_to_influx(const std::string& line_protocol) {
    curl_easy_setopt(impl_->curl, CURLOPT_POSTFIELDS, line_protocol.c_str());
    curl_easy_setopt(impl_->curl, CURLOPT_POSTFIELDSIZE, line_protocol.size());
    
    CURLcode res = curl_easy_perform(impl_->curl);
    
    if (res != CURLE_OK) {
        LOG_ERROR("[InfluxDB] Write failed: CURL error: %s", curl_easy_strerror(res));
        return false;
    }
    
    long http_code = 0;
    curl_easy_getinfo(impl_->curl, CURLINFO_RESPONSE_CODE, &http_code);
    
    if (http_code != 204) {  // InfluxDB returns 204 No Content on success
        LOG_ERROR("[InfluxDB] Write failed: HTTP %ld (expected 204)", http_code);
        return false;
    }
    
    // Success - log more frequently at INFO level for visibility
    static int write_count = 0;
    write_count++;
    
    if (write_count == 1) {
        // Always log first successful write
        LOG_INFO("[InfluxDB] ✓ First write successful!");
    } else if (write_count % 20 == 0) {
        // Log every 20 writes (~5 seconds at 4Hz)
        LOG_INFO("[InfluxDB] Successfully wrote %d data points", write_count);
    }
    
    return true;
}

// ============================================================================
// Time Conversion
// ============================================================================

int64_t InfluxClient::wall_clock_time_ns() {
    // Get current wall clock time in nanoseconds since Unix epoch
    // This makes data appear at "now" in InfluxDB UI
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);
    return nanoseconds.count();
}

int64_t InfluxClient::sim_time_to_ns(double sim_time_s) {
    // Convert simulation time (seconds) to nanoseconds
    // NOTE: This creates timestamps starting from Unix epoch (1970-01-01)
    // which won't show up in InfluxDB UI when querying "recent" data
    return static_cast<int64_t>(sim_time_s * 1e9);
}

} // namespace utils