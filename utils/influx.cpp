// utils/influx.cpp
#include "influx.hpp"
#include "logging.hpp"
#include <curl/curl.h>
#include <sstream>
#include <cmath>
#include <stdexcept>
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
    (void)contents;
    (void)userp;
    return size * nmemb;
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

InfluxClient::InfluxClient(const Config& config)
    : config_(config)
    , last_write_time_(-1.0)
    , impl_(std::make_unique<Impl>())
{
    if (!config_.enabled) {
        LOG_INFO("[InfluxDB] Client created but disabled");
        return;
    }

    std::ostringstream url_builder;
    url_builder << config_.url << "/api/v2/write"
                << "?org=" << config_.org
                << "&bucket=" << config_.bucket
                << "&precision=ns";
    impl_->write_url = url_builder.str();

    impl_->headers = curl_slist_append(impl_->headers, "Content-Type: text/plain; charset=utf-8");

    if (!config_.token.empty()) {
        impl_->auth_header = "Authorization: Token " + config_.token;
        impl_->headers = curl_slist_append(impl_->headers, impl_->auth_header.c_str());
    } else {
        LOG_WARN("[InfluxDB] No token provided — writes may fail on authenticated instances");
    }

    curl_easy_setopt(impl_->curl, CURLOPT_URL, impl_->write_url.c_str());
    curl_easy_setopt(impl_->curl, CURLOPT_HTTPHEADER, impl_->headers);
    curl_easy_setopt(impl_->curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(impl_->curl, CURLOPT_TIMEOUT, 5L);

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

bool InfluxClient::write_vehicle_truth(const plant::PlantState& state, double sim_time)
{
    if (!config_.enabled) return false;

    if ((sim_time - last_write_time_) < config_.write_interval_s) return false;
    last_write_time_ = sim_time;

    int64_t ts = wall_clock_time_ns();
    return send_to_influx(build_vehicle_truth_line(state, ts));
}

void InfluxClient::flush() {
    // No buffering in current implementation
}

// ============================================================================
// Line Protocol Builder
// ============================================================================

std::string InfluxClient::build_vehicle_truth_line(const plant::PlantState& state,
                                                    int64_t timestamp_ns)
{
    std::ostringstream line;

    line << "vehicle_truth";

    line << " "
         << "x_m=" << state.x_m << ","
         << "y_m=" << state.y_m << ","
         << "yaw_deg=" << (state.yaw_rad * 180.0 / M_PI) << ","
         << "v_mps=" << state.v_mps << ","
         << "vy_mps=" << state.vy_mps << ","
         << "yaw_rate_radps=" << state.yaw_rate_radps << ","
         << "a_long_mps2=" << state.a_long_mps2 << ","
         << "a_lat_mps2=" << state.a_lat_mps2 << ","
         << "steer_deg=" << (state.steer_virtual_rad * 180.0 / M_PI) << ","
         << "delta_fl_deg=" << (state.delta_fl_rad * 180.0 / M_PI) << ","
         << "delta_fr_deg=" << (state.delta_fr_rad * 180.0 / M_PI) << ","
         << "motor_torque_nm=" << state.motor_torque_nm << ","
         << "brake_force_kN=" << state.brake_force_kN << ","
         << "wheel_fl_rps=" << state.wheel_fl_rps << ","
         << "wheel_fr_rps=" << state.wheel_fr_rps << ","
         << "wheel_rl_rps=" << state.wheel_rl_rps << ","
         << "wheel_rr_rps=" << state.wheel_rr_rps;

    line << " " << timestamp_ns;

    return line.str();
}

// ============================================================================
// HTTP Communication
// ============================================================================

bool InfluxClient::send_to_influx(const std::string& line_protocol) {
    curl_easy_setopt(impl_->curl, CURLOPT_POSTFIELDS, line_protocol.c_str());
    curl_easy_setopt(impl_->curl, CURLOPT_POSTFIELDSIZE, line_protocol.size());

    CURLcode res = curl_easy_perform(impl_->curl);

    if (res != CURLE_OK) {
        LOG_ERROR("[InfluxDB] Write failed: %s", curl_easy_strerror(res));
        return false;
    }

    long http_code = 0;
    curl_easy_getinfo(impl_->curl, CURLINFO_RESPONSE_CODE, &http_code);

    if (http_code != 204) {
        LOG_ERROR("[InfluxDB] Write failed: HTTP %ld (expected 204)", http_code);
        return false;
    }

    static int write_count = 0;
    ++write_count;
    if (write_count == 1) {
        LOG_INFO("[InfluxDB] First write successful");
    } else if (write_count % 20 == 0) {
        LOG_INFO("[InfluxDB] %d points written", write_count);
    }

    return true;
}

// ============================================================================
// Time Utility
// ============================================================================

int64_t InfluxClient::wall_clock_time_ns() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

} // namespace utils
