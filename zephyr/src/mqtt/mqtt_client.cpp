// zephyr/src/mqtt/mqtt_client.cpp
// MQTT client thread — subscribes to hdv/cmd/actuator (inbound actuator commands)
// and publishes hdv/state/vehicle at 1 Hz (outbound telemetry).
//
// Source arbitration: writes g_cmd only when g_ctrl_source == CTRL_MQTT.
// Thread priority 9 — between HTTP (10) and stats (11).

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/data/json.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define HDV_RAD_TO_DEG 57.295779513082320876

#include "mqtt_client.hpp"
#include "mqtt/i_mqtt_client.hpp"
#include "tls/tls_creds.hpp"
#include "utils/mutex_guard.hpp"
#include "sim/actuator_cmd.hpp"
#include "plant/plant_main/plant_state.hpp"
#include "state/sim_state.hpp"
#include "state/control_bus.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// ── Shared state buses (defined in main.cpp) ─────────────────────────────────
extern hdv::SimStateBus  g_sim_bus;
extern hdv::ControlBus   g_ctrl_bus;
extern struct k_mutex    g_sim_plant_mtx;
extern struct k_mutex    g_sim_cmd_mtx;

// ── Broker configuration ──────────────────────────────────────────────────────
static config::BrokerConfig s_broker_cfg_init() {
    config::BrokerConfig cfg{};
    strncpy(cfg.addr, CONFIG_HDV_MQTT_BROKER_ADDR, sizeof(cfg.addr) - 1);
    cfg.port = CONFIG_HDV_MQTT_BROKER_PORT;
    return cfg;
}
static config::BrokerConfig s_broker_cfg = s_broker_cfg_init();

namespace mqtt {
config::BrokerConfig& broker_config() { return s_broker_cfg; }
} // namespace mqtt

bool g_mqtt_connected = false;

// ── Topics ────────────────────────────────────────────────────────────────────
static const char kTopicCmd[]   = "hdv/cmd/actuator";
static const char kTopicState[] = "hdv/state/vehicle";

// ── MQTT library buffers ──────────────────────────────────────────────────────
#define RX_BUF_SIZE  256
#define TX_BUF_SIZE  256
#define PAYLOAD_SIZE 256

// ── Incoming command JSON descriptor ─────────────────────────────────────────
struct MqttCmd {
    int         enable;
    const char* gear;
    int         torque;
    int         steer;
    int         brake;
};

static const struct json_obj_descr kMqttCmdDescr[] = {
    JSON_OBJ_DESCR_PRIM(struct MqttCmd, enable, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM_NAMED(struct MqttCmd, "gear", gear, JSON_TOK_STRING),
    JSON_OBJ_DESCR_PRIM(struct MqttCmd, torque, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct MqttCmd, steer,  JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct MqttCmd, brake,  JSON_TOK_NUMBER),
};

// ── ZephyrMqttClient ──────────────────────────────────────────────────────────
// Concrete Zephyr implementation of IMqttClient.
// Only mqtt_thread_fn instantiates this — no header needed.

class ZephyrMqttClient : public mqtt::IMqttClient {
public:
    int  connect()    override;
    void disconnect() override;
    bool is_connected() const override { return connected_; }
    int  publish(const char* topic, const void* payload, size_t len) override;

    // poll(): zsock_poll(1000 ms) → mqtt_input → mqtt_live.
    // Returns 0 on success, negative errno on network error.
    int  poll() override;

    // Firmware-specific: snapshot g_sim_bus.plant and publish to kTopicState.
    void publish_vehicle_state();

private:
    static void s_evt_handler(struct mqtt_client* c, const struct mqtt_evt* evt);
    void handle_evt(const struct mqtt_evt* evt);
    void apply_cmd(const char* payload, int len);

    static inline double clamp_d(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    struct mqtt_client client_{};
    struct sockaddr_in broker_addr_{};
    bool connected_ = false;

    uint8_t rx_buf_[RX_BUF_SIZE]{};
    uint8_t tx_buf_[TX_BUF_SIZE]{};

    // Trampoline needs to reach the instance; only one exists at a time.
    static ZephyrMqttClient* s_instance_;
};

ZephyrMqttClient* ZephyrMqttClient::s_instance_ = nullptr;

// ── connect ───────────────────────────────────────────────────────────────────

int ZephyrMqttClient::connect()
{
    memset(&broker_addr_, 0, sizeof(broker_addr_));
    broker_addr_.sin_family = AF_INET;
    broker_addr_.sin_port   = htons((uint16_t)s_broker_cfg.port);

    int rc = net_addr_pton(AF_INET, s_broker_cfg.addr, &broker_addr_.sin_addr);
    if (rc != 0) {
        LOG_ERR("MQTT: invalid broker address '%s'", s_broker_cfg.addr);
        return -EINVAL;
    }

    static const sec_tag_t kTlsTags[] = { HDV_TLS_CA_TAG };

    mqtt_client_init(&client_);
    s_instance_ = this;

    client_.broker             = &broker_addr_;
    client_.evt_cb             = s_evt_handler;
    client_.client_id.utf8     = (const uint8_t*)"hdv-sim";
    client_.client_id.size     = 7;
    client_.password           = nullptr;
    client_.user_name          = nullptr;
    client_.protocol_version   = MQTT_VERSION_3_1_1;
    client_.rx_buf             = rx_buf_;
    client_.rx_buf_size        = sizeof(rx_buf_);
    client_.tx_buf             = tx_buf_;
    client_.tx_buf_size        = sizeof(tx_buf_);
    client_.transport.type     = MQTT_TRANSPORT_SECURE;
    client_.transport.tls.config.peer_verify   = TLS_PEER_VERIFY_REQUIRED;
    client_.transport.tls.config.sec_tag_list  = kTlsTags;
    client_.transport.tls.config.sec_tag_count = ARRAY_SIZE(kTlsTags);
    client_.transport.tls.config.hostname      = s_broker_cfg.addr;

    rc = mqtt_connect(&client_);
    if (rc != 0) {
        LOG_ERR("MQTT: connect to %s:%d failed (%d)",
                s_broker_cfg.addr, s_broker_cfg.port, rc);
        // Release TLS socket to avoid leaking contexts on repeated failures.
        mqtt_abort(&client_);
        return rc;
    }
    return 0;
}

// ── disconnect ────────────────────────────────────────────────────────────────

void ZephyrMqttClient::disconnect()
{
    mqtt_disconnect(&client_);
    connected_ = false;
}

// ── publish ───────────────────────────────────────────────────────────────────

int ZephyrMqttClient::publish(const char* topic, const void* payload, size_t len)
{
    struct mqtt_publish_param pub{};
    pub.message.topic.qos        = MQTT_QOS_0_AT_MOST_ONCE;
    pub.message.topic.topic.utf8 = (const uint8_t*)topic;
    pub.message.topic.topic.size = strlen(topic);
    pub.message.payload.data     = (uint8_t*)payload;
    pub.message.payload.len      = (uint32_t)len;
    pub.message_id               = 0;
    pub.dup_flag                 = 0;
    pub.retain_flag              = 0;
    return mqtt_publish(&client_, &pub);
}

// ── poll ──────────────────────────────────────────────────────────────────────

int ZephyrMqttClient::poll()
{
    struct zsock_pollfd fds{};
    fds.fd     = client_.transport.tls.sock;
    fds.events = ZSOCK_POLLIN;

    int rc = zsock_poll(&fds, 1, 1000);
    if (rc < 0) return rc;

    if (rc > 0 && (fds.revents & ZSOCK_POLLIN)) {
        rc = mqtt_input(&client_);
        if (rc != 0) {
            connected_ = false;
            return rc;
        }
    }

    rc = mqtt_live(&client_);
    if (rc != 0 && rc != -EAGAIN) {
        connected_ = false;
        return rc;
    }
    return 0;
}

// ── publish_vehicle_state ─────────────────────────────────────────────────────

void ZephyrMqttClient::publish_vehicle_state()
{
    plant::PlantState s{};
    { hdv::MutexGuard g(g_sim_plant_mtx); s = g_sim_bus.plant; }

    char buf[PAYLOAD_SIZE];
    int len = snprintf(buf, sizeof(buf),
        "{\"speed_mps\":%.2f,\"yaw_deg\":%.1f,\"x_m\":%.1f,\"y_m\":%.1f,"
        "\"soc_pct\":%.1f,\"batt_v\":%.1f,\"motor_kw\":%.1f}",
        s.v_mps,
        s.yaw_rad * HDV_RAD_TO_DEG,
        s.x_m, s.y_m,
        s.batt_soc_pct,
        s.batt_v,
        s.motor_power_kW);

    if (len <= 0 || len >= (int)sizeof(buf)) return;

    int rc = publish(kTopicState, buf, (size_t)len);
    if (rc != 0) {
        LOG_WRN("MQTT: publish failed (%d)", rc);
    }
}

// ── apply_cmd (private) ───────────────────────────────────────────────────────

void ZephyrMqttClient::apply_cmd(const char* payload, int len)
{
    struct MqttCmd mc{};

    int rc = json_obj_parse(const_cast<char*>(payload), (size_t)len,
                            kMqttCmdDescr, ARRAY_SIZE(kMqttCmdDescr), &mc);
    if (rc < 0) {
        LOG_WRN("MQTT: JSON parse error (%d), payload ignored", rc);
        return;
    }

    if (atomic_get(&g_ctrl_bus.ctrl_source) != CTRL_MQTT) {
        LOG_DBG("MQTT: cmd discarded — ctrl source is not MQTT");
        return;
    }

    char gear_ch = (mc.gear && mc.gear[0]) ? mc.gear[0] : 'N';

    sim::ActuatorCmd c{};
    c.system_enable       = (mc.enable != 0);
    c.steer_cmd_deg       = clamp_d((double)mc.steer,  -45.0,     45.0);
    c.drive_torque_cmd_nm = clamp_d((double)mc.torque,   0.0, 145000.0);
    c.brake_cmd_pct       = clamp_d((double)mc.brake,    0.0,    100.0);
    c.gear_position       = (gear_ch == 'F') ? sim::GearPosition::FORWARD :
                            (gear_ch == 'R') ? sim::GearPosition::REVERSE :
                                               sim::GearPosition::NEUTRAL;
    c.last_update_t_s     = (double)k_uptime_get_32() / 1000.0;

    { hdv::MutexGuard g(g_sim_cmd_mtx); g_sim_bus.cmd = c; }

    atomic_inc(&g_ctrl_bus.mqtt_rx_count);

    LOG_INF("MQTT: cmd rx → enable=%d gear=%c torque=%d steer=%d brake=%d",
            mc.enable, gear_ch, mc.torque, mc.steer, mc.brake);
}

// ── event handler + trampoline ────────────────────────────────────────────────

void ZephyrMqttClient::s_evt_handler(struct mqtt_client* client,
                                     const struct mqtt_evt* evt)
{
    if (s_instance_) s_instance_->handle_evt(evt);
    (void)client;
}

void ZephyrMqttClient::handle_evt(const struct mqtt_evt* evt)
{
    switch (evt->type) {

    case MQTT_EVT_CONNACK:
        if (evt->result == 0) {
            connected_ = true;
            g_mqtt_connected = true;
            LOG_INF("MQTT: connected to %s:%d", s_broker_cfg.addr, s_broker_cfg.port);

            struct mqtt_topic topic{};
            topic.qos        = MQTT_QOS_0_AT_MOST_ONCE;
            topic.topic.utf8 = (const uint8_t*)kTopicCmd;
            topic.topic.size = strlen(kTopicCmd);

            struct mqtt_subscription_list sub_list{};
            sub_list.list       = &topic;
            sub_list.list_count = 1;
            sub_list.message_id = 1;

            int rc = mqtt_subscribe(&client_, &sub_list);
            if (rc != 0) {
                LOG_ERR("MQTT: subscribe failed (%d)", rc);
            } else {
                LOG_INF("MQTT: subscribed to %s", kTopicCmd);
            }
        } else {
            LOG_ERR("MQTT: CONNACK refused (rc=%d)", evt->result);
        }
        break;

    case MQTT_EVT_PUBLISH: {
        const struct mqtt_publish_param* p = &evt->param.publish;
        uint32_t plen = p->message.payload.len;

        char payload[PAYLOAD_SIZE];
        uint32_t read_len = (plen < sizeof(payload) - 1) ? plen : (sizeof(payload) - 1);

        int rc = mqtt_read_publish_payload_blocking(&client_,
                                                    (uint8_t*)payload, read_len);
        if (rc < 0) {
            LOG_WRN("MQTT: payload read error (%d)", rc);
            break;
        }
        payload[rc] = '\0';

        // Drain any remaining bytes we didn't read
        if ((uint32_t)rc < plen) {
            uint8_t drain[32];
            uint32_t remaining = plen - (uint32_t)rc;
            while (remaining > 0) {
                uint32_t n = (remaining < sizeof(drain)) ? remaining : sizeof(drain);
                int dr = mqtt_read_publish_payload_blocking(&client_, drain, n);
                if (dr <= 0) break;
                remaining -= (uint32_t)dr;
            }
        }

        apply_cmd(payload, rc);
        break;
    }

    case MQTT_EVT_DISCONNECT:
        connected_ = false;
        g_mqtt_connected = false;
        LOG_INF("MQTT: disconnected");
        break;

    case MQTT_EVT_SUBACK:
        LOG_INF("MQTT: subscription confirmed");
        break;

    default:
        break;
    }
}

// ── MQTT thread ───────────────────────────────────────────────────────────────

static void mqtt_thread_fn(void*, void*, void*)
{
    k_msleep(2000);  // let Ethernet settle

    ZephyrMqttClient client;
    int backoff_ms = 5000;

    while (true) {
        g_mqtt_connected    = false;
        s_broker_cfg.reconnect_req = false;

        int rc = client.connect();
        if (rc != 0) {
            LOG_WRN("MQTT: retry in %d ms", backoff_ms);
            k_msleep(backoff_ms);
            backoff_ms = (backoff_ms < 60000) ? backoff_ms * 2 : 60000;
            continue;
        }
        backoff_ms = 5000;

        int64_t last_pub_ms = k_uptime_get();

        while (true) {
            if (s_broker_cfg.reconnect_req) {
                LOG_INF("MQTT: reconnecting to %s:%d",
                        s_broker_cfg.addr, s_broker_cfg.port);
                client.disconnect();
                break;
            }

            rc = client.poll();
            if (rc < 0) {
                LOG_WRN("MQTT: poll error (%d), reconnecting", rc);
                break;
            }

            int64_t now = k_uptime_get();
            if (client.is_connected() && (now - last_pub_ms) >= 1000) {
                client.publish_vehicle_state();
                last_pub_ms = now;
            }
        }
    }
}

K_THREAD_DEFINE(mqtt_tid, 3072, mqtt_thread_fn, NULL, NULL, NULL, 9, 0, 0);
