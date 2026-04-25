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
#include "tls/tls_creds.hpp"
#include "sim/actuator_cmd.hpp"
#include "plant/plant_main/plant_state.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// ── Shared globals (defined in main.cpp) ─────────────────────────────────────
extern sim::ActuatorCmd  g_cmd;
extern struct k_mutex    g_cmd_mutex;
extern plant::PlantState g_state;
extern struct k_mutex    g_state_mutex;
extern atomic_t          g_ctrl_source;
extern atomic_t          g_mqtt_rx_count;

// ── Module-level state (extern'd in mqtt_client.hpp) ─────────────────────────
char g_mqtt_broker_addr[32] = CONFIG_HDV_MQTT_BROKER_ADDR;
int  g_mqtt_broker_port     = CONFIG_HDV_MQTT_BROKER_PORT;
bool g_mqtt_reconnect_req   = false;
bool g_mqtt_connected       = false;

// ── Topics ────────────────────────────────────────────────────────────────────
static const char kTopicCmd[]   = "hdv/cmd/actuator";
static const char kTopicState[] = "hdv/state/vehicle";

// ── MQTT library buffers ──────────────────────────────────────────────────────
#define RX_BUF_SIZE  256
#define TX_BUF_SIZE  256
#define PAYLOAD_SIZE 256

static uint8_t s_rx_buf[RX_BUF_SIZE];
static uint8_t s_tx_buf[TX_BUF_SIZE];

// ── Incoming command JSON descriptor ─────────────────────────────────────────
struct MqttCmd {
    int         enable;
    const char* gear;   // JSON_TOK_STRING stores a pointer into the parse buffer
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

// ── Helpers ───────────────────────────────────────────────────────────────────

static inline double clamp_d(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static void apply_mqtt_cmd(const char* payload, int len)
{
    struct MqttCmd mc{};

    int rc = json_obj_parse(const_cast<char*>(payload), (size_t)len,
                            kMqttCmdDescr, ARRAY_SIZE(kMqttCmdDescr), &mc);
    if (rc < 0) {
        LOG_WRN("MQTT: JSON parse error (%d), payload ignored", rc);
        return;
    }

    if (atomic_get(&g_ctrl_source) != CTRL_MQTT) {
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

    k_mutex_lock(&g_cmd_mutex, K_FOREVER);
    g_cmd = c;
    k_mutex_unlock(&g_cmd_mutex);

    atomic_inc(&g_mqtt_rx_count);

    LOG_INF("MQTT: cmd rx → enable=%d gear=%c torque=%d steer=%d brake=%d",
            mc.enable, gear_ch, mc.torque, mc.steer, mc.brake);
}

// ── Publish vehicle state ─────────────────────────────────────────────────────

static void publish_vehicle_state(struct mqtt_client* client)
{
    plant::PlantState s{};
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    s = g_state;
    k_mutex_unlock(&g_state_mutex);

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

    struct mqtt_publish_param pub{};
    pub.message.topic.qos        = MQTT_QOS_0_AT_MOST_ONCE;
    pub.message.topic.topic.utf8 = (const uint8_t*)kTopicState;
    pub.message.topic.topic.size = strlen(kTopicState);
    pub.message.payload.data     = (uint8_t*)buf;
    pub.message.payload.len      = (uint32_t)len;
    pub.message_id               = 0;
    pub.dup_flag                 = 0;
    pub.retain_flag              = 0;

    int rc = mqtt_publish(client, &pub);
    if (rc != 0) {
        LOG_WRN("MQTT: publish failed (%d)", rc);
    }
}

// ── MQTT event handler ────────────────────────────────────────────────────────

static void mqtt_evt_handler(struct mqtt_client* client,
                             const struct mqtt_evt* evt)
{
    switch (evt->type) {

    case MQTT_EVT_CONNACK:
        if (evt->result == 0) {
            g_mqtt_connected = true;
            LOG_INF("MQTT: connected to %s:%d", g_mqtt_broker_addr, g_mqtt_broker_port);

            // Subscribe to actuator commands
            struct mqtt_topic topic{};
            topic.qos        = MQTT_QOS_0_AT_MOST_ONCE;
            topic.topic.utf8 = (const uint8_t*)kTopicCmd;
            topic.topic.size = strlen(kTopicCmd);

            struct mqtt_subscription_list sub_list{};
            sub_list.list      = &topic;
            sub_list.list_count = 1;
            sub_list.message_id = 1;

            int rc = mqtt_subscribe(client, &sub_list);
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

        int rc = mqtt_read_publish_payload_blocking(client,
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
                int dr = mqtt_read_publish_payload_blocking(client, drain, n);
                if (dr <= 0) break;
                remaining -= (uint32_t)dr;
            }
        }

        apply_mqtt_cmd(payload, rc);
        break;
    }

    case MQTT_EVT_DISCONNECT:
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

// ── TLS config ────────────────────────────────────────────────────────────────
// HDV_TLS_CA_TAG (=2) holds the broker CA cert loaded by tls_creds_init().
static const sec_tag_t kMqttTlsTags[] = { HDV_TLS_CA_TAG };

// ── Connect helper ────────────────────────────────────────────────────────────

static int mqtt_do_connect(struct mqtt_client* client, struct sockaddr_in* broker)
{
    // Resolve broker address
    memset(broker, 0, sizeof(*broker));
    broker->sin_family = AF_INET;
    broker->sin_port   = htons((uint16_t)g_mqtt_broker_port);

    int rc = net_addr_pton(AF_INET, g_mqtt_broker_addr, &broker->sin_addr);
    if (rc != 0) {
        LOG_ERR("MQTT: invalid broker address '%s'", g_mqtt_broker_addr);
        return -EINVAL;
    }

    mqtt_client_init(client);

    client->broker        = broker;
    client->evt_cb        = mqtt_evt_handler;
    client->client_id.utf8 = (const uint8_t*)"hdv-sim";
    client->client_id.size = 7;
    client->password      = nullptr;
    client->user_name     = nullptr;
    client->protocol_version = MQTT_VERSION_3_1_1;
    client->rx_buf        = s_rx_buf;
    client->rx_buf_size   = sizeof(s_rx_buf);
    client->tx_buf        = s_tx_buf;
    client->tx_buf_size   = sizeof(s_tx_buf);
    client->transport.type = MQTT_TRANSPORT_SECURE;
    client->transport.tls.config.peer_verify   = TLS_PEER_VERIFY_REQUIRED;
    client->transport.tls.config.sec_tag_list  = kMqttTlsTags;
    client->transport.tls.config.sec_tag_count = ARRAY_SIZE(kMqttTlsTags);
    client->transport.tls.config.hostname      = g_mqtt_broker_addr;

    rc = mqtt_connect(client);
    if (rc != 0) {
        LOG_ERR("MQTT: connect to %s:%d failed (%d)",
                g_mqtt_broker_addr, g_mqtt_broker_port, rc);
        // Release TLS socket that mqtt_connect may have opened before failing.
        // Without this, each failed attempt leaks a TLS context until
        // TLS_MAX_CONTEXTS is exhausted and HTTPS handshakes start failing.
        mqtt_abort(client);
        return rc;
    }
    return 0;
}

// ── MQTT thread ───────────────────────────────────────────────────────────────

static void mqtt_thread_fn(void*, void*, void*)
{
    k_msleep(2000);  // let Ethernet settle (same as HTTP thread)

    static struct mqtt_client  s_client;
    static struct sockaddr_in  s_broker;

    int backoff_ms = 5000;

    while (true) {
        g_mqtt_connected    = false;
        g_mqtt_reconnect_req = false;

        int rc = mqtt_do_connect(&s_client, &s_broker);
        if (rc != 0) {
            LOG_WRN("MQTT: retry in %d ms", backoff_ms);
            k_msleep(backoff_ms);
            backoff_ms = (backoff_ms < 60000) ? backoff_ms * 2 : 60000;
            continue;
        }
        backoff_ms = 5000;  // reset on successful connect

        int64_t last_pub_ms = k_uptime_get();

        // Wait for CONNACK via first mqtt_input call
        struct zsock_pollfd fds{};
        fds.fd     = s_client.transport.tls.sock;
        fds.events = ZSOCK_POLLIN;

        while (true) {
            // Check for external reconnect request (e.g. broker addr changed)
            if (g_mqtt_reconnect_req) {
                LOG_INF("MQTT: reconnecting to %s:%d",
                        g_mqtt_broker_addr, g_mqtt_broker_port);
                mqtt_disconnect(&s_client);
                break;
            }

            int poll_rc = zsock_poll(&fds, 1, 1000);
            if (poll_rc < 0) {
                LOG_WRN("MQTT: poll error (%d), reconnecting", poll_rc);
                mqtt_disconnect(&s_client);
                break;
            }

            if (poll_rc > 0 && (fds.revents & ZSOCK_POLLIN)) {
                rc = mqtt_input(&s_client);
                if (rc != 0) {
                    LOG_WRN("MQTT: input error (%d), reconnecting", rc);
                    g_mqtt_connected = false;
                    break;
                }
            }

            rc = mqtt_live(&s_client);
            if (rc != 0 && rc != -EAGAIN) {
                LOG_WRN("MQTT: keepalive error (%d), reconnecting", rc);
                g_mqtt_connected = false;
                break;
            }

            // 1 Hz telemetry publish
            int64_t now = k_uptime_get();
            if (g_mqtt_connected && (now - last_pub_ms) >= 1000) {
                publish_vehicle_state(&s_client);
                last_pub_ms = now;
            }
        }
    }
}

K_THREAD_DEFINE(mqtt_tid, 3072, mqtt_thread_fn, NULL, NULL, NULL, 9, 0, 0);
