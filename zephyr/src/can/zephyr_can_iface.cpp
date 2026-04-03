// zephyr/src/can/zephyr_can_iface.cpp
// Zephyr CAN driver wrapper.
//
// Translation layer between our SocketCAN-compatible struct can_frame
// (can_id / can_dlc / data[8]) and Zephyr's struct can_frame
// (id / dlc / data[CAN_MAX_DLEN]).
// This is the ONLY file that includes <zephyr/drivers/can.h> directly.

#include "can/zephyr_can_iface.hpp"

#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

LOG_MODULE_REGISTER(zephyr_can_iface, LOG_LEVEL_INF);

// ── helpers: translate between our can_frame and Zephyr's can_frame ──────────

// Our struct  → Zephyr struct
static void to_zephyr(const ::can_frame& src, struct can_frame& dst)
{
    dst.id  = src.can_id & CAN_EXT_ID_MASK;
    dst.dlc = src.can_dlc;
    dst.flags = (src.can_id & CAN_EFF_FLAG) ? CAN_FRAME_IDE : 0;
    if (src.can_id & CAN_RTR_FLAG) dst.flags |= CAN_FRAME_RTR;
    for (int i = 0; i < src.can_dlc && i < CAN_MAX_DLEN; ++i) {
        dst.data[i] = src.data[i];
    }
}

// Zephyr struct → our struct
static void from_zephyr(const struct can_frame& src, ::can_frame& dst)
{
    dst.can_id  = src.id;
    if (src.flags & CAN_FRAME_IDE) dst.can_id |= CAN_EFF_FLAG;
    if (src.flags & CAN_FRAME_RTR) dst.can_id |= CAN_RTR_FLAG;
    dst.can_dlc = src.dlc;
    for (int i = 0; i < src.dlc && i < 8; ++i) {
        dst.data[i] = src.data[i];
    }
}

namespace can {

// ── ZephyrCanIface::open ──────────────────────────────────────────────────────

bool ZephyrCanIface::open(const char* /*devname*/)
{
    dev_ = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
    if (!device_is_ready(dev_)) {
        LOG_ERR("CAN device not ready");
        dev_ = nullptr;
        return false;
    }

    int ret = can_start(dev_);
    if (ret < 0 && ret != -EALREADY) {
        LOG_ERR("can_start failed: %d", ret);
        dev_ = nullptr;
        return false;
    }

    LOG_INF("FDCAN1 started at 500 kbps");
    return true;
}

// ── ZephyrCanIface::close ─────────────────────────────────────────────────────

ZephyrCanIface::~ZephyrCanIface()
{
    close();
}

void ZephyrCanIface::close()
{
    if (dev_) {
        if (filter_id_ >= 0) {
            can_remove_rx_filter(dev_, filter_id_);
            filter_id_ = -1;
        }
        can_stop(dev_);
        dev_ = nullptr;
    }
}

// ── ZephyrCanIface::write_frame ───────────────────────────────────────────────

bool ZephyrCanIface::write_frame(const ::can_frame& frame)
{
    if (!dev_) return false;

    struct can_frame zframe{};
    to_zephyr(frame, zframe);

    int ret = can_send(dev_, &zframe, K_MSEC(100), nullptr, nullptr);
    if (ret < 0) {
        LOG_WRN("can_send failed: %d", ret);
        return false;
    }
    return true;
}

// ── ZephyrCanIface::read_nonblocking ─────────────────────────────────────────

bool ZephyrCanIface::read_nonblocking(::can_frame& frame)
{
    // Populated via the msgq registered with add_rx_filter_msgq().
    // This overload is kept for interface compatibility; the CAN RX thread
    // uses k_msgq_get(K_FOREVER) directly for lower latency.
    (void)frame;
    return false;
}

// ── ZephyrCanIface::add_rx_filter_msgq ───────────────────────────────────────

bool ZephyrCanIface::add_rx_filter_msgq(struct k_msgq* msgq, uint32_t can_id_raw)
{
    if (!dev_) return false;

    // Build a Zephyr CAN filter for the given 29-bit J1939 ID
    struct can_filter filter{};
    filter.id    = can_id_raw & CAN_EXT_ID_MASK;
    filter.mask  = CAN_EXT_ID_MASK;      // exact match
    filter.flags = CAN_FILTER_IDE;       // extended frame

    // NOTE: Zephyr's k_msgq for CAN uses struct can_frame (Zephyr type).
    // We register a Zephyr-native msgq here, then in can_rx_thread we
    // call from_zephyr() to convert before writing to can_rx_msgq.
    // For simplicity at Phase 3, we re-use a single static Zephyr msgq.
    static struct can_frame zephyr_rx_buf[8];
    static struct k_msgq    zephyr_rx_msgq;
    static bool             zephyr_msgq_init = false;

    if (!zephyr_msgq_init) {
        k_msgq_init(&zephyr_rx_msgq,
                    reinterpret_cast<char*>(zephyr_rx_buf),
                    sizeof(struct can_frame), 8);
        zephyr_msgq_init = true;
    }

    filter_id_ = can_add_rx_filter_msgq(dev_, &zephyr_rx_msgq, &filter);
    if (filter_id_ < 0) {
        LOG_ERR("can_add_rx_filter_msgq failed: %d", filter_id_);
        return false;
    }

    // Spawn a small bridge task: converts Zephyr frames and pushes to our msgq
    static K_THREAD_STACK_DEFINE(bridge_stack, 512);
    static struct k_thread bridge_thread;

    struct bridge_args_t {
        struct k_msgq* zephyr_q;
        struct k_msgq* app_q;
    };
    static bridge_args_t bargs = { &zephyr_rx_msgq, msgq };

    k_thread_create(&bridge_thread, bridge_stack, K_THREAD_STACK_SIZEOF(bridge_stack),
        [](void* a, void*, void*) {
            auto* args = static_cast<bridge_args_t*>(a);
            struct can_frame zf{};
            ::can_frame      af{};
            while (true) {
                if (k_msgq_get(args->zephyr_q, &zf, K_FOREVER) == 0) {
                    from_zephyr(zf, af);
                    k_msgq_put(args->app_q, &af, K_NO_WAIT);
                }
            }
        },
        &bargs, nullptr, nullptr,
        1, 0, K_NO_WAIT);

    LOG_INF("RX filter registered for ID 0x%08X (filter_id=%d)",
            can_id_raw, filter_id_);
    return true;
}

} // namespace can
