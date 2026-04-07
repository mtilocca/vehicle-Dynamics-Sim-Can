// src/can/can_frame_compat.hpp
// Portability shim: provides a SocketCAN-compatible struct can_frame on all platforms.
//
// Platform behaviour:
//   Linux   — use real <linux/can.h> (SocketCAN kernel header)
//   Zephyr  — define our own struct (do NOT include <zephyr/drivers/can.h> here;
//              zephyr_can_iface.cpp is the only file that touches the Zephyr CAN API
//              and translates between Zephyr's can_frame and ours at the boundary)
//   macOS / Windows — minimal stub so the rest of the code compiles
#pragma once

#if defined(__linux__)
#  include <linux/can.h>

#elif defined(__ZEPHYR__)
#  include <cstdint>

struct can_frame {
    uint32_t can_id;
    uint8_t  can_dlc;
    uint8_t  __pad;
    uint8_t  __res0;
    uint8_t  __res1;
    uint8_t  data[8];
};

#  define CAN_SFF_MASK 0x000007FFU
#  define CAN_EFF_MASK 0x1FFFFFFFU
#  define CAN_RTR_FLAG 0x40000000U
#  define CAN_EFF_FLAG 0x80000000U
#  define CAN_ERR_FLAG 0x20000000U

#else  // macOS / Windows
#  include <cstdint>

struct can_frame {
    uint32_t can_id;
    uint8_t  can_dlc;
    uint8_t  __pad;
    uint8_t  __res0;
    uint8_t  __res1;
    uint8_t  data[8];
};

#  define CAN_SFF_MASK 0x000007FFU
#  define CAN_EFF_MASK 0x1FFFFFFFU
#  define CAN_RTR_FLAG 0x40000000U
#  define CAN_EFF_FLAG 0x80000000U
#  define CAN_ERR_FLAG 0x20000000U

#endif
