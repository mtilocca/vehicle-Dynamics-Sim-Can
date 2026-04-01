// src/can/can_frame_compat.hpp
// Portability shim: provides struct can_frame on non-Linux builds.
// On Linux, the real SocketCAN kernel header is used.
// On other platforms (macOS, Windows), a minimal stub is defined so that
// the rest of the code compiles; SocketCanIface methods become no-ops.
#pragma once

#ifdef __linux__
#  include <linux/can.h>
#else
#  include <cstdint>

struct can_frame {
    uint32_t can_id;
    uint8_t  can_dlc;
    uint8_t  __pad;
    uint8_t  __res0;
    uint8_t  __res1;
    uint8_t  data[8];
};

// Standard CAN ID masks (normally from <linux/can.h>)
#  define CAN_SFF_MASK 0x000007FFU
#  define CAN_EFF_MASK 0x1FFFFFFFU
#  define CAN_RTR_FLAG 0x40000000U
#  define CAN_EFF_FLAG 0x80000000U
#  define CAN_ERR_FLAG 0x20000000U

#endif
