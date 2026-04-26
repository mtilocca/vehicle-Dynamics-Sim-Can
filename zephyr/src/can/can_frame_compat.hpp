#pragma once
// Portable can_frame for both Zephyr firmware and host unit tests.
//
// On Zephyr: delegates to <zephyr/drivers/can.h> — uses the native struct
//   (fields: dlc, data[]).  ZephyrCanIface bridges .dlc ↔ .can_dlc.
// On Linux host: uses SocketCAN <linux/can.h> (fields: can_id, can_dlc, data[]).
// Elsewhere (macOS, MSVC): provides a minimal SocketCAN-compatible stub.

#if defined(__ZEPHYR__)
#  include <zephyr/drivers/can.h>
#elif defined(__linux__)
#  include <linux/can.h>
#else
#  include <cstdint>
struct can_frame {
    uint32_t can_id;
    uint8_t  can_dlc;
    uint8_t  __pad[3];
    uint8_t  data[8];
};
#endif
