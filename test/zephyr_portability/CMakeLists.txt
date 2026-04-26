# Zephyr portability tests — host-buildable, no Zephyr SDK needed.
# Stub headers in zephyr_stubs/ satisfy <zephyr/kernel.h> etc.

set(ZEPHYR_STUBS ${CMAKE_CURRENT_SOURCE_DIR}/zephyr_stubs)
set(ZEPHYR_SRC   ${CMAKE_SOURCE_DIR}/zephyr/src)

# ── test_result ───────────────────────────────────────────────────────────────
add_gtest_test(test_result)
target_include_directories(test_result PRIVATE
    ${ZEPHYR_SRC}
    ${ZEPHYR_STUBS}
)

# ── test_can_tx_encoding ──────────────────────────────────────────────────────
add_gtest_test(test_can_tx_encoding)
target_include_directories(test_can_tx_encoding PRIVATE
    ${ZEPHYR_SRC}
    ${ZEPHYR_STUBS}
)

# ── test_actuator_cmd_decode ──────────────────────────────────────────────────
add_gtest_test(test_actuator_cmd_decode)
target_include_directories(test_actuator_cmd_decode PRIVATE
    ${ZEPHYR_SRC}
    ${ZEPHYR_STUBS}
)
