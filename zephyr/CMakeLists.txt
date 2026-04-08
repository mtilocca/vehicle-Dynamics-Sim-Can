cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hdv_sim VERSION 1.0.0 LANGUAGES CXX C)

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)


# Phase 0
set(APP_SRCS
    src/main.cpp
)

# Phase 1: shell, LED, HTTP server (now HTTPS)
list(APPEND APP_SRCS src/shell/debug_cmds.cpp)
list(APPEND APP_SRCS src/led/led_task.cpp)
list(APPEND APP_SRCS
    src/http/http_server.cpp
    src/http/http_cmd.cpp
    src/http/http_page.cpp
    src/http/http_session.cpp
)

# Phase 3: CAN RX thread
list(APPEND APP_SRCS src/can/can_rx.cpp)

# Phase 4: Plant model, CAN TX
file(GLOB_RECURSE PLANT_SRCS "${REPO_ROOT}/src/plant/*.cpp")
list(FILTER PLANT_SRCS EXCLUDE REGEX ".*/plant_model\\.cpp$")
list(APPEND APP_SRCS
    ${PLANT_SRCS}
    ${REPO_ROOT}/src/plant/subsystem_manager/subsystem_manager.cpp
    src/plant/plant_model_zephyr.cpp
    src/plant/plant_thread.cpp
    src/can/can_tx.cpp
)

# Auxiliary threads: stats only (watchdog disabled for boot diagnostics)
list(APPEND APP_SRCS
    src/stats/stats_thread.cpp
)

target_sources(app PRIVATE ${APP_SRCS})

# ── Embed dashboard CSS ───────────────────────────────────────────────────────
generate_inc_file_for_target(app
    ${CMAKE_CURRENT_SOURCE_DIR}/src/http/dashboard.css
    ${CMAKE_CURRENT_BINARY_DIR}/dashboard.css.inc
)


target_include_directories(app PRIVATE
    ${REPO_ROOT}
    ${REPO_ROOT}/src
    ${REPO_ROOT}/src/plant
    ${REPO_ROOT}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/http
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp_compat
    ${CMAKE_CURRENT_BINARY_DIR}    # dashboard.css.inc, server_cert.inc, server_key.inc
)
