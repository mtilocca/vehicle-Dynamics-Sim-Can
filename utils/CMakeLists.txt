# utils/CMakeLists.txt

# ============================================================================
# Find Dependencies
# ============================================================================

# libcurl is required for InfluxDB HTTP client
find_package(CURL REQUIRED)

# ============================================================================
# Utils Library
# ============================================================================

add_library(utils STATIC
    bitpack.cpp
    influx.cpp
)

target_include_directories(utils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CURL_INCLUDE_DIRS}
)

target_compile_features(utils PUBLIC cxx_std_17)

target_link_libraries(utils PUBLIC
    ${CURL_LIBRARIES}
)

# Link pthread if on Linux (required by some curl builds)
if(UNIX AND NOT APPLE)
    target_link_libraries(utils PUBLIC pthread)
endif()