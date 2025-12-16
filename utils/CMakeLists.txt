add_library(utils STATIC
    bitpack.cpp
)

target_include_directories(utils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(utils PUBLIC cxx_std_17)