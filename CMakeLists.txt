# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/deps/ubxlib/port/platform/esp-idf/mcu/esp32/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(S3-IDF_NV3041A-QSPI_GT911-I2C_LVGL)