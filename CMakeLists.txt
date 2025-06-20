include($ENV{IDF_PATH}/tools/cmake/version.cmake)

set(srcs "src/am2302_rmt.c")
set(public_requires)

# Starting from esp-idf v5.3, the RMT driver is moved to separate components
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
    list(APPEND public_requires "esp_driver_rmt")
else()
    list(APPEND public_requires "driver")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       REQUIRES ${public_requires})
