cmake_minimum_required(VERSION 2.6.4)
project(CFE_CONVERT_LIB C)
#project(CFE_CONVERT_LIB CPP)

include_directories(fsw/public_inc)

# The shared OSAL and cFE include directories should always be used
# Note that this intentionally does NOT include PSP-specific includes, just the generic
include_directories(${CFECORE_SOURCE_DIR}/src/inc)
include_directories(${CFEPSP_SOURCE_DIR}/fsw/inc)

aux_source_directory(fsw/src LIB_SRC_FILES)

SET(CMAKE_CXX_FLAGS " -ggbd -O3 -fPIC -std=c++11 -march=native -mno-avx")

# Create the app module
add_cfe_app(convert_lib ${LIB_SRC_FILES})

