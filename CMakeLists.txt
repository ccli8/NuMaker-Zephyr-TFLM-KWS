# Copyright (c) 2025 Nuvoton Technology Corporation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(NuMaker-Zephyr-TFLM-KWS)

target_compile_definitions(app
  PRIVATE
    # Required by app for dcache conditional code
    $<$<BOOL:${CONFIG_CACHE_MANAGEMENT}>:NVT_DCACHE_ON>
    # Required by app to enable DMIC
    $<$<BOOL:${CONFIG_NVT_ML_KWS_INPUT_DMIC}>:USE_DMIC>
    # Required by app to enable Ethos-U profiling
    $<$<BOOL:${CONFIG_NVT_ML_ETHOS_U_PROFILE}>:__PROFILE__>
    # Required by ml-embedded-evaluation-kit to go Ethos-U way
    $<$<BOOL:${CONFIG_NVT_ML_REQUIRES_ETHOS_U}>:ARM_NPU>
    # Required by TFLM to define tensor arena size
    ACTIVATION_BUF_SZ=${CONFIG_NVT_ML_TFLM_TENSOR_ARENA_SIZE}
    # Required by app to tell which model
    $<$<BOOL:${CONFIG_NVT_ML_KWS_MODEL_DS_CNN}>:MODEL_DS_CNN>
)

set(APP_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_include_directories(app
  PRIVATE
    ${APP_SOURCE_DIR}/Device/include
    ${APP_SOURCE_DIR}/ml-embedded-evaluation-kit_clone/application_api_common/include
    ${APP_SOURCE_DIR}/ml-embedded-evaluation-kit_clone/application_api_use_case_kws/include
    ${APP_SOURCE_DIR}/ml-embedded-evaluation-kit_clone/application_main/include
    ${APP_SOURCE_DIR}/ml-embedded-evaluation-kit_clone/log/include
    ${APP_SOURCE_DIR}/ml-embedded-evaluation-kit_clone/math/include
    ${APP_SOURCE_DIR}/ml-embedded-evaluation-kit_clone/profiler/include
    ${APP_SOURCE_DIR}/Model/include
    ${APP_SOURCE_DIR}/MQTT/include
    ${APP_SOURCE_DIR}/NPU/include
    ${APP_SOURCE_DIR}/Pattern/include
    ${APP_SOURCE_DIR}/ProfilerCounter/include
    ${APP_SOURCE_DIR}
)

# Add all BSP_Clone *.cc/*.cpp/*.c files
if(CONFIG_SOC_SERIES_M55M1X)
  file(GLOB_RECURSE SOURCE_BSP_CLONE
    "${APP_SOURCE_DIR}/BSP_Clone/M55M1_StdDriver/*.cc"
    "${APP_SOURCE_DIR}/BSP_Clone/M55M1_StdDriver/*.cpp"
    "${APP_SOURCE_DIR}/BSP_Clone/M55M1_StdDriver/*.c"
  )
endif()

# Add all Device *.cc/*.cpp/*.c files
file(GLOB_RECURSE SOURCE_DEVICE
    "${APP_SOURCE_DIR}/Device/*.cc"
    "${APP_SOURCE_DIR}/Device/*.cpp"
    "${APP_SOURCE_DIR}/Device/*.c"
)
# Exclude DMIC*.c* or dmic*.c* files if not enabled
if(NOT CONFIG_NVT_ML_KWS_INPUT_DMIC)
    list(FILTER SOURCE_DEVICE EXCLUDE REGEX ".*(DMIC)|(dmic).*\\.c.*$")
endif()

# Add all ml-embedded-evaluation-kit_clone *.cc/*.cpp/*.c files
file(GLOB_RECURSE SOURCE_ML_EMBEDDED_EVALUATION_KIT_CLONE
    "${APP_SOURCE_DIR}/ml-embedded-evaluation-kit_clone/*.cc"
    "${APP_SOURCE_DIR}/ml-embedded-evaluation-kit_clone/*.cpp"
    "${APP_SOURCE_DIR}/ml-embedded-evaluation-kit_clone/*.c"
)

# Add all Model *.cc/*.cpp/*.c files
file(GLOB_RECURSE SOURCE_MODEL
    "${APP_SOURCE_DIR}/Model/*.cc"
    "${APP_SOURCE_DIR}/Model/*.cpp"
    "${APP_SOURCE_DIR}/Model/*.c"
)

if(CONFIG_NVT_ML_KWS_OUTPUT_MQTT)
    # Add all MQTT *.cc/*.cpp/*.c files
    file(GLOB_RECURSE SOURCE_MQTT
        "${APP_SOURCE_DIR}/MQTT/*.cc"
        "${APP_SOURCE_DIR}/MQTT/*.cpp"
        "${APP_SOURCE_DIR}/MQTT/*.c"
    )
endif()

# Add all NPU *.cc/*.cpp/*.c files
file(GLOB_RECURSE SOURCE_NPU
    "${APP_SOURCE_DIR}/NPU/*.cc"
    "${APP_SOURCE_DIR}/NPU/*.cpp"
    "${APP_SOURCE_DIR}/NPU/*.c"
)
# Exclude ethosu*.c* or ethos_u*.c* files if not enabled
if(NOT CONFIG_NVT_ML_REQUIRES_ETHOS_U)
    list(FILTER SOURCE_NPU EXCLUDE REGEX ".*ethos_?u.*\\.c.*$")
endif()

# Add all Pattern *.cc/*.cpp/*.c files
file(GLOB_RECURSE SOURCE_PATTERN
    "${APP_SOURCE_DIR}/Pattern/*.cc"
    "${APP_SOURCE_DIR}/Pattern/*.cpp"
    "${APP_SOURCE_DIR}/Pattern/*.c"
)

if(CONFIG_NVT_ML_ETHOS_U_PROFILE)
    # Add all ProfilerCounter *.cc/*.cpp/*.c files
    file(GLOB_RECURSE SOURCE_PROFILER_COUNTER
        "${APP_SOURCE_DIR}/ProfilerCounter/*.cc"
        "${APP_SOURCE_DIR}/ProfilerCounter/*.cpp"
        "${APP_SOURCE_DIR}/ProfilerCounter/*.c"
    )
endif()

# Add root *.cc/*.cpp/*.c files
file(GLOB SOURCE_ROOT
    "${APP_SOURCE_DIR}/*.cc"
    "${APP_SOURCE_DIR}/*.cpp"
    "${APP_SOURCE_DIR}/*.c"
)

target_sources(app
  PRIVATE
    ${SOURCE_BSP_CLONE}
    ${SOURCE_DEVICE}
    ${SOURCE_ML_EMBEDDED_EVALUATION_KIT_CLONE}
    ${SOURCE_MODEL}
    ${SOURCE_MQTT}
    ${SOURCE_NPU}
    ${SOURCE_PATTERN}
    ${SOURCE_PROFILER_COUNTER}
    ${SOURCE_ROOT}
)

# Add networking common code
include(${ZEPHYR_BASE}/samples/net/common/common.cmake)
