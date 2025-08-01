/*
 * SPDX-FileCopyrightText: Copyright 2021 Arm Limited and/or its affiliates
 * <open-source-office@arm.com> SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ML_EMBEDDED_CORE_LOG_H
#define ML_EMBEDDED_CORE_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <stdio.h>

#define LOG_LEVEL_TRACE 0
#define LOG_LEVEL_DEBUG 1
#define LOG_LEVEL_INFO  2
#define LOG_LEVEL_WARN  3
#define LOG_LEVEL_ERROR 4

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif /*LOG_LEVEL*/

#if !defined(UNUSED)
#define UNUSED(x) ((void)(x))
#endif /* #if !defined(UNUSED) */

#if (LOG_LEVEL == LOG_LEVEL_TRACE)
#define trace(...)      \
    printf("TRACE - "); \
    printf(__VA_ARGS__)
#else
#define trace(...)
#endif /* LOG_LEVEL == LOG_LEVEL_TRACE */

#if (LOG_LEVEL <= LOG_LEVEL_DEBUG)
#define debug(...)      \
    printf("DEBUG - "); \
    printf(__VA_ARGS__)
#else
#define debug(...)
#endif /* LOG_LEVEL > LOG_LEVEL_TRACE */

#if (LOG_LEVEL <= LOG_LEVEL_INFO)
#define info(...)      \
    printf("INFO - "); \
    printf(__VA_ARGS__)
#else
#define info(...)
#endif /* LOG_LEVEL > LOG_LEVEL_DEBUG */

#if (LOG_LEVEL <= LOG_LEVEL_WARN)
#define warn(...)      \
    printf("WARN - "); \
    printf(__VA_ARGS__)
#else
#define warn(...)
#endif /* LOG_LEVEL > LOG_LEVEL_INFO */

#if (LOG_LEVEL <= LOG_LEVEL_ERROR)
#define printf_err(...) \
    printf("ERROR - "); \
    printf(__VA_ARGS__)
#else
#define printf_err(...)
#endif /* LOG_LEVEL > LOG_LEVEL_INFO */

#ifdef __cplusplus
}
#endif

/* On zephyr, redirect ml-embedded-evaluation-kit logging to zephyr way */
#if defined(__ZEPHYR__)

#undef trace
#undef debug
#undef info
#undef warn
#undef printf_err

#include <zephyr/logging/log.h>

#if (LOG_LEVEL <= LOG_LEVEL_DEBUG)
#define APP_LOG_LEVEL LOG_LEVEL_DBG
#elif (LOG_LEVEL <= LOG_LEVEL_INFO)
#define APP_LOG_LEVEL LOG_LEVEL_INF
#elif (LOG_LEVEL <= LOG_LEVEL_WARN)
#define APP_LOG_LEVEL LOG_LEVEL_WRN
#elif (LOG_LEVEL <= LOG_LEVEL_ERROR)
#define APP_LOG_LEVEL LOG_LEVEL_ERR
#endif

#if defined(REGISTER_LOG_MODULE_APP)
LOG_MODULE_REGISTER(app, APP_LOG_LEVEL);
#else
LOG_MODULE_DECLARE(app, APP_LOG_LEVEL);
#endif

#define trace(...)      LOG_DBG(__VA_ARGS__)
#define debug(...)      LOG_DBG(__VA_ARGS__)
#define info(...)       LOG_INF(__VA_ARGS__)
#define warn(...)       LOG_WRN(__VA_ARGS__)
#define printf_err(...) LOG_ERR(__VA_ARGS__)

/*
 * Meet clash of macro PI between bsp and cmsis-dsp
 *
 * The macro PI is defined by both bsp for gpio port i and cmsis-dsp
 * for math π 3.14. Here, it is chosen as π to favor cmsis-dsp compile.
 */
#if defined(CONFIG_SOC_FAMILY_NUMAKER)
#undef PI
#define PI              3.14159265358979f
#endif

#endif

#endif /* ML_EMBEDDED_CORE_LOG_H */
