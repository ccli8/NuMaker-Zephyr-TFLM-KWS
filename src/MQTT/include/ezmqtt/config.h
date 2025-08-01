/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

/* Use MQTT broker test.mosquitto.org */

#define EZMQTT_TLS_SNI_HOSTNAME "test.mosquitto.org"

/* MQTT broker hostname */
#define EZMQTT_SERVER_HOSTNAME EZMQTT_TLS_SNI_HOSTNAME

/* MQTT broker port */
#ifdef CONFIG_MQTT_LIB_TLS
#ifdef CONFIG_MQTT_LIB_WEBSOCKET
#define EZMQTT_SERVER_PORT 8081
#else
#define EZMQTT_SERVER_PORT 8883
#endif
#else
#ifdef CONFIG_MQTT_LIB_WEBSOCKET
#define EZMQTT_SERVER_PORT 8080
#else
#define EZMQTT_SERVER_PORT 1883
#endif
#endif

#define EZMQTT_QOS MQTT_QOS_1_AT_LEAST_ONCE

#endif
