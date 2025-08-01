/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EZMQTT_H__
#define __EZMQTT_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int ezmqtt_setup(void);
int ezmqtt_get_client_id(char *client_id, uint32_t client_id_len);
int ezmqtt_publish_str(char *topic, char *payload);
int ezmqtt_publish_gen(uint8_t *topic, uint32_t topic_len, uint8_t *payload, uint32_t payload_len);
int ezmqtt_teardown(void);

#ifdef __cplusplus
}
#endif

#endif
