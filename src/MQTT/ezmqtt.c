/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ezmqtt, LOG_LEVEL_INF);

#include <zephyr/kernel.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/random/random.h>
#include <zephyr/drivers/hwinfo.h>

#include <string.h>
#include <errno.h>

#include "ezmqtt/certs.h"
#include "ezmqtt/config.h"
#include "ezmqtt/ezmqtt.h"

#include "net_sample_common.h"

#define EZMQTT_CONNECT_TIMEOUT_MS 2000
#define EZMQTT_SLEEP_MSECS        500
#define EZMQTT_CONNECT_TRIES      10
#define EZMQTT_MQTT_BUFFER_SIZE   128

/* Buffers for MQTT client. */
static uint8_t rx_buffer[EZMQTT_MQTT_BUFFER_SIZE];
static uint8_t tx_buffer[EZMQTT_MQTT_BUFFER_SIZE];

#if defined(CONFIG_MQTT_LIB_WEBSOCKET)
/* Making RX buffer large enough that the full IPv6 packet can fit into it */
#define MQTT_LIB_WEBSOCKET_RECV_BUF_LEN 1280

/* Websocket needs temporary buffer to store partial packets */
static uint8_t temp_ws_rx_buf[MQTT_LIB_WEBSOCKET_RECV_BUF_LEN];
#endif

/* The mqtt client struct */
static struct mqtt_client client_ctx;

/* MQTT Broker details. */
static struct sockaddr_storage broker;

static struct pollfd fds[1];
static int nfds;
static bool connected;

/* Whether to include full topic in the publish message, or alias only (MQTT 5). */
static bool include_topic;
static bool aliases_enabled;

#define EZMQTT_TOPIC_ALIAS 1

#if defined(CONFIG_MQTT_LIB_TLS)

#include "ezmqtt/certs.h"

#define EZMQTT_CA_CERT_TAG 1
#define EZMQTT_PSK_TAG     2

static sec_tag_t m_sec_tags[] = {
#if defined(MBEDTLS_X509_CRT_PARSE_C) || defined(CONFIG_NET_SOCKETS_OFFLOAD)
	EZMQTT_CA_CERT_TAG,
#endif
#if defined(MBEDTLS_KEY_EXCHANGE_SOME_PSK_ENABLED)
	EZMQTT_PSK_TAG,
#endif
};

static int tls_init(void)
{
	int err = -EINVAL;

#if defined(MBEDTLS_X509_CRT_PARSE_C) || defined(CONFIG_NET_SOCKETS_OFFLOAD)
	err = tls_credential_add(EZMQTT_CA_CERT_TAG, TLS_CREDENTIAL_CA_CERTIFICATE, ca_certificate,
				 sizeof(ca_certificate));
	if (err < 0) {
		LOG_ERR("Failed to register public certificate: %d", err);
		return err;
	}
#endif

#if defined(MBEDTLS_KEY_EXCHANGE_SOME_PSK_ENABLED)
	err = tls_credential_add(EZMQTT_PSK_TAG, TLS_CREDENTIAL_PSK, client_psk,
				 sizeof(client_psk));
	if (err < 0) {
		LOG_ERR("Failed to register PSK: %d", err);
		return err;
	}

	err = tls_credential_add(EZMQTT_PSK_TAG, TLS_CREDENTIAL_PSK_ID, client_psk_id,
				 sizeof(client_psk_id) - 1);
	if (err < 0) {
		LOG_ERR("Failed to register PSK ID: %d", err);
	}
#endif

	return err;
}

#endif /* CONFIG_MQTT_LIB_TLS */

static void prepare_fds(struct mqtt_client *client)
{
	if (client->transport.type == MQTT_TRANSPORT_NON_SECURE) {
		fds[0].fd = client->transport.tcp.sock;
	}
#if defined(CONFIG_MQTT_LIB_TLS)
	else if (client->transport.type == MQTT_TRANSPORT_SECURE) {
		fds[0].fd = client->transport.tls.sock;
	}
#endif

	fds[0].events = POLLIN;
	nfds = 1;
}

static void clear_fds(void)
{
	nfds = 0;
}

static int wait(int timeout)
{
	int ret = 0;

	if (nfds > 0) {
		ret = poll(fds, nfds, timeout);
		if (ret < 0) {
			LOG_ERR("poll error: %d", errno);
		}
	}

	return ret;
}

/* Guard from net_pkt OOM by setting threshold
 *
 * NOTE: These configs must be adjusted together with the following
 * Kconfig options:
 * - CONFIG_NET_BUF_RX_COUNT
 * - CONFIG_NET_BUF_TX_COUNT
 * - CONFIG_NET_PKT_RX_COUNT
 * - CONFIG_NET_PKT_TX_COUNT
 */
#define EZMQTT_PUBLISH_TIMEOUT_MS 200
#define EZMQTT_PUBLISH_POLL_MS    20
#define EZMQTT_NET_PKT_RX_THOLD   5
#define EZMQTT_NET_PKT_TX_THOLD   5

void mqtt_evt_handler(struct mqtt_client *const client, const struct mqtt_evt *evt)
{
	int err;

	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result != 0) {
			LOG_ERR("MQTT connect failed %d", evt->result);
			break;
		}

		connected = true;
		LOG_INF("MQTT client connected!");

#if defined(CONFIG_MQTT_VERSION_5_0)
		if (evt->param.connack.prop.rx.has_topic_alias_maximum &&
		    evt->param.connack.prop.topic_alias_maximum > 0) {
			LOG_INF("Topic aliases allowed by the broker, max %u.",
				evt->param.connack.prop.topic_alias_maximum);

			aliases_enabled = true;
		} else {
			LOG_INF("Topic aliases disallowed by the broker.");
		}
#endif

		break;

	case MQTT_EVT_DISCONNECT:
		LOG_INF("MQTT client disconnected %d", evt->result);

		connected = false;
		clear_fds();

		break;

	case MQTT_EVT_PUBACK:
		if (evt->result != 0) {
			LOG_ERR("MQTT PUBACK error %d", evt->result);
			break;
		}

		LOG_INF("PUBACK packet id: %u", evt->param.puback.message_id);

		break;

	case MQTT_EVT_PUBREC:
		if (evt->result != 0) {
			LOG_ERR("MQTT PUBREC error %d", evt->result);
			break;
		}

		LOG_INF("PUBREC packet id: %u", evt->param.pubrec.message_id);

		const struct mqtt_pubrel_param rel_param = {.message_id =
								    evt->param.pubrec.message_id};

		err = mqtt_publish_qos2_release(client, &rel_param);
		if (err != 0) {
			LOG_ERR("Failed to send MQTT PUBREL: %d", err);
		}

		break;

	case MQTT_EVT_PUBCOMP:
		if (evt->result != 0) {
			LOG_ERR("MQTT PUBCOMP error %d", evt->result);
			break;
		}

		LOG_INF("PUBCOMP packet id: %u", evt->param.pubcomp.message_id);

		break;

	case MQTT_EVT_PINGRESP:
		LOG_INF("PINGRESP packet");
		break;

	default:
		break;
	}
}

static int publish(struct mqtt_client *client, enum mqtt_qos qos, uint8_t *topic,
		   uint32_t topic_len, uint8_t *payload, uint32_t payload_len)
{
	struct mqtt_publish_param param = {0};

	/* Always true for MQTT 3.1.1.
	 * True only on first publish message for MQTT 5.0 if broker allows aliases.
	 */
	if (include_topic) {
		param.message.topic.topic.utf8 = topic;
		param.message.topic.topic.size = topic_len;
	}

	param.message.topic.qos = qos;
	param.message.payload.data = payload;
	param.message.payload.len = payload_len;
	param.message_id = sys_rand16_get();
	param.dup_flag = 0U;
	param.retain_flag = 0U;

#if defined(CONFIG_MQTT_VERSION_5_0)
	if (aliases_enabled) {
		param.prop.topic_alias = EZMQTT_TOPIC_ALIAS;
		include_topic = false;
	}
#endif

	return mqtt_publish(client, &param);
}

#define RC_STR(rc) ((rc) == 0 ? "OK" : "ERROR")

#define PRINT_RESULT(func, rc) LOG_INF("%s: %d <%s>", (func), rc, RC_STR(rc))

#if defined(CONFIG_NET_IPV6)
static char server_addr_buf[INET6_ADDRSTRLEN];
#else
static char server_addr_buf[INET_ADDRSTRLEN];
#endif
static char *server_addr = server_addr_buf;

static void broker_init(void)
{
#if defined(CONFIG_NET_IPV6)
	struct sockaddr_in6 *broker6 = (struct sockaddr_in6 *)&broker;

	broker6->sin6_family = AF_INET6;
	broker6->sin6_port = htons(EZMQTT_SERVER_PORT);
	inet_pton(AF_INET6, server_addr, &broker6->sin6_addr);
#else
	struct sockaddr_in *broker4 = (struct sockaddr_in *)&broker;

	broker4->sin_family = AF_INET;
	broker4->sin_port = htons(EZMQTT_SERVER_PORT);
	inet_pton(AF_INET, server_addr, &broker4->sin_addr);
#endif
}

#define EZMQTT_CLIENT_ID_MAXSIZE 20
#define EZMQTT_CLIENT_ID_DEFAULT "test_id"

static uint8_t client_id_buf_bin[EZMQTT_CLIENT_ID_MAXSIZE];
static char client_id_buf_hex[EZMQTT_CLIENT_ID_MAXSIZE * 2 + 1];

static int get_client_id(char *client_id, uint32_t client_id_len)
{
	int rc = hwinfo_get_device_id(client_id_buf_bin, ARRAY_SIZE(client_id_buf_bin));
	if (rc <= 0) {
		return rc;
	} else if (rc >= sizeof(client_id_buf_bin)) {
		rc = -ENOBUFS;
		return rc;
	}

	rc = bin2hex(client_id_buf_bin, rc, client_id, client_id_len);
	if (rc < 0) {
		return rc;
	} else if (rc == 0) {
		rc = -ENOBUFS;
		return rc;
	}

	return rc;
}

static void client_init_id(struct mqtt_client *client)
{
	int rc = get_client_id(client_id_buf_hex, sizeof(client_id_buf_hex));
	if (rc > 0) {
		client->client_id.utf8 = (const uint8_t *)client_id_buf_hex;
		client->client_id.size = rc;
	} else {
		client->client_id.utf8 = (const uint8_t *)EZMQTT_CLIENT_ID_DEFAULT;
		client->client_id.size = strlen(EZMQTT_CLIENT_ID_DEFAULT);
	}
}

static void client_init(struct mqtt_client *client)
{
	mqtt_client_init(client);

	broker_init();

	/* MQTT client configuration */
	client->broker = &broker;
	client->evt_cb = mqtt_evt_handler;
	client_init_id(client);
	client->password = NULL;
	client->user_name = NULL;
#if defined(CONFIG_MQTT_VERSION_5_0)
	client->protocol_version = MQTT_VERSION_5_0;
#else
	client->protocol_version = MQTT_VERSION_3_1_1;
#endif

	/* MQTT buffers configuration */
	client->rx_buf = rx_buffer;
	client->rx_buf_size = sizeof(rx_buffer);
	client->tx_buf = tx_buffer;
	client->tx_buf_size = sizeof(tx_buffer);

	/* MQTT transport configuration */
#if defined(CONFIG_MQTT_LIB_TLS)
#if defined(CONFIG_MQTT_LIB_WEBSOCKET)
	client->transport.type = MQTT_TRANSPORT_SECURE_WEBSOCKET;
#else
	client->transport.type = MQTT_TRANSPORT_SECURE;
#endif

	struct mqtt_sec_config *tls_config = &client->transport.tls.config;

	tls_config->peer_verify = TLS_PEER_VERIFY_REQUIRED;
	tls_config->cipher_list = NULL;
	tls_config->sec_tag_list = m_sec_tags;
	tls_config->sec_tag_count = ARRAY_SIZE(m_sec_tags);
#if defined(MBEDTLS_X509_CRT_PARSE_C) || defined(CONFIG_NET_SOCKETS_OFFLOAD)
	tls_config->hostname = EZMQTT_TLS_SNI_HOSTNAME;
#else
	tls_config->hostname = NULL;
#endif

#else
#if defined(CONFIG_MQTT_LIB_WEBSOCKET)
	client->transport.type = MQTT_TRANSPORT_NON_SECURE_WEBSOCKET;
#else
	client->transport.type = MQTT_TRANSPORT_NON_SECURE;
#endif
#endif

#if defined(CONFIG_MQTT_LIB_WEBSOCKET)
	client->transport.websocket.config.host = server_addr;
	client->transport.websocket.config.url = "/mqtt";
	client->transport.websocket.config.tmp_buf = temp_ws_rx_buf;
	client->transport.websocket.config.tmp_buf_len = sizeof(temp_ws_rx_buf);
	client->transport.websocket.timeout = 5 * MSEC_PER_SEC;
#endif
}

/* In this routine we block until the connected variable is 1 */
static int try_to_connect(struct mqtt_client *client)
{
	int rc, i = 0;

	while (i++ < EZMQTT_CONNECT_TRIES && !connected) {

		client_init(client);

		rc = mqtt_connect(client);
		if (rc != 0) {
			PRINT_RESULT("mqtt_connect", rc);
			k_sleep(K_MSEC(EZMQTT_SLEEP_MSECS));
			continue;
		}

		prepare_fds(client);

		if (wait(EZMQTT_CONNECT_TIMEOUT_MS)) {
			mqtt_input(client);
		}

		if (!connected) {
			mqtt_abort(client);
		}
	}

	if (connected) {
		return 0;
	}

	return -EINVAL;
}

static void ezmqtt_poll(void *, void *, void *);

#define EZMQTT_THREAD_STACK_SIZE 2400
#define EZMQTT_THREAD_PRIORITY   CONFIG_MAIN_THREAD_PRIORITY

K_THREAD_DEFINE(ezmqtt_tid, EZMQTT_THREAD_STACK_SIZE, ezmqtt_poll, NULL, NULL, NULL,
		EZMQTT_THREAD_PRIORITY, 0, -1);

K_SEM_DEFINE(ezmqtt_sem_end, 0, 1);

K_MUTEX_DEFINE(ezmqtt_mutex_sync);

static void ezmqtt_poll(void *, void *, void *)
{
	int rc;

	include_topic = true;
	aliases_enabled = false;

	while (connected && !k_sem_count_get(&ezmqtt_sem_end)) {
		k_mutex_lock(&ezmqtt_mutex_sync, K_FOREVER);
		mqtt_live(&client_ctx);
		mqtt_input(&client_ctx);
		k_mutex_unlock(&ezmqtt_mutex_sync);
		;

		wait(EZMQTT_SLEEP_MSECS);
	}

	k_mutex_lock(&ezmqtt_mutex_sync, K_FOREVER);
	rc = mqtt_disconnect(&client_ctx, NULL);
	k_mutex_unlock(&ezmqtt_mutex_sync);
	PRINT_RESULT("mqtt_disconnect", rc);

	LOG_INF("Bye!");
}

#define EZMQTT_DNS_TIMEOUT_MS 10000
K_SEM_DEFINE(ezmqtt_sem_dns_end, 0, 1);

static void dns_result_cb(enum dns_resolve_status status, struct dns_addrinfo *info,
			  void *user_data)
{
	ARG_UNUSED(user_data);

	char *hr_family;
	void *addr;

	switch (status) {
	case DNS_EAI_CANCELED:
		LOG_INF("DNS query was canceled");
		goto notify_dns_end;
	case DNS_EAI_FAIL:
		LOG_INF("DNS resolve failed");
		goto notify_dns_end;
	case DNS_EAI_NODATA:
		LOG_INF("Cannot resolve address");
		goto notify_dns_end;
	case DNS_EAI_ALLDONE:
		LOG_INF("DNS resolving finished");
		goto notify_dns_end;
	case DNS_EAI_INPROGRESS:
		break;
	default:
		LOG_INF("DNS resolving error (%d)", status);
		goto notify_dns_end;
	}

	if (!info) {
		goto notify_dns_end;
	}

	if (info->ai_family == AF_INET) {
		hr_family = "IPv4";
		addr = &net_sin(&info->ai_addr)->sin_addr;
	} else if (info->ai_family == AF_INET6) {
		hr_family = "IPv6";
		addr = &net_sin6(&info->ai_addr)->sin6_addr;
	} else {
		LOG_ERR("Invalid IP address family %d", info->ai_family);
		goto notify_dns_end;
	}

	net_addr_ntop(info->ai_family, addr, server_addr_buf, sizeof(server_addr_buf));

notify_dns_end:

	k_sem_give(&ezmqtt_sem_dns_end);

	return;
}

int ezmqtt_setup(void)
{
#if defined(CONFIG_NET_DHCPV4)
	net_dhcpv4_start(net_if_get_default());
#endif

#if defined(CONFIG_WIFI)
#if defined(CONFIG_NET_L2_WIFI_SHELL)
	LOG_WRN("Press 'wifi connect' to connect to WiFi AP");
#endif
#endif

	wait_for_network();

	int rc = 0;

#if defined(CONFIG_MQTT_LIB_TLS)
	rc = tls_init();
	PRINT_RESULT("tls_init", rc);
	if (rc != 0) {
		return rc;
	}
#endif

	LOG_INF("Resolving address %s", EZMQTT_SERVER_HOSTNAME);
	uint16_t dns_id;

	k_sem_reset(&ezmqtt_sem_dns_end);
	server_addr[0] = '\0';
	rc = dns_get_addr_info(EZMQTT_SERVER_HOSTNAME,
#if defined(CONFIG_NET_IPV6)
			       DNS_QUERY_TYPE_AAAA
#else
			       DNS_QUERY_TYPE_A,
#endif
				       & dns_id,
			       dns_result_cb, NULL, SYS_FOREVER_MS);
	if (rc < 0) {
		LOG_ERR("Resolve address %s failed: %d", EZMQTT_SERVER_HOSTNAME, rc);
		return rc;
	}

	rc = k_sem_take(&ezmqtt_sem_dns_end, K_MSEC(EZMQTT_DNS_TIMEOUT_MS));
	dns_cancel_addr_info(dns_id);
	if (rc == -EAGAIN) {
		LOG_ERR("Resolve address %s timed out (%d ms)", EZMQTT_SERVER_HOSTNAME,
			EZMQTT_DNS_TIMEOUT_MS);
		return rc;
	} else if (rc != 0) {
		LOG_ERR("Resolve address %s failed", EZMQTT_SERVER_HOSTNAME);
		return rc;
	} else if (strlen(server_addr) == 0) {
		LOG_ERR("Resolve address %s failed", EZMQTT_SERVER_HOSTNAME);
		return rc;
	}
	LOG_INF("Resolved address %s: %s", EZMQTT_SERVER_HOSTNAME, server_addr);

	LOG_INF("attempting to connect %s", server_addr);
	rc = try_to_connect(&client_ctx);
	PRINT_RESULT("try_to_connect", rc);
	if (rc != 0) {
		return rc;
	}

	k_thread_start(ezmqtt_tid);

	return rc;
}

int ezmqtt_get_client_id(char *client_id, uint32_t client_id_len)
{
	return get_client_id(client_id, client_id_len);
}

int ezmqtt_publish_str(char *topic, char *payload)
{
	return ezmqtt_publish_gen((uint8_t *)topic, strlen(topic), (uint8_t *)payload,
				  strlen(payload));
}

int ezmqtt_publish_gen(uint8_t *topic, uint32_t topic_len, uint8_t *payload, uint32_t payload_len)
{
	int rc = 0;
	int64_t ms_ref = k_uptime_get();
	int64_t ms_elapsed = 0;

	while (ms_elapsed < EZMQTT_PUBLISH_TIMEOUT_MS) {
		struct k_mem_slab *rx_pkts = NULL;
		struct k_mem_slab *tx_pkts = NULL;
		uint32_t rx_pkts_free = 0;
		uint32_t tx_pkts_free = 0;
		net_pkt_get_info(&rx_pkts, &tx_pkts, NULL, NULL);
		__ASSERT_NO_MSG(rx_pkts);
		__ASSERT_NO_MSG(tx_pkts);
		rx_pkts_free = k_mem_slab_num_free_get(rx_pkts);
		tx_pkts_free = k_mem_slab_num_free_get(tx_pkts);
		if (rx_pkts_free >= EZMQTT_NET_PKT_RX_THOLD &&
		    tx_pkts_free >= EZMQTT_NET_PKT_TX_THOLD) {
			break;
		}

		k_msleep(EZMQTT_PUBLISH_POLL_MS);
		ms_elapsed = k_uptime_delta(&ms_ref);
	}

	if (ms_elapsed >= EZMQTT_PUBLISH_TIMEOUT_MS) {
		rc = -EAGAIN;
		return rc;
	}

	k_mutex_lock(&ezmqtt_mutex_sync, K_FOREVER);
	rc = publish(&client_ctx, EZMQTT_QOS, topic, topic_len, payload, payload_len);
	k_mutex_unlock(&ezmqtt_mutex_sync);

	return rc;
}

int ezmqtt_teardown(void)
{
	k_sem_give(&ezmqtt_sem_end);
	int rc = k_thread_join(ezmqtt_tid, K_FOREVER);

	return rc;
}
