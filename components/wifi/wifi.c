#include "esp_wifi.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "lwip/api.h"
#include "lwip/sockets.h"
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <wifi.h>

static char *tag = "wifi";
static bool init_done = false;
static bool is_connected = false;
static const char *current_uuid;
static uint8_t mac[6];
static char ip_address[16];
static esp_netif_t *sta_netif = NULL;

static struct {
	wifi_credentials_t *ap_list;
	int list_size;
} credentials_list;

static esp_event_handler_instance_t instance_any_id;
static esp_event_handler_instance_t instance_got_ip;
static wifi_config_t sta_cfg = {
	.sta.pmf_cfg.capable = true,
};

static void event_start_scan(void)
{
	const wifi_scan_config_t scan_config = {
		.ssid = 0,
		.bssid = 0,
		.channel = 0,	/* 0--all channel scan */
		.show_hidden = 0,
		.scan_type = WIFI_SCAN_TYPE_ACTIVE,
		.scan_time.active.min = 0,
		.scan_time.active.max = 0,
	};
	if (esp_wifi_scan_start(&scan_config, false) ==
		ESP_ERR_WIFI_NOT_STARTED)
		esp_wifi_start();

}

static int event_find_ap_from_list(void)
{
	uint16_t ap_found;
	wifi_ap_record_t *ap_list;
	wifi_ap_record_t *ap_alloc;
	bool is_found = false;

	ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_found));

	if (ap_found <= 0) {
		ESP_LOGD(tag, "No APs found, retry...");
		return EAGAIN;
	}

	ap_alloc = malloc(ap_found * sizeof(*ap_list));
	if (!ap_alloc) {
		ESP_LOGE(tag, "Memory low, retry later...");
		return ENOMEM;
	}

	ap_list = ap_alloc;

	ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_found, ap_list));

	for (uint16_t j = 0; j != ap_found; j++) {
		wifi_credentials_t *ap = credentials_list.ap_list;
		ESP_LOGD(tag, "UUID: %s, RSSI=%d", (char *)ap_list->ssid,
			ap_list->rssi);
		for (uint16_t i = 0; i != credentials_list.list_size; i++) {
			if (!strcmp(ap->uuid, (char *)ap_list->ssid)) {
				ESP_LOGD(tag, "Connecting to %s...", ap->uuid);
				strcpy((char *)sta_cfg.sta.ssid, ap->uuid);
				strcpy((char *)sta_cfg.sta.password, ap->pass);
				sta_cfg.sta.threshold.authmode =
					(wifi_auth_mode_t)ap->auth;
				current_uuid = ap->uuid;
				is_found = true;
				goto done;
			}
			ap++;
		}
		ap_list++;
	}
	ESP_LOGD(tag, "No known APs found, retry...");
done:
	free(ap_alloc);

	if (init_done)
		ESP_ERROR_CHECK(esp_wifi_clear_ap_list());
	else
		return ECANCELED;

	if (is_found) {
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
		ESP_ERROR_CHECK(esp_wifi_connect());
		return 0;
	}

	return EFAULT;
}

static void event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT) {
		switch (event_id) {
		case WIFI_EVENT_STA_START:
		case WIFI_EVENT_STA_DISCONNECTED:	/* Step 1 */
		case WIFI_EVENT_STA_BEACON_TIMEOUT:
			ESP_LOGD(tag, "Disconnected, scanning APs...");
			is_connected = false;
			event_start_scan();
			return;
		case WIFI_EVENT_SCAN_DONE:		/* Step 2 */
			ESP_LOGD(tag, "Scanning is done, looking for the AP");
			if (event_find_ap_from_list()) {
				is_connected = false;
				event_start_scan();
			}
			return;
		case WIFI_EVENT_STA_CONNECTED:		/* Step 3 */
			ESP_LOGD(tag, "Connection established.");
			return;
		case WIFI_EVENT_STA_STOP:
			ESP_LOGD(tag, "Wifi stopped.");
			return;
		default: /* TODO: remove this */
			ESP_LOGE(tag, "Undefined event %d, reconnecting...",
				(int)event_id);
			return;
		}
	}

	/* Step 4 */
	if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t *event = (ip_event_got_ip_t*)event_data;
		snprintf(ip_address, sizeof(ip_address), IPSTR,
			IP2STR(&event->ip_info.ip));
		ESP_LOGI(tag, "Got ip: %s", ip_address);
		is_connected = true;
	}
}

int connect_to_server(int *socketfd, const char *server_ip, uint32_t port)
{
	struct sockaddr_in dest = {
		.sin_family = AF_INET,
		.sin_port = htons(port),
	};
	int tmp;

	dest.sin_addr.s_addr = inet_addr(server_ip);

	tmp = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (tmp < 0)
		return errno;

	if (connect(tmp, (struct sockaddr *)&dest, sizeof(dest))) {
		close(tmp);
		return errno;
	}

	*socketfd = tmp;
	return 0;
}

int connect_to_server_from_list(int *socketfd,
				const char *server_ip_list[],
				int list_size,
				uint32_t port) {
	int n = 0;
	int res;

	do {
		res = connect_to_server(socketfd, server_ip_list[n++], port);
	} while (res && n < list_size);
	return res;
}

void wifi_start(wifi_credentials_t *ap_list, int size)
{
	const wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();

	ESP_ERROR_CHECK(ap_list == NULL || size <= 0);
	ESP_ERROR_CHECK(init_done);

	credentials_list.ap_list = ap_list;
	credentials_list.list_size = size;

	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	if (!sta_netif)
		sta_netif = esp_netif_create_default_wifi_sta();
	ESP_ERROR_CHECK(esp_wifi_init(&config));
	ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_STA, mac));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
		ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
		IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_start());
	init_done = true;
}

bool wifi_is_connected(void)
{
	return is_connected;
}

void wifi_stop(void)
{
	if (!init_done)
		return;

	if (is_connected)
		esp_wifi_disconnect();

	is_connected = false;
	init_done = false;

	esp_wifi_stop();
}

const char *wifi_get_current_uuid(void)
{
	return current_uuid;
}

const uint8_t *wifi_get_mac(void)
{
	return mac;
}

const char *wifi_get_ip(void)
{
	return ip_address;
}

void wifi_set_hostname(const char *new_name)
{
	if (!sta_netif)
		sta_netif = esp_netif_create_default_wifi_sta();

	ESP_ERROR_CHECK(
		esp_netif_set_hostname(sta_netif, new_name));
	tag = (char *)new_name;
}
