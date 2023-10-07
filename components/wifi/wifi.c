#include "esp_wifi.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "lwip/api.h"
#include "lwip/sockets.h"
#include <stdint.h>
#include <string.h>
#include "wifi.h"

static const char *tag = "wifi";
static bool init_done = false;
static bool is_connected = false;
static const char *current_uuid;

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
	ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, false));
}

static void event_find_ap_from_list(void)
{
	uint16_t ap_found;
	wifi_ap_record_t *ap_list;
	bool is_found = false;

	ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_found));

	if (ap_found <= 0) {
		ESP_LOGI(tag, "No APs found, retry...");
		event_start_scan();
		return;
	}

	ap_list = malloc(ap_found * sizeof(*ap_list));
	if (!ap_list) {
		ESP_LOGE(tag, "Memory low, retry later...");
		event_start_scan();
		return;
	}

	ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_found, ap_list));

	for (uint16_t j = 0; j != ap_found; j++) {
		wifi_credentials_t *ap = credentials_list.ap_list;
		for (uint16_t i = 0; i != credentials_list.list_size; i++) {
			ESP_LOGI(tag, "UUID: %s, RSSI=%d",
				(char *)ap_list->ssid, ap_list->rssi);
			if (!strcmp(ap->uuid, (char *)ap_list->ssid)) {
				ESP_LOGI(tag, "Connecting to %s...", ap->uuid);
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
	ESP_LOGI(tag, "No known APs found, retry...");
done:
	ESP_ERROR_CHECK(esp_wifi_clear_ap_list());
	free(ap_list);

	if (is_found) {
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
		ESP_ERROR_CHECK(esp_wifi_connect());
	}
}

static void event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT) {
		switch (event_id) {
		case WIFI_EVENT_STA_START:
		case WIFI_EVENT_STA_DISCONNECTED:	/* Step 1 */
			ESP_LOGI(tag, "Disconnected, scanning APs...");
			is_connected = false;
			event_start_scan();
			break;
		case WIFI_EVENT_SCAN_DONE:		/* Step 2 */
			ESP_LOGI(tag, "Scanning is done, looking for the AP");
			event_find_ap_from_list();
			break;
		case WIFI_EVENT_STA_CONNECTED:		/* Step 3 */
			ESP_LOGI(tag, "Connection established.");
			break;
		}
		return;
	}

	/* Step 4 */
	if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		char ip_address[16];
		ip_event_got_ip_t *event = (ip_event_got_ip_t*)event_data;
		snprintf(ip_address, sizeof(ip_address), IPSTR,
			IP2STR(&event->ip_info.ip));
		ESP_LOGI(tag, "Wifi got ip: %s", ip_address);
		is_connected = true;
	}
}

int connect_to_server(int *socketfd, const char *server, uint32_t port)
{
	struct sockaddr_in dest = {
		.sin_family = AF_INET,
		.sin_port = htons(port),
	};
	int tmp;

	dest.sin_addr.s_addr = inet_addr(server);

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
	esp_netif_create_default_wifi_sta();
	ESP_ERROR_CHECK(esp_wifi_init(&config));
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

	is_connected = false;
	init_done = false;
	esp_wifi_stop();
}

const char *wifi_get_current_uuid(void)
{
	return current_uuid;
}
