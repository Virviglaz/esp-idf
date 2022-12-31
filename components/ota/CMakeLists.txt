idf_build_get_property(target IDF_TARGET)

set(srcs
	"ota.c"
)

set(include_dirs "include")

idf_component_register(
	SRCS "${srcs}"
	INCLUDE_DIRS "${include_dirs}" "../include"
	PRIV_INCLUDE_DIRS "${priv_include_dirs}"
	PRIV_REQUIRES
		app_update
		esp_netif
		esp_timer
		nvs_flash
		wifi
)