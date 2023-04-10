idf_build_get_property(target IDF_TARGET)

set(srcs
	"ltc6804.c"
	"HD44780.c"
	"esp_encoder.cpp"
	"esp_stepper.cpp"
	"esp32_spi.cpp"
)

set(include_dirs "include")

idf_component_register(
	SRCS "${srcs}"
	INCLUDE_DIRS "${include_dirs}"
	PRIV_INCLUDE_DIRS "${priv_include_dirs}"
	PRIV_REQUIRES esp_timer
	REQUIRES driver
)