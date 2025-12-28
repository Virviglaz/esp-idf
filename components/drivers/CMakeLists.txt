idf_build_get_property(target IDF_TARGET)

set(srcs "")
set(public_include "include")

list(APPEND srcs
	"ltc6804.c"
	"HD44780.c"
	"esp_stepper.cpp"
	"esp32_spi.cpp"
    )

set(include_dirs "include")

idf_component_register(
	SRCS ${srcs}
	INCLUDE_DIRS ${public_include}
	PRIV_INCLUDE_DIRS "${priv_include_dirs}"
	PRIV_REQUIRES esp_timer
	REQUIRES driver
	REQUIRES esp_driver_gpio
	REQUIRES esp_driver_spi
	#REQUIRES esp_driver_i2c
)