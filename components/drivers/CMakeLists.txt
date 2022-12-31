idf_build_get_property(target IDF_TARGET)

set(srcs
	"ltc6804.c"
)

set(include_dirs "include")

idf_component_register(
	SRCS "${srcs}"
	INCLUDE_DIRS "${include_dirs}"
	PRIV_INCLUDE_DIRS "${priv_include_dirs}"
)