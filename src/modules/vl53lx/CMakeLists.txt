cmake_minimum_required(VERSION 3.13.1)

# For the sake of demonstration, we add the driver directory as a zephyr module
# by hand. If your driver is a project that's managed by west, you can remove this line.
# list(APPEND ZEPHYR_EXTRA_MODULES ${PROJECT_SOURCE_DIR}/src/modules/vl53lx)

if(CONFIG_VL53LX)

	zephyr_include_directories()
	# zephyr_library() #BUG gives ninja errors at compile time using compatible = "st,vl53lx" but not using compatible = "gpio-leds"

	# vendor sources
	add_subdirectory(${PROJECT_SOURCE_DIR}/vendor/st/vl53lx ${PROJECT_SOURCE_DIR}/vendor/st/vl53lx)
	zephyr_include_directories(${PROJECT_SOURCE_DIR}/vendor/st/vl53lx)

	# module sources
	zephyr_library_sources(vl53lx.c)
	zephyr_library_sources(vl53lx_platform.c)
	zephyr_library_sources(vl53lx_platform_ipp.c)
	zephyr_library_sources(vl53lx_platform_log.c)

endif()

