
px4_add_module(
	MODULE drivers__rstcan
	MAIN rstcan
	COMPILE_FLAGS
		-Os
	SRCS
		rstcan_manager.cpp
		rstcan_node.cpp
		rstcan_node_bypass.cpp
		rstcan_node_4g_ctrl.cpp
		rstcan_node_rgbled.cpp
		rstcan_node_camera.cpp
		rstcan_node_radar.cpp
        stm32_can_drv.cpp
	DEPENDS
		platforms__common
	)

target_link_libraries(drivers__rstcan ${CMAKE_SOURCE_DIR}/src/drivers/rstcan/librstcan-m.a)

