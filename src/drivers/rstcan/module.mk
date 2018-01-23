#
# robsense driver for can
#

MODULE_COMMAND	 = rstcan

SRCS		 = rstcan_manager.cpp \
			   rstcan_node.cpp\
			   rstcan_node_bypass.cpp\
			   rstcan_node_4g_ctrl.cpp\
			   rstcan_node_rgbled.cpp\
			   rstcan_node_devman.cpp\
			   stm32_can_drv.cpp

MODULE_STACKSIZE = 1200

#LIBS += -l$(shell pwd)/librstcan-m.a

#INCLUDE_DIRS	+= $(NUTTX_SRC)/arch/arm/src/stm32 $(NUTTX_SRC)/arch/arm/src/common

MAXOPTIMIZATION	 = -Os
