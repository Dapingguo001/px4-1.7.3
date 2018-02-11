############################################################################
# Copyright(C),2015-2018, Robsense Tech. All rights reserved.
# FileName:    rst_dev_manage.txt
# Author:      LiuYi
# Version:     0.1
# Date:        2018-2-8 15:29:06
# Last Modify: 2018-2-8 15:29:06
# Description: 
############################################################################
px4_add_module(
	MODULE modules__rst_dev_manage
	MAIN rst_dev_manage
	STACK_MAIN 2000
	SRCS
		rst_dev_manage.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
