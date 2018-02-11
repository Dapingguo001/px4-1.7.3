/*********************************************************************************
 *Copyright(C),2015-2018, Robsense Tech. All rights reserved.
 *FileName:    rst_dev_manage.hpp
 *Author:      LiuYi
 *Version:     0.1
 *Date:        2018-2-8 15:29:06
 *Last Modify: 2018-2-8 15:29:06
 *Description: 
**********************************************************************************/

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <poll.h>
#include <mathlib/mathlib.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/rst_dev_cmd.h>
#include <uORB/topics/rstcan_camera.h>
#include <drivers/drv_rstcan.h>


class RSTdeviceManage
{
public:
	/**
	 * Constructor
	 */
	RSTdeviceManage();

	/**
	 * Destructor, also kills task.
	 */
	~RSTdeviceManage();

	/**
	 * Start the task.
	 *
	 * @return		OK on success.
	 */
	int			start();

	/**
	 * Stop the task.
	 */
	void		stop();

private:

	bool		_task_should_exit;		/**< if true, task should exit */
	int			_main_task;				/**< handle for task */

	int			_dev_cmd_sub;

	orb_advert_t	_dev_cmd_pub;


	void		task_main();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	void handle_rst_camera(const struct rst_dev_cmd_s &cmd_mavlink);
};
