/*********************************************************************************
 *Copyright(C),2015-2018, Robsense Tech. All rights reserved.
 *FileName:    rst_dev_manage.cpp
 *Author:      LiuYi
 *Version:     0.1
 *Date:        2018-2-8 15:29:06
 *Last Modify: 2018-2-8 15:29:06
 *Description: 
**********************************************************************************/

#include "rst_dev_manage.hpp"

namespace rst_dev_manage
{
RSTdeviceManage	*g_rst_dev_manage;
}

RSTdeviceManage::RSTdeviceManage() :
	_task_should_exit(false),
	_main_task(-1)
{

}

RSTdeviceManage::~RSTdeviceManage()
{

	if (_main_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_main_task);
				break;
			}
		} while (_main_task != -1);
	}

	rst_dev_manage::g_rst_dev_manage = nullptr;
}

int
RSTdeviceManage::start()
{

	ASSERT(_main_task == -1);

	/* start the task */
	_main_task = px4_task_spawn_cmd("rst_dev_manage",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT + 15,
					1600,
					(px4_main_t)&RSTdeviceManage::task_main_trampoline,
					nullptr);

	if (_main_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;

}

void
RSTdeviceManage::stop()
{
	if (rst_dev_manage::g_rst_dev_manage != nullptr) {
		delete (rst_dev_manage::g_rst_dev_manage);
	}
}


void
RSTdeviceManage::task_main()
{
	// Polling sources
	_dev_cmd_sub = orb_subscribe(ORB_ID(rst_dev_cmd));
	struct rst_dev_cmd_s dev_cmd = {};

	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = _dev_cmd_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 20ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20);

		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			continue;
		}

		/* handle MAVLink command */
		if (fds[0].revents & POLLIN) {

			orb_copy(ORB_ID(rst_dev_cmd), _dev_cmd_sub, &dev_cmd);

			switch (dev_cmd.cmd) {
			
			case rst_dev_cmd_s::RST_DEV_CAMERA:	
				handle_rst_camera(dev_cmd);
				break;

			//-------------------------------
			/*add other device command here*/
			//-------------------------------

			default: 
				break;
			}
		}

	}

	PX4_INFO("Exiting.");
	_main_task = -1;

}

void
RSTdeviceManage::task_main_trampoline(int argc, char *argv[])
{
	rst_dev_manage::g_rst_dev_manage->task_main();
}

static int usage()
{
	PX4_INFO("usage: rst_dev_manage {start|stop}\n");
	return 1;
}

extern "C" __EXPORT int rst_dev_manage_main(int argc, char *argv[]);

int rst_dev_manage_main(int argc, char *argv[])
{
	if (argc < 2) {
		return usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (rst_dev_manage::g_rst_dev_manage != nullptr) {
			PX4_WARN("already running");
			return 0;
		}

		rst_dev_manage::g_rst_dev_manage = new RSTdeviceManage();

		if (rst_dev_manage::g_rst_dev_manage == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		rst_dev_manage::g_rst_dev_manage->start();
		return 0;
	}

	if (rst_dev_manage::g_rst_dev_manage == nullptr) {
		PX4_WARN("not running");
		return 1;

	} else if (!strcmp(argv[1], "stop")) {
		rst_dev_manage::g_rst_dev_manage->stop();

	} else {
		return usage();
	}

	return 0;
}

//----------------------------------------------------------相机相关---------------------------------------------------
void
RSTdeviceManage::handle_rst_camera(const struct rst_dev_cmd_s &cmd_mavlink)
{
	const int num = 2;//camera total number.
	int index = 0;

	/* advertise topic */
	ptz_ctrl_t ptz;
	memset(&ptz, 0, sizeof(ptz));
	struct rstcan_camera_s camera;
	memset(&camera, 0, sizeof(camera));
	static orb_advert_t camera_pub[num];

	static bool bAdvertised = false;
	if (!bAdvertised) {
		//第一个发布返回的instance一定是0，如果要测试第二个节点，必须发布第二次，第三个节点以此类推
		for(int i = 0; i < num; i++)
		{
			orb_advert_t tmp_pub = orb_advertise_multi(ORB_ID(rstcan_camera), &camera, &index, ORB_PRIO_DEFAULT);
			camera_pub[index] = tmp_pub;
			orb_publish(ORB_ID(rstcan_camera), camera_pub[index], &camera);
			::printf ("instance: %d\n", index);
		}
		bAdvertised = true;
	}

	switch (cmd_mavlink.data[0]){//|up or down(0:default,1:stop,2:up,3:down)
	case 1:	camera.cmd = rstcan_camera_s::RST_CAMERA_PTZ_CTRL;
			break;
	case 2:	camera.cmd = rstcan_camera_s::RST_CAMERA_PTZ_CTRL;
			ptz.pitch = 1;
			break;
	case 3:	camera.cmd = rstcan_camera_s::RST_CAMERA_PTZ_CTRL;
			ptz.pitch = -1;
			break;
	default:break;
	}
	switch (cmd_mavlink.data[1]){//|left or right(0:default,1:stop,2:left,3:right)
	case 1:	camera.cmd = rstcan_camera_s::RST_CAMERA_PTZ_CTRL;
			break;
	case 2:	camera.cmd = rstcan_camera_s::RST_CAMERA_PTZ_CTRL;
			ptz.yaw = -1;
			break;
	case 3:	camera.cmd = rstcan_camera_s::RST_CAMERA_PTZ_CTRL;
			ptz.yaw = 1;
			break;
	default:break;
	}
	switch (cmd_mavlink.data[2]){//|camera shot(0:default,1:shot)
	case 1:	camera.cmd = rstcan_camera_s::RST_CAMERA_SNAPSHOT;
			break;
	default:break;
	}
	switch (cmd_mavlink.data[3]){//|camera video(0:default,1:stop,2:recording)
	case 1:	camera.cmd = rstcan_camera_s::RST_CAMERA_RECORD_STOP;
			break;
	case 2:	camera.cmd = rstcan_camera_s::RST_CAMERA_RECORD_START;
			break;
	default:break;
	}
	switch (cmd_mavlink.data[4]){//|zoom in or out(0:default,1:stop,2:zoom in,3:zoom out)
	case 1:	camera.cmd = rstcan_camera_s::RST_CAMERA_ZOOM_STOP;
			break;
	case 2:	camera.cmd = rstcan_camera_s::RST_CAMERA_ZOOM_IN;
			break;
	case 3:	camera.cmd = rstcan_camera_s::RST_CAMERA_ZOOM_OUT;
			break;
	default:break;
	}
	if (cmd_mavlink.data[5] > 0 && cmd_mavlink.data[5] <= num){//0:none camera,1:down camera,2:up camera)
		index = cmd_mavlink.data[5] - 1;
	} else {
		index = 0;
	}
	switch (cmd_mavlink.data[6]){//|回中(0:default,1:回中)
	case 1:	camera.cmd = rstcan_camera_s::RST_CAMERA_PTZ_CTRL;
			ptz.mode = RST_PTZ_CTRL_MIDDLE_MODE;
			break;
	default:ptz.mode = RST_PTZ_CTRL_HEAD_LOCK_MODE;
			break;
	}
	

	if (camera.cmd == rstcan_camera_s::RST_CAMERA_PTZ_CTRL){//姿态控制时，添加姿态信息pitch、yaw
		memcpy(&camera.data, &ptz, sizeof(ptz));
	}
	/* publish topic */
	orb_publish(ORB_ID(rstcan_camera), camera_pub[index], &camera);
	::printf ("camera- %d: %d %d %d %d %d %d\n", index, cmd_mavlink.data[0], 
														cmd_mavlink.data[1],
														cmd_mavlink.data[2],
														cmd_mavlink.data[3],
														cmd_mavlink.data[4],
														cmd_mavlink.data[5]);

}
