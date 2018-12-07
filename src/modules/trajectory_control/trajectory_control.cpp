#include <stdlib.h>
#include <string.h>

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>
#include <systemlib/hysteresis/hysteresis.h>

#include <float.h>
#include <mathlib/mathlib.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>

#include "trajectory_control.h"

extern "C" __EXPORT int trajectory_control_main(int argc, char *argv[]);

namespace trajectory_control
{
    TrajectoryControl *g_trajectory_contrrol;
} 

TrajectoryControl::TrajectoryControl():
					_trajectory_control_task(-1),
					_task_should_exit{false},
					_vehicle_command_pub(nullptr),
					_control_mode_sub(-1)
{
	memset(&_vehicle_command, 0, sizeof(_vehicle_command));
	memset(&_sp_man, 0, sizeof(_sp_man));
}

TrajectoryControl::~TrajectoryControl()
{
	if (_trajectory_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_trajectory_control_task);
				break;
			}
		} while (_trajectory_control_task != -1);
	}

	trajectory_control::g_trajectory_contrrol = nullptr;
}

void
TrajectoryControl::task_main()
{

	//publish 消息
	_vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_vehicle_command);

	//subscribe 消息
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));

	_sp_man_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    


	while(!_task_should_exit)
	{
		usleep(4000);
		bool updated;

		orb_check(_control_mode_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
		}

        orb_check(_sp_man_sub, &updated);
        if (updated) {
			orb_copy(ORB_ID(manual_control_setpoint), _sp_man_sub, &_sp_man);
        }
		//判断是否开启环形飞行控制任务
		if(_sp_man.aux1 < -0.8f){
        	_start_circle_control = false;
    	}
    	if(_sp_man.aux1 >  0.8f){
       		_start_circle_control = true;
    	}

		//任务开始，开启offboard模式
		if(_start_circle_control && !_control_mode.flag_control_offboard_enabled)
		{
			send_vehicle_command(true);//true :设置offboard模式  ； false:设置loiter模式
		}

		//判断任务是否开启，开启后开始执行任务
		if(_start_circle_control && _control_mode.flag_control_offboard_enabled)
		{
			_circle_control_command_start_success = true;
		}

		//任务执行结束后，切换至loiter模式
		if(_circle_control_command_start_success && !_start_circle_control)
		{
			if(!_control_mode.flag_control_auto_enabled)
			{
				send_vehicle_command(false);
			}
			else
			{
				_circle_control_command_start_success = false;
			}
			
		}
	}
}

void
TrajectoryControl::task_main_trampoline(int argc, char *argv[])
{
	trajectory_control::g_trajectory_contrrol->task_main();
}

int
TrajectoryControl::start()
{
	ASSERT(_trajectory_control_task == -1);

	/* start the task */
	_trajectory_control_task = px4_task_spawn_cmd("trajectory_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_POSITION_CONTROL,
					   1900,
					   (px4_main_t)&TrajectoryControl::task_main_trampoline,
					   nullptr);

	if (_trajectory_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void 
TrajectoryControl::send_vehicle_command(bool is_offboard_mode)
{
	if(is_offboard_mode)
	{
		_vehicle_command.timestamp = hrt_absolute_time();
		_vehicle_command.param5 = 0;
		_vehicle_command.param6 = 0;

		_vehicle_command.param1 = (float)0b00000001;//(float)new_mode.base_mode;
		_vehicle_command.param2 = (float)6;//(float)custom_mode.main_mode;
		_vehicle_command.param3 = 0;//(float)custom_mode.sub_mode;
		_vehicle_command.param4 = 0;
		_vehicle_command.param7 = 0;
		_vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
		_vehicle_command.target_system = 1;//new_mode.target_system;
		_vehicle_command.target_component = 0;
		_vehicle_command.source_system = 11;//msg->sysid;
		_vehicle_command.source_component = 1;//msg->compid;
		_vehicle_command.confirmation = 1;
		_vehicle_command.from_external = true;

		_vehicle_command.cmd_start_global_syn_time1 = 0;
		_vehicle_command.cmd_start_global_syn_time2 = 0;
		_vehicle_command.cmd_start_global_syn_time3 = 0;
		_vehicle_command.cmd_start_global_syn_time4 = 0;
		_vehicle_command.cmd_start_global_syn_time5 = 0;
		_vehicle_command.cmd_start_global_syn_time6 = 0;
		_vehicle_command.cmd_start_global_syn_time7 = 0;
		_vehicle_command.cmd_start_global_syn_time8 = 0;

		orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &_vehicle_command);	
	}
	else
	{
		_vehicle_command.timestamp = hrt_absolute_time();
		_vehicle_command.param5 = 0;
		_vehicle_command.param6 = 0;

		_vehicle_command.param1 = 0b00000001;//(float)new_mode.base_mode;
		_vehicle_command.param2 = 4;//(float)custom_mode.main_mode;
		_vehicle_command.param3 = 3;//(float)custom_mode.sub_mode;
		_vehicle_command.param4 = 0;
		_vehicle_command.param7 = 0;
		_vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
		_vehicle_command.target_system = 1;//new_mode.target_system;
		_vehicle_command.target_component = 0;
		_vehicle_command.source_system = 11;//msg->sysid;
		_vehicle_command.source_component = 0;//msg->compid;
		_vehicle_command.confirmation = 1;
		_vehicle_command.from_external = true;

		_vehicle_command.cmd_start_global_syn_time1 = 0;
		_vehicle_command.cmd_start_global_syn_time2 = 0;
		_vehicle_command.cmd_start_global_syn_time3 = 0;
		_vehicle_command.cmd_start_global_syn_time4 = 0;
		_vehicle_command.cmd_start_global_syn_time5 = 0;
		_vehicle_command.cmd_start_global_syn_time6 = 0;
		_vehicle_command.cmd_start_global_syn_time7 = 0;
		_vehicle_command.cmd_start_global_syn_time8 = 0;

		orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &_vehicle_command);		
	}
}

int trajectory_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: trajectory {start|stop}");
		return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (trajectory_control::g_trajectory_contrrol != nullptr) {
			warnx("already running");
			return 1;
		}

        trajectory_control::g_trajectory_contrrol = new TrajectoryControl;

		if (trajectory_control::g_trajectory_contrrol == nullptr) {
			warnx("alloc failed");
			return 1;
		}

        if (OK != trajectory_control::g_trajectory_contrrol->start()) {
			delete trajectory_control::g_trajectory_contrrol;
			trajectory_control::g_trajectory_contrrol = nullptr;
			warnx("start failed");
			return 1;
		}

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
		if (trajectory_control::g_trajectory_contrrol == nullptr) {
			warnx("not running");
			return 1;
		}

		delete trajectory_control::g_trajectory_contrrol;
		trajectory_control::g_trajectory_contrrol = nullptr;
		return 0;
	}
    
    warnx("unrecognized command");
	return 1;
}