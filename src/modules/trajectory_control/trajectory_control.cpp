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
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>


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
	memset(&_offboard_control_mode, 0, sizeof(_offboard_control_mode));
	memset(&_gps_pos, 0, sizeof(_gps_pos));
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
	_offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &_offboard_control_mode);
	//subscribe 消息
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_sp_man_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    
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

        orb_check(_gps_pos_sub, &updated);
        if (updated) {
			orb_copy(ORB_ID(vehicle_gps_position), _gps_pos_sub, &_gps_pos);
        }

        orb_check(_global_pos_sub, &updated);
        if (updated) {
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
        }				

		if(_gps_pos.fix_type > 3 && hrt_absolute_time() < _gps_pos.timestamp + 1 * 1000 *1000)
		{
			_gps_valid = true;
		}
		else
		{
			_gps_valid = false;
		}

		//判断是否开启环形飞行控制任务
		if(_sp_man.aux1 < -0.8f){
        	_start_circle_control = false;
    	}
    	if(_sp_man.aux1 >  0.8f){
       		_start_circle_control = true;
			//初始化环形命令
			circle_init(_circle_trajectory_command_bag);
    	}

		//任务开始，开启offboard模式
		if(_start_circle_control && !_control_mode.flag_control_offboard_enabled && _gps_valid)
		{
			send_vehicle_command(true);//true :设置offboard模式  ； false:设置loiter模式
			send_offboard_control_mode();
		}

		//判断任务是否开启，开启后开始执行任务
		if(_start_circle_control && _control_mode.flag_control_offboard_enabled && _gps_valid)
		{
			_circle_control_command_start_success = true;

			send_offboard_control_mode();
		}

		//任务执行结束后，gps有效，切换至loiter模式
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

void 
TrajectoryControl::send_offboard_control_mode()
{
	_offboard_control_mode.ignore_thrust             = true;
	_offboard_control_mode.ignore_attitude           = true;
	_offboard_control_mode.ignore_bodyrate           = true;
	_offboard_control_mode.ignore_position           = false;
	_offboard_control_mode.ignore_velocity           = true;
	_offboard_control_mode.ignore_acceleration_force = true;

	_offboard_control_mode.timestamp = hrt_absolute_time();

	orb_publish(ORB_ID(offboard_control_mode), _offboard_control_mode_pub, &_offboard_control_mode);
}

void
TrajectoryControl::do_circle(circle_trajectory_command_bag   cir_tra_com_bag)
{
	struct map_projection_reference_s ref_pos;
	//计算三维环形轨道位置转换矩阵
	float level_2D_position[3];
	float level_3D_position_NEU[3]; 
	double level_3D_position_LLH[3];
	static float theta_delta;
	static bool calculate_cos_angle_dec = true;
	float dt = (hrt_absolute_time() - _last_time_stamp)/(1 * 1000 * 1000);

	level_2D_position[0] = cir_tra_com_bag.circle_R * cosf(cir_tra_com_bag.initial_theta + theta_delta);
	level_2D_position[1] = cir_tra_com_bag.circle_R * sinf(cir_tra_com_bag.initial_theta + theta_delta);
	level_2D_position[2] = 0;

	//水平位置转3D位置
	level_3D_position_NEU[0] = cir_tra_com_bag.circle_3d_tranfer_matrix[0][0] * level_2D_position[0] + cir_tra_com_bag.circle_3d_tranfer_matrix[0][1] * level_2D_position[1]
						   		+ cir_tra_com_bag.circle_3d_tranfer_matrix[0][2] * level_2D_position[2];

	level_3D_position_NEU[1] = cir_tra_com_bag.circle_3d_tranfer_matrix[1][0] * level_2D_position[0] + cir_tra_com_bag.circle_3d_tranfer_matrix[1][1] * level_2D_position[1]
						   		+ cir_tra_com_bag.circle_3d_tranfer_matrix[1][2] * level_2D_position[2];

	level_3D_position_NEU[2] = cir_tra_com_bag.circle_3d_tranfer_matrix[2][0] * level_2D_position[0] + cir_tra_com_bag.circle_3d_tranfer_matrix[2][1] * level_2D_position[1]
						   		+ cir_tra_com_bag.circle_3d_tranfer_matrix[2][2] * level_2D_position[2];

	ref_pos.lat_rad = cir_tra_com_bag.center_of_circle_lat;
	ref_pos.lon_rad = cir_tra_com_bag.center_of_circle_lon;
	map_projection_reproject(&ref_pos, level_3D_position_NEU[0], level_3D_position_NEU[1], 
								&level_3D_position_LLH[0], &level_3D_position_LLH[1]);
	level_3D_position_LLH[2] = level_3D_position_NEU[2] + cir_tra_com_bag.center_of_circle_hgt;

	//计算加减速;
	if(calculate_cos_angle_dec)
	{
		_cos_angle_dec = (cir_tra_com_bag.start_angle_vel * cir_tra_com_bag.start_angle_vel - 
				cir_tra_com_bag.end_angle_vel * cir_tra_com_bag.end_angle_vel)/(cir_tra_com_bag.max_angle_acc);
	}

	if((cir_tra_com_bag.angle_sum_accomplish - theta_delta) > (_cos_angle_dec + 2/180*(float)M_PI))
	{
		cir_tra_com_bag.angle_vel = cir_tra_com_bag.angle_vel + cir_tra_com_bag.max_angle_acc * dt;  //初始加速阶段
	}
	else
	{
		calculate_cos_angle_dec = false;
		if (cir_tra_com_bag.angle_vel > cir_tra_com_bag.end_angle_vel)
		{
			cir_tra_com_bag.angle_vel = cir_tra_com_bag.angle_vel - cir_tra_com_bag.max_angle_acc * dt;  //终止减速阶段
		}
		if (cir_tra_com_bag.angle_vel < 1/180*(float)M_PI)
		{
			cir_tra_com_bag.angle_vel = 1/180*(float)M_PI;
		}        
	}

	if (cir_tra_com_bag.angle_vel >  cir_tra_com_bag.set_angle_vel)
	{
		cir_tra_com_bag.angle_vel = _last_angle_vel;
	}

	theta_delta = theta_delta + cir_tra_com_bag.angle_vel * dt;

	_last_angle_vel = cir_tra_com_bag.angle_vel;

	_last_time_stamp = hrt_absolute_time();
}

void
TrajectoryControl::circle_init(circle_trajectory_command_bag   cir_tra_com_bag)
{
	//计算环形轨道半径
	struct map_projection_reference_s ref_pos;
	ref_pos.lat_rad = _global_pos.lat * M_DEG_TO_RAD;
	ref_pos.lon_rad = _global_pos.lon * M_DEG_TO_RAD;
	map_projection_project(&ref_pos,
					       cir_tra_com_bag.center_of_circle_lat, cir_tra_com_bag.center_of_circle_lon,
					       &cir_tra_com_bag.circle_R_n, &cir_tra_com_bag.circle_R_e);
	//计算半径					   
	cir_tra_com_bag.circle_R = sqrt(cir_tra_com_bag.circle_R_n * cir_tra_com_bag.circle_R_n + 
									cir_tra_com_bag.circle_R_e * cir_tra_com_bag.circle_R_e);

	//计算初始角度theta
	if(cir_tra_com_bag.circle_R_n > -0.01f && cir_tra_com_bag.circle_R_n < 0.01f)
	{
		if(cir_tra_com_bag.circle_R_e > -0.01f && cir_tra_com_bag.circle_R_e < 0.01f)
		{
			cir_tra_com_bag.initial_theta = 0;
		}
		else if(cir_tra_com_bag.circle_R_e < 0)
		{
			cir_tra_com_bag.initial_theta = (float)M_PI * 3/2;
		}
		else
		{
			cir_tra_com_bag.initial_theta = (float)M_PI/2;
		}
	}
	else if(cir_tra_com_bag.circle_R_n > 0)
	{
		if(cir_tra_com_bag.circle_R_e > -0.01f && cir_tra_com_bag.circle_R_e < 0.01f)
		{
			cir_tra_com_bag.initial_theta = 0;
		}
		else if(cir_tra_com_bag.circle_R_e < 0)
		{
			cir_tra_com_bag.initial_theta = atanf(cir_tra_com_bag.circle_R_n/cir_tra_com_bag.circle_R_e) + 2.0f * (float)M_PI;
		}
		else
		{
			cir_tra_com_bag.initial_theta = atanf(cir_tra_com_bag.circle_R_n/cir_tra_com_bag.circle_R_e);
		}
	}
	else
	{
		if(cir_tra_com_bag.circle_R_e > -0.01f && cir_tra_com_bag.circle_R_e < 0.01f)
		{
			cir_tra_com_bag.initial_theta = (float)M_PI;
		}
		else
		{
			cir_tra_com_bag.initial_theta = atanf(cir_tra_com_bag.circle_R_n/cir_tra_com_bag.circle_R_e) + (float)M_PI;
		}
	}

	cir_tra_com_bag.circle_3d_tranfer_matrix[0][0] = cosf(cir_tra_com_bag.circle_dip_angle_est);
	cir_tra_com_bag.circle_3d_tranfer_matrix[0][1] = sinf(cir_tra_com_bag.circle_dip_angle_north) * sinf(cir_tra_com_bag.circle_dip_angle_est);
	cir_tra_com_bag.circle_3d_tranfer_matrix[0][2] = -cosf(cir_tra_com_bag.circle_dip_angle_north)*sinf(cir_tra_com_bag.circle_dip_angle_est);
	
	cir_tra_com_bag.circle_3d_tranfer_matrix[1][0] = 0.0f;
	cir_tra_com_bag.circle_3d_tranfer_matrix[1][1] = cosf(cir_tra_com_bag.circle_dip_angle_north);
	cir_tra_com_bag.circle_3d_tranfer_matrix[1][2] = sinf(cir_tra_com_bag.circle_dip_angle_north);

	cir_tra_com_bag.circle_3d_tranfer_matrix[2][0] = sinf(cir_tra_com_bag.circle_dip_angle_est);
	cir_tra_com_bag.circle_3d_tranfer_matrix[2][1] = -sinf(cir_tra_com_bag.circle_dip_angle_north)*cosf(cir_tra_com_bag.circle_dip_angle_est);
	cir_tra_com_bag.circle_3d_tranfer_matrix[2][2] = cosf(cir_tra_com_bag.circle_dip_angle_north)*cosf(cir_tra_com_bag.circle_dip_angle_est);

	cir_tra_com_bag.max_angle_acc = (float)10/180*(float)M_PI;
}

void
TrajectoryControl::tragectory_publish(circle_trajectory_command_bag   cir_tra_com_bag)
{
	
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