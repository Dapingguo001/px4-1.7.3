#include <stdlib.h>
#include <string.h>

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <drivers/drv_hrt.h>

#include <systemlib/param/param.h>

#include <uORB/topics/rst_swarm_link_light_control_receive.h>
#include <uORB/topics/rst_swarm_link_fc_statue_receive.h>
#include <uORB/topics/rst_swarm_link_light_control_component.h>

#include <uORB/topics/rst_swarm_link_fc_statue_send.h>

#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/battery_status.h>

#include <uORB/topics/led_control.h>

#include <uORB/topics/vehicle_local_position.h>

#include <drivers/drv_led.h>
#include "swarm_link.h"


extern "C" __EXPORT int swarm_link_main(int argc, char *argv[]);

namespace swarm_link
{

    Swarm_Link	*g_swarm_link;
}

Swarm_Link::Swarm_Link():
    _task_should_exit(false),
    _swarm_link_task(-1)
{

}

Swarm_Link::~Swarm_Link()
{
	if (_swarm_link_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_swarm_link_task);
				break;
			}
		} while (_swarm_link_task != -1);
	}

	swarm_link::g_swarm_link = nullptr;
}

int
Swarm_Link::start()
{
    ASSERT(_swarm_link_task == -1);
    
        /* start the task */
        _swarm_link_task = px4_task_spawn_cmd("swarm_link",
                           SCHED_DEFAULT,
                           SCHED_PRIORITY_DEFAULT,
                           1700,
                           (px4_main_t)&Swarm_Link::task_main_trampoline,
                           nullptr);
    
        if (_swarm_link_task < 0) {
            warn("task start failed");
            return -errno;
        }
    
        return OK;
}

void
Swarm_Link::task_main_trampoline(int argc, char *argv[])
{
    swarm_link::g_swarm_link->task_main();
}

void
Swarm_Link::task_main()
{
    int32_t calibration_id;
    bool updated;
    param_get(param_find("CAL_MAG0_ID"), &(calibration_id));
    //订阅消息
//    _swarm_link_light_control_receive_sub = orb_subscribe(ORB_ID(rst_swarm_link_light_control_receive));
    _swarm_link_fc_statue_receive_sub = orb_subscribe(ORB_ID(rst_swarm_link_fc_statue_receive));

    _vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
    _home_position_sub = orb_subscribe(ORB_ID(home_position));
    _battery_status_sub = orb_subscribe(ORB_ID(battery_status));
    _local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

    //公告消息
//    led_control_pub = orb_advertise(ORB_ID(led_control), &_led_control);
    //led_control_pub = orb_advertise_queue(ORB_ID(led_control), &_led_control, LED_UORB_QUEUE_LENGTH);
//    light_control_component_pub = orb_advertise(ORB_ID(rst_swarm_link_light_control_component), 
 //                                   &_light_control_component);

    _swarm_link_fc_statue_send_sub = orb_advertise(ORB_ID(rst_swarm_link_fc_statue_send), 
                                        &_fc_statue_send);
                                
    memset(&_fc_statue_send,0,sizeof(_fc_statue_send));

    while(!_task_should_exit)
    {

        orb_check(_vehicle_gps_position_sub, &updated);
        if(updated)
        {
            orb_copy(ORB_ID(vehicle_gps_position), _vehicle_gps_position_sub, &_gps_position);
        }

        orb_check(_local_position_sub, &updated);
        if(updated)
        {
            orb_copy(ORB_ID(vehicle_local_position), _local_position_sub, &_local_position);
        }

        orb_check(_home_position_sub, &updated);
        if(updated)
        {
            orb_copy(ORB_ID(home_position), _home_position_sub, &_home_position);
        }

        orb_check(_battery_status_sub, &updated);
        if(updated)
        {
            orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
        }

        orb_check(_swarm_link_fc_statue_receive_sub, &updated);
        if(updated)
        {
            orb_copy(ORB_ID(rst_swarm_link_fc_statue_receive), _swarm_link_fc_statue_receive_sub, 
                    &_fc_statue_receive);

            if(_fc_statue_receive.param1)
            {
                if(_gps_position.fix_type == 6)
                {
                    _fc_statue_send.fix_type = 3;
                }
                else if(_gps_position.fix_type == 5)
                {
                    _fc_statue_send.fix_type = 2;
                }
                else if(_gps_position.fix_type == 4)
                {
                    _fc_statue_send.fix_type = 1;
                }
                else
                {
                    _fc_statue_send.fix_type = 0;
                }
                _fc_statue_send.lat = _gps_position.lat;
                _fc_statue_send.lon = _gps_position.lon;
                _fc_statue_send.alt = (int32_t)(_local_position.z * 1E3f);
                _fc_statue_send.relative_alt = (int32_t)((_local_position.z - _home_position.alt) * 1E3f);
                _fc_statue_send.voltage_battery = (uint16_t)(_battery_status.voltage_filtered_v * 1E2f);
                _fc_statue_send.battery_remaining = (uint8_t)(_battery_status.remaining * 100);//百分治

                if(calibration_id)
                {
                    _fc_statue_send.param_index = 1;
                }
                else
                {
                    _fc_statue_send.param_index = 0;
                }
                _fc_statue_send.param1 = 0;
                _fc_statue_send.param2 = 0;
                _fc_statue_send.param3 = 0;
                _fc_statue_send.param4 = 0;
                orb_publish(ORB_ID(rst_swarm_link_fc_statue_send), _swarm_link_fc_statue_send_sub,
                             &_fc_statue_send);
            }
        }        


        usleep(50000);
    }

}


int swarm_link_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: swarm_link {start|stop}");
		return 1;
    }
    if (!strcmp(argv[1], "start")) {
        
        if (swarm_link::g_swarm_link != nullptr) {
            warnx("already running");
            return 1;
        }

        swarm_link::g_swarm_link = new Swarm_Link;

        if (swarm_link::g_swarm_link == nullptr) {
            warnx("alloc failed");
            return 1;
        }

        if (OK != swarm_link::g_swarm_link->start()) {
            delete swarm_link::g_swarm_link;
            swarm_link::g_swarm_link = nullptr;
            warnx("start failed");
            return 1;
        }

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (swarm_link::g_swarm_link == nullptr) {
            warnx("not running");
            return 1;
        }

        delete swarm_link::g_swarm_link;
        swarm_link::g_swarm_link = nullptr;
        return 0;
    }

    warnx("unrecognized command");
	return 1;
}

