#include <uORB/topics/rst_swarm_link_light_control_receive.h>
#include <uORB/topics/rst_swarm_link_fc_statue_receive.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/rst_swarm_link_fc_statue_send.h>
#include <uORB/topics/vehicle_local_position.h>
#include <systemlib/param/param.h>
#include <uORB/topics/vehicle_global_position.h>




class Swarm_Link{

public:
    Swarm_Link();
    ~Swarm_Link();

    int	start();


    param_t			_param_cal_mag0_id;

    int _swarm_link_light_control_receive_sub{-1};
    int _swarm_link_fc_statue_receive_sub{-1};

    int _vehicle_gps_position_sub{-1};
    int _home_position_sub{-1};
    int _battery_status_sub{-1};
    int _local_position_sub{-1};
    int _global_position_sub{-1};


    struct rst_swarm_link_fc_statue_receive_s  _fc_statue_receive;
    struct rst_swarm_link_light_control_receive_s _light_control_receive;
    struct vehicle_gps_position_s _gps_position;
    struct home_position_s _home_position;
    struct vehicle_global_position_s _global_position;
    struct battery_status_s _battery_status;

    struct vehicle_local_position_s _local_position;


    orb_advert_t led_control_pub = nullptr;
    orb_advert_t light_control_component_pub = nullptr;
    orb_advert_t _swarm_link_fc_statue_send_sub = nullptr;


    struct led_control_s                               _led_control;
    struct rst_swarm_link_light_control_component_s    _light_control_component;
    struct rst_swarm_link_fc_statue_send_s             _fc_statue_send;

private:

    bool	_task_should_exit;
    int		_swarm_link_task;


    static void task_main_trampoline(int argc, char *argv[]);

    void task_main();

};