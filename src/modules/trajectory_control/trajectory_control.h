
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>

class TrajectoryControl{
public:
    TrajectoryControl();
    ~TrajectoryControl();

    static void task_main_trampoline(int argc, char *argv[]);
    int             start();

private:
    
    void            task_main();


    int             _trajectory_control_task;
    bool	        _task_should_exit;

    orb_advert_t    _vehicle_command_pub;
    int             _control_mode_sub;
    int             _sp_man_sub;

    bool            _start_circle_control = false;
    bool            _circle_control_command_start_success = false;

    vehicle_command_s  _vehicle_command;
    manual_control_setpoint_s _sp_man;

    vehicle_control_mode_s  _control_mode;

    void send_vehicle_command(bool is_offboard_mode);


};