#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>


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
    orb_advert_t    _offboard_control_mode_pub;
    orb_advert_t    _pos_set_tri_pub;

    int             _control_mode_sub;
    int             _sp_man_sub;
    int             _gps_pos_sub;
    int             _global_pos_sub;

    bool            _start_circle_control = false;
    bool            _circle_control_command_start_success = false;
    bool            _gps_valid = false;

    uint64_t        _last_time_stamp;
    float           _last_angle_vel;
    float           _cos_angle_dec;

    vehicle_command_s           _vehicle_command;
    offboard_control_mode_s     _offboard_control_mode;

    manual_control_setpoint_s   _sp_man;
    vehicle_control_mode_s      _control_mode;
    vehicle_gps_position_s      _gps_pos;
    vehicle_global_position_s   _global_pos;
    position_setpoint_triplet_s _pos_set_tri;

    struct circle_trajectory_command_bag{
        //用户输入参数
        float center_of_circle_lat;       //圆心所在的维度
        float center_of_circle_lon;       //圆心所在的经度
        float center_of_circle_hgt;       //圆心所在的高度
        float circle_dip_angle_north;     //圆环北向倾斜角度
        float circle_dip_angle_est;       //圆形东向倾斜角度
        float end_angle_vel;              //终止速度
        float set_angle_sum;              //运行总角度  
        float set_angle_vel;              //设定运行角速度
        uint8_t direction_of_rotation;    //运行方向 1:顺时针旋转, 0:逆时针旋转

        //初始化圆环参数
        float circle_R;
        float circle_R_n;
        float circle_R_e;
        float initial_theta;
        float max_angle_acc;
        float circle_3d_tranfer_matrix[3][3]; 

        //任务运行状态
        bool first_carry_out_task;        //true:第一次运行程序， false:非第一次运行程序
        bool task_complete;               //true:完成， false:未完成
        float start_angle_vel;            //起始角速度，任务初始化时，获取飞行器当前角速度
        float angle_vel;                  //当前时刻角速度
        float angle_sum_accomplish;
    };

    circle_trajectory_command_bag   _circle_trajectory_command_bag;

    void send_vehicle_command(bool is_offboard_mode);

    void send_offboard_control_mode();

    void            do_circle(circle_trajectory_command_bag   cir_tra_com_bag);
    void            do_line();
    void            circle_init(circle_trajectory_command_bag   cir_tra_com_bag);
    void            tragectory_publish(circle_trajectory_command_bag   cir_tra_com_bag);
};