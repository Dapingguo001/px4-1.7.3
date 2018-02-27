/*********************************************************************************
 *Copyright(C),2015-2017, Robsense Tech. All rights reserved.
 *FileName:    drv_rstcan.h
 *Author:      HeBin
 *Version:     0.1
 *Date:        2017-11-21 19:53:24
 *Last Modify: 2017-11-21 19:53:24
 *Description: 
**********************************************************************************/

#pragma once

#include <stdint.h>
#include <sys/ioctl.h>

#define RSTCAN_UID_SIZE             (8)  //设备ID号一共8个字节

#define RSTCAN_MANAGER_DEV_BASE_PATH   "/dev/rstcan"
#define RSTCAN_MANAGER_DEV0            "/dev/rstcan0"
#define RSTCAN_MANAGER_DEV1            "/dev/rstcan1"


#define RSTCAN_IOC_BASE		0x0300
#define RSTCAN_IOC_GET_4G_BOOT      _IOC(RSTCAN_IOC_BASE, 0)
#define RSTCAN_IOC_SET_4G_SERVER    _IOC(RSTCAN_IOC_BASE, 1)

#define RSTCAN_IOC_SLAVE_REBOOT     _IOC(RSTCAN_IOC_BASE, 2)
#define RSTCAN_IOC_GET_UID          _IOC(RSTCAN_IOC_BASE, 3)

//作为struct rstcan_camera中的data部分
//XXX 结构体长度不可超过camera_can_payload_t中的data数组
//XXX 必须和dev manager中的ptz_ctrl_t相同！！
#define RST_PTZ_CTRL_MIDDLE_MODE      (1)
#define RST_PTZ_CTRL_VERTICAL_MODE    (2)
#define RST_PTZ_CTRL_HEAD_LOCK_MODE   (3)
#define RST_PTZ_CTRL_HEAD_FOLLOW_MODE (4)
#pragma pack(push, 1)
typedef struct ptz_ctrl {
    uint8_t mode;
    float roll;
    float pitch;
    float yaw;
}ptz_ctrl_t;
#pragma pack(pop)


//XXX 必须和dev manager中的相同！！
#define DATA_MASK_DISTANCE  (1)
#define DATA_MASK_SPEED     (1<<1)

#pragma pack(push, 1)
typedef struct radar_data {
    uint32_t data_mask;   //告知使用者该雷达有哪些有效数据
    uint16_t distance;    // mm
    uint32_t speed;       // cm/s
}radar_data_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct battery_info{
    uint16_t batt_num;       //电池组中有多少个电池
    uint32_t capacity;       //电池容量，单位mAh
    uint32_t total_vol;      //电池组总电压, 单位mV
    uint32_t curr;           //充放电电流，正数充电，负数放电, 单位mA
    uint16_t temp;           //电池温度，单位0.1度
    uint8_t  charge_stat;    //充电状态，单位%
    uint16_t cycle_count;    //电池循环计数
    uint8_t  healthy;        //健康状况，单位%
}batt_info_t;
#pragma pack(pop)



