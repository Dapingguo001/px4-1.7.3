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

#define RSTCAN_MANAGER_DEV_BASE_PATH   "/dev/rstcan"
#define RSTCAN_MANAGER_DEV0            "/dev/rstcan0"
#define RSTCAN_MANAGER_DEV1            "/dev/rstcan1"


#define RSTCAN_IOC_BASE		0x0300
#define RSTCAN_IOC_GET_4G_BOOT      _IOC(RSTCAN_IOC_BASE, 0)
#define RSTCAN_IOC_SET_4G_SERVER    _IOC(RSTCAN_IOC_BASE, 1)

#define RSTCAN_IOC_SLAVE_REBOOT     _IOC(RSTCAN_IOC_BASE, 2)

//作为struct rstcan_camera中的data部分
//XXX 结构体长度不可超过camera_can_payload_t中的data数组
//XXX 必须和dev manager中的ptz_ctrl_t相同！！
typedef struct ptz_ctrl {
    float roll;
    float pitch;
    float yaw;
}ptz_ctrl_t;



