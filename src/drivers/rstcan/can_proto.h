/*********************************************************************************
 *Copyright(C),2015-2017, Robsense Tech. All rights reserved.
 *FileName:    can_proto.h
 *Author:      HeBin
 *Version:     0.1
 *Date:        2017-11-13 14:13:20
 *Last Modify: 2017-11-13 14:13:20
 *Description: 
**********************************************************************************/

/*
 * XXX 此头文件,飞控端和CAN节点端必须一样!!!
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "rstcan.h"

/*
 * 节点类型
 */
typedef enum{
    RSTCAN_NODE_NONE = 0,
    //for bootloader
    RSTCAN_NODE_BL_USB,    //bootloader阶段usb节点用这个
    RSTCAN_NODE_SLAVE_105, //所有接在can口上的子节点，bootloader升级程序是用作广播id
    RSTCAN_NODE_SLAVE_103, //所有接在can口上的子节点，bootloader升级程序是用作广播id

    //for app, 加节点类型需慎重, 和can优先级密切相关, 根据优先级来确定枚举的位置, 不可随便加
    RSTCAN_NODE_4G_CONTROL ,
    RSTCAN_NODE_USB,
    RSTCAN_NODE_4G_TRANS,
    RSTCAN_NODE_RGBLED,
    RSTCAN_NODE_CAMERA,
    RSTCAN_NODE_GPS,
    RSTCAN_NODE_BARO,
    RSTCAN_NODE_MAG,
    RSTCAN_NODE_GYRO,
    RSTCAN_NODE_ACCEL,  //13
    RSTCAN_NODE_MASTER, //14
    RSTCAN_NODE_NR,

}RSTCAN_NODE_TYPE_E;

//所有广播命令统一放在这个枚举中
enum BST_SUB_MSG{
    //bootloader
    BL_BST_ERASE_FLASH_SUB_MSG = CUSTOM_SUB_MSG_BASE,   //flash擦除命令
    BL_BST_PROG_MULTI_SUB_MSG,                          //数据烧写命令
    BL_BST_GET_CRC_SUB_MSG,                             //获取crc校验值命令
    BL_BST_START_UPLOAD_SUB_MSG,                        //从节点开始升级命令
    BL_BST_NODE_BOOT_SUB_MSG,                           //从节点启动app命令
    BL_BST_PROG_SUM_SUB_MSG, 

    //app
    BST_SLAVE_REBOOT_SUB_MSG,                           //从节点重启命令

    BST_SUB_MSG_NR
};

//bootloader中所有从节点通用的普通消息
enum BL_NOR_SUB_MSG{
    BL_NOR_SLV_ACK_SUB_MSG = CUSTOM_SUB_MSG_BASE,   //从节点回复消息
    BL_NOR_SUB_MSG_NR
};

/*
 * bootloader中USB节点的服务消息
 */
enum USB_NODE_SVC_SUB_MSG{
    BL_SVC_USB_BYPASS_SUB_MSG = CUSTOM_SUB_MSG_BASE,    //数据透传服务
    BL_SVC_SUB_MSG_NR
};

/*
 * app中所有透传节点(USB,4G,GPS等)的服务消息
 */
enum BYPASS_NODES_SVC_SUB_MSG{
    BYPASS_NODES_DATA_SUB_MSG = CUSTOM_SUB_MSG_BASE,//数据透传服务
    BYPASS_NODES_SVC_NR
};

/*
 * 4G模块控制节点
 */
enum RADIO_4G_NOR_SUB_MSG{
    RADIO_4G_GET_STATUS_SUB_MSG = CUSTOM_SUB_MSG_BASE, //4g模块can命令, 获取4G模块状态,目前只有是否启动, 后续可以继续加 
    RADIO_4G_SET_SERVER_SUB_MSG, //4g模块can命令, 设置服务器地址和端口
    RADIO_4G_ACK_SUB_MSG,        //4g模块can命令, 每设置一次,节点的回复,或者查询的内容, 设置成功返回"OK", 否则返回"ERR"
    RADIO_4G_SUB_MSG_NR
};

/*
 * RGB LED节点控制命令
 */
enum RGBLED_NOR_SUB_MSG{
    RGBLED_SET_RGB_SUB_MSG = CUSTOM_SUB_MSG_BASE,    //设置rgb的值
    RGBLED_SET_COLOR_SUB_MSG,                        //设置颜色
    RGBLED_SET_MODE_SUB_MSG,                         //设置亮灯模式
    RGBLED_SET_PATTERN_SUB_MSG,
    RGBLED_SUB_MSG_NR
};

#define TRANS_LEN(p)    (sizeof(uint32_t) + p)
//传输的时候data不一定时64，有多少传多少
#pragma pack(push, 1)
typedef struct camera_can_payload {
    uint32_t data_len;
    uint8_t data[64];
}camera_can_payload_t;
#pragma pack(pop)

/*
 * Device manager节点控制命令 
 */
enum CAMERA_NOR_SUB_MSG {
    CAMERA_ZOOM_IN_SUB_MSG = CUSTOM_SUB_MSG_BASE,  //放大
    CAMERA_ZOOM_OUT_SUB_MSG,                       //缩小
    CAMERA_ZOOM_STOP_SUB_MSG,                      //停止缩放
    CAMERA_SNAPSHOT_SUB_MSG,
    CAMERA_RECORD_START_SUB_MSG,
    CAMERA_RECORD_STOP_SUB_MSG,
    CAMERA_PTZ_CTRL_SUB_MSG,
    CAMERA_FOCUS_SUB_MSG,
    CAMERA_SUB_MSG_NR,
};


