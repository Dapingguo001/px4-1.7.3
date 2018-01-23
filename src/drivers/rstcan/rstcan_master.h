/*********************************************************************************
 *Copyright(C),2015-2017, Robsense Tech. All rights reserved.
 *FileName:    rstcan_master.h
 *Author:      HeBin
 *Version:     0.1
 *Date:        2017-10-13 14:05:18
 *Last Modify: 2017-10-13 14:05:18
 *Description: 
**********************************************************************************/
#pragma once
#include "rstcan.h"
#ifdef __cplusplus
extern "C"{
#endif

/****************************************************************************
 * Description:
 *      master节点的CAN ID分配函数(非阻塞),
 *      分配完毕的几点可以在rstcan_get_new_node中获取
 * Input parameters:
 *      master_node - master节点的虚拟节点
 * Returned Value:
 *      0  - 正常运行
 *      -1 - 出现错误
 ****************************************************************************/
int32_t rstcan_alloc_id_service(void *master_node);

/****************************************************************************
 * Description:
 *      停止CAN ID分配
 * Input parameters:
 *      -
 * Returned Value:
 *      0  - 正常运行
 ****************************************************************************/
int32_t rstcan_stop_alloc_id_service();

/****************************************************************************
 * Description:
 *      获取分配好ID的虚拟节点指针
 * Input parameters:
 *      new_node - 虚拟节点都是void指针,获取新虚拟节点需要二级指针
 * Returned Value:
 *      0  - 成功获取
 *      -1 - 无新节点
 ****************************************************************************/
int32_t rstcan_get_new_node(void **new_node);

#ifdef __cplusplus
}
#endif
