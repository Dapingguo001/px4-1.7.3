/*********************************************************************************
 *Copyright(C),2015-2017, Robsense Tech. All rights reserved.
 *FileName:    rstcan.h
 *Author:      HeBin
 *Version:     0.1
 *Date:        2017-09-25 19:12:04
 *Last Modify: 2017-09-25 19:12:04
 *Description: 
**********************************************************************************/

/*
 * API用法
 * master:
 * rstcan_node_init -> thread1: rstcan_alloc_id_service
 *                  -> thread2: rstcan_get_new_node/rstcan_register_msg
 *
 * slave:
 * rstcan_node_init -> rstcan_alloc_id/rstcan_register_msg  -> tx: rstcan_send_msg rx: rstcan_recv_msg isr:rstcan_put_frame
 */

#pragma once
#include <inttypes.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

/*
 * 同类型节点最多8个
 */
#define RSTCAN_MAX_INDEX    (8)

/*
 * 节点类型最多127个
 */
#define RSTCAN_MAX_NODE_TYPE    (127)

/*
 * 消息类型
 */
typedef enum{
    RSTCAN_MSG_TYPE_ANON,   //匿名消息
    RSTCAN_MSG_TYPE_NOR,    //普通消息
    RSTCAN_MSG_TYPE_BST,    //广播消息
    RSTCAN_MSG_TYPE_SVC,    //服务消息
    RSTCAN_MSG_TYPE_NR,
}RSTCAN_MSG_TYPE_E;

/*
 * 用户自定义的所有消息类型都必须从10开始
 * 10之前的子类型号留给rstcan内部使用
 */
#define CUSTOM_SUB_MSG_BASE      (10)

#define _MSG_CONSTRUCTOR(msg, s, d, l)\
        msg.sub_type = s;\
        msg.data = d;\
        msg.len = l;\

/****************************************************************************
 * Description:
 *      构建广播消息的消息结构体
 * Input parameters:
 *      msg - rstcan_msg_t的结构体变量
 *      s   - 消息子类型
 *      d   - 要发送的数据
 *      l   - 数据长度
 ****************************************************************************/
#define RSTCAN_BST_MSG_CONSTRUCTOR(msg, s, d, l)\
        msg.msg_type = RSTCAN_MSG_TYPE_BST;\
        _MSG_CONSTRUCTOR(msg, s, d, l)

/****************************************************************************
 * Description:
 *      构建普通消息的消息结构体
 * Input parameters:
 *      msg - rstcan_msg_t的结构体变量
 *      s   - 消息子类型
 *      d   - 要发送的数据
 *      l   - 数据长度
 ****************************************************************************/
#define RSTCAN_NOR_MSG_CONSTRUCTOR(msg, s, d, l)\
        msg.msg_type = RSTCAN_MSG_TYPE_NOR;\
        _MSG_CONSTRUCTOR(msg, s, d, l)

/****************************************************************************
 * Description:
 *      构建匿名消息的消息结构体
 * Input parameters:
 *      msg - rstcan_msg_t的结构体变量
 *      s   - 消息子类型
 *      d   - 要发送的数据
 *      l   - 数据长度
 ****************************************************************************/
#define RSTCAN_ANON_MSG_CONSTRUCTOR(msg, s, d, l)\
        msg.msg_type = RSTCAN_MSG_TYPE_ANON;\
        _MSG_CONSTRUCTOR(msg, s, d, l)

/****************************************************************************
 * Description:
 *      构建服务消息的消息结构体
 * Input parameters:
 *      msg - rstcan_msg_t的结构体变量
 *      s   - 消息子类型
 *      d   - 要发送的数据
 *      l   - 数据长度
 ****************************************************************************/
#define RSTCAN_SVC_MSG_CONSTRUCTOR(msg, s, d, l)\
        msg.msg_type = RSTCAN_MSG_TYPE_SVC;\
        _MSG_CONSTRUCTOR(msg, s, d, l)

#define RSTCAN_FRAME_MAX_DATA   (8)
typedef struct rstcan_frame{
    uint32_t    can_id; //CAN帧中的标准id或者扩展id, rstcan中统一用扩展id
    uint8_t     ide;    //扩展id标志位
    uint8_t     rtr;    //远程帧标志位
    uint8_t     dlc;    //数据长度
    uint8_t     data[RSTCAN_FRAME_MAX_DATA];//数据
}rstcan_frame_t;
#define RSTCAN_FRAME_SIZE   (sizeof(rstcan_frame_t))

#define RSTCAN_PAYLOAD_BYTES        (RSTCAN_FRAME_MAX_DATA - 1)

/*
 * can口的操作接口集合, 由使用者提供
 */
struct rstcan_ops{
    int32_t (*can_send_frame)(void *fd, rstcan_frame_t *frame);//发送一帧can帧
    //TODO id分配完毕后, 根据节点情况设置can滤波器
    int32_t (*can_set_filter)(void *fd, uint32_t filter_mask);
    int32_t (*can_get_send_space)(void *fd);
    int32_t (*usleep)(uint32_t usec);//考虑到裸机程序可能没有usleep导致编译不过,所以延时函数也由使用者提供
};

/*
 * msg的数据最大64帧的长度
 */
#define RSTCAN_MSG_DATA_MAX_LEN     (63 * (RSTCAN_FRAME_MAX_DATA - 1))
/*
 * rstcan 消息结构体
 */
typedef struct rstcan_msg{
    RSTCAN_MSG_TYPE_E msg_type;
    uint32_t sub_type;  //服务类型, 广播类型, 普通消息类型或者匿名消息中的uid
    uint8_t *data;
    uint32_t len;
}rstcan_msg_t;

#ifdef __cplusplus
extern "C"{
#endif
/****************************************************************************
 * Description:
 *      初始化对应当前物理节点的虚拟节点和can口
 * Input parameters:
 *      node_type - 当前物理节点的节点类型
 *      name      - 节点名字
 *      ops       - can口的操作接口集合, 由使用者提供
 *      can_fd    - can操作接口句柄
 *      is_master - 是否是master节点
 *      max_type  - 该can通信系统中节点类型最大值
 * Returned Value:
 *      not NULL  - 虚拟节点指针
 *      NULL -  初始化失败
 ****************************************************************************/
void *rstcan_node_init(uint8_t node_type, char *name, struct rstcan_ops *ops, void *can_fd, bool is_master, uint8_t max_type);

/****************************************************************************
 * Description:
 *      销毁一个节点对象
 * Input parameters:
 *      node - 待销毁的节点对象
 * Returned Value:
 *      0  - 成功
 *      -1 - 失败
 ****************************************************************************/
int32_t rstcan_free_node(void *node);

/****************************************************************************
 * Description:
 *      注册一种类型的rstcan消息,并初始化ringbuffer中放多少个frame, 
 *      一个节点必须在注册以后才能收到对应类型的消息
 *      除了匿名消息,匿名消息只有master可以收
 * Input parameters:
 *      from_node - 数据发送方的can节点对象
 *      msg_type  - 注册的消息类型
 *      num       - buffer大小, 表示这个buffer可以存多少帧can帧(rstcan_frame)
 *      stream    - 是否是数据流模式,数据流模式直接存can数据,不存can帧
 * Returned Value:
 *      0  - 成功
 *      -1 - 失败
 ****************************************************************************/
int32_t rstcan_register_msg(void *from_node, RSTCAN_MSG_TYPE_E msg_type, uint16_t num, bool stream);

/****************************************************************************
 * Description:
 *      把收到的一帧can帧放入对应节点的buffer中
 * Input parameters:
 *      this_node - 当前物理节点的节点对象
 *      frame     - can帧结构体指针
 * Returned Value:
 *      0  - 成功
 *      -1 - 失败
 ****************************************************************************/
int32_t rstcan_put_frame(void *this_node, rstcan_frame_t *frame);

/****************************************************************************
 * Description:
 *      发送一则rstcan消息给tar_node
 * Input parameters:
 *      this_node - 当前物理节点的节点对象
 *      tar_node  - 目标节点对象
 *      msg       - 消息包
 * Returned Value:
 *      0  - 发送成功
 *      -1 - 发送失败
 ****************************************************************************/
int32_t rstcan_send_msg(void *this_node, void *tar_node, rstcan_msg_t *msg);

#define RSTCAN_RECV_SUCCESS    (0)      //成功收完一则消息
#define RSTCAN_RECV_GOING      (1)      //还在组包中
#define RSTCAN_RECV_ERR        (-1)     //发生错误
#define RSTCAN_RECV_NO_DATA    (-2)      //buffer中没有数据

/****************************************************************************
 * Description:
 *      接收一则rstcan消息, 用户必须保证里面的buffer足够大
 * Input parameters:
 *      from_node - 数据发送方的can节点对象
 *      msg       - 用户提供的消息结构体, 存放组包后的数据
 * Returned Value:
 *      RSTCAN_RECV_SUCCESS  - 一个消息组包成功, 用户可以拿去处理
 *      RSTCAN_RECV_ERR      - 发生错误
 *      RSTCAN_RECV_GOING    - 组包还未完成, 通知用户继续传递同一个msg
 *      RSTCAN_RECV_NO_DATA  - buffer中无数据, 需等待
 ****************************************************************************/
int32_t rstcan_recv_msg(void *from_node, rstcan_msg_t *msg);

/****************************************************************************
 * Description:
 *      获取节点的类型
 * Input parameters:
 *      node - 想要获取类型的那个节点的对象
 * Returned Value:
 *      >0 - 节点类型
 *      -1 - 获取失败
 ****************************************************************************/
int32_t rstcan_get_node_type(void *node);

/****************************************************************************
 * Description:
 *      获取节点的名字
 * Input parameters:
 *      node - 想要获取名字的那个节点的对象
 *      name - 二级指针,获取节点名的指针
 * Returned Value:
 *      0  - 获取成功
 *      -1 - 节点无名字,获取失败
 ****************************************************************************/
int32_t rstcan_get_node_name(void *node, char **name);

/****************************************************************************
 * Description:
 *      获取节点的索引号
 * Input parameters:
 *      node - 想要获取索引号的那个节点的对象
 * Returned Value:
 *      >=0 - 节点索引号
 *      -1  - 节点无名字,获取失败
 ****************************************************************************/
int32_t rstcan_get_node_idx(void *node);

/****************************************************************************
 * Description:
 *      获取虚拟节点中的消息类型对应的有效数据长度
 * Input parameters:
 *      node - 想要获取索引号的那个节点的对象
 *      msg_type - 消息类型
 * Returned Value:
 *      >=0 - buffer中有效数据长度
 *      -1  - 节点无名字,获取失败
 ****************************************************************************/
int32_t rstcan_get_data_len(void *node, uint8_t msg_type);

/****************************************************************************
 * Description:
 *      获取虚拟节点中的消息类型对应的缓冲区空余长度
 * Input parameters:
 *      node - 想要获取索引号的那个节点的对象
 *      msg_type - 消息类型
 * Returned Value:
 *      >=0 - buffer中空余空间长度
 *      -1  - 节点无名字,获取失败
 ****************************************************************************/
int32_t rstcan_get_data_space(void *node, uint8_t msg_type);

/****************************************************************************
 * Description:
 *      获取can驱动中发送缓冲区的长度,如果用户驱动没提供接口则返回0
 * Input parameters:
 *      node - 想要获取发送缓冲区长度的那个节点的对象
 * Returned Value:
 *      >0 - can驱动中发送缓冲区空余长度
 *      0  - 节点无名字,获取失败
 ****************************************************************************/
int32_t rstcan_get_send_space(void *node);

/****************************************************************************
 * Description:
 *      获取rstcan的git版本
 * Input parameters:
 * Returned Value:
 *      >0  - git版本号
 *      <=0 - 获取失败
 ****************************************************************************/
uint64_t rstcan_get_version();

/****************************************************************************
 * Description:
 *      获取这帧can消息是哪个节点发来的
 *      注意：匿名消息和广播消息没有目标节点，返回失败！！
 * Input parameters:
 *      frame    - can数据帧
 *      src_type - 获取到的目标节点类型
 *      src_idx  - 获取到的目标节点索引号
 * Returned Value:
 *      0  - 获取成功
 *      <0 - 获取失败
 ****************************************************************************/
int32_t rstcan_get_msg_source(rstcan_frame_t *frame, uint8_t *src_type, uint8_t *src_idx);

#ifdef __cplusplus
}
#endif
