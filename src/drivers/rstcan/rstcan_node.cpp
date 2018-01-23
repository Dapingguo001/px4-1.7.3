/*********************************************************************************
 *Copyright(C),2015-2017, Robsense Tech. All rights reserved.
 *FileName:    rstcan_node.cpp
 *Author:      HeBin
 *Version:     0.1
 *Date:        2017-11-10 10:29:06
 *Last Modify: 2017-11-10 10:29:06
 *Description: 
**********************************************************************************/

#include "rstcan_manager.h"
#include "rstcan_node.h"

static RSTCan_Node *instance = nullptr;

RSTCan_Node::RSTCan_Node(const char *name, const char *path, void *_master_node, void *_slave_node):
    CDev(name, path),
    master_node(_master_node),
    _call{},
    _task_fd(0),
    slave_node(_slave_node)
{

}

RSTCan_Node::~RSTCan_Node()
{
    if(slave_node != nullptr)
        rstcan_free_node(slave_node);
}

int	RSTCan_Node::init()
{
    int ret;

    printf("node %s init\n", _name);

	ret = CDev::init();

    instance = this;

    return ret;
}

int	RSTCan_Node::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    return 0;
}

ssize_t	RSTCan_Node::write(struct file *filp, const char *buffer, size_t len)
{

    return 0;
}

ssize_t RSTCan_Node::read(struct file *filp, char *buffer, size_t buflen)
{

    return 0;
}

void RSTCan_Node::poll_notify(pollevent_t events)
{
    //调用父类CDev中的poll_notify
    CDev::poll_notify(events);
}

void RSTCan_Node::node_task_start(int32_t interval, void *arg)
{
//    _call.period = interval;
//
//	hrt_call_every(&_call,
//                       1000,
//                       interval,
//                       RSTCan_Node::_task_trampoline, this);

    _task_fd = px4_task_spawn_cmd(_name,
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   5900,
					   (px4_main_t)&RSTCan_Node::_task_trampoline,
					   nullptr);


}

void RSTCan_Node::node_task_stop()
{
//    hrt_cancel(&_call);
}
 
void RSTCan_Node::_task_trampoline(int argc, char *argv[])
{
//    RSTCan_Node *_node = static_cast<RSTCan_Node *>(arg);

//    _node->_task();    
    instance->_task();

    return;
}

int32_t RSTCan_Node::_task()
{

    return 0;
}

void RSTCan_Node::print_data()
{
}

void RSTCan_Node::test()
{

}
