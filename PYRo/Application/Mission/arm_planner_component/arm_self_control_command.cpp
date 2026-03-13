#include "arm_self_control_command.h"

arm_self_control_command::arm_self_control_command(){}

arm_self_control_command::~arm_self_control_command(){}

void arm_self_control_command::bind(pyro::databoard* databoard)
{
    this->_databoard = databoard;
    _selfcontrol_axis1_id = databoard->get_topic_id("selfcontrol axis1");
    _selfcontrol_axis2_id = databoard->get_topic_id("selfcontrol axis2");
    _selfcontrol_axis3_id = databoard->get_topic_id("selfcontrol axis3");
    _selfcontrol_axis4_id = databoard->get_topic_id("selfcontrol axis4");
    _selfcontrol_axis5_id = databoard->get_topic_id("selfcontrol axis5");
    _selfcontrol_axis6_id = databoard->get_topic_id("selfcontrol axis6");
}

void arm_self_control_command::update(user_command_t& user_command)
{
    uint32_t timestamp;
    _databoard->read(_selfcontrol_axis1_id,(pyro::genenral_data_t*)&(_self_control_command[0]),timestamp);
    _databoard->read(_selfcontrol_axis2_id,(pyro::genenral_data_t*)&(_self_control_command[1]),timestamp);
    _databoard->read(_selfcontrol_axis3_id,(pyro::genenral_data_t*)&(_self_control_command[2]),timestamp);
    _databoard->read(_selfcontrol_axis4_id,(pyro::genenral_data_t*)&(_self_control_command[3]),timestamp);
    _databoard->read(_selfcontrol_axis5_id,(pyro::genenral_data_t*)&(_self_control_command[4]),timestamp);
    _databoard->read(_selfcontrol_axis6_id,(pyro::genenral_data_t*)&(_self_control_command[5]),timestamp);
}

void arm_self_control_command::get_self_control_command(float xdata[6])
{
    memcpy(xdata,_self_control_command,sizeof(float)*6);
}

float * arm_self_control_command::get_self_control_command()
{
    return _self_control_command;
}