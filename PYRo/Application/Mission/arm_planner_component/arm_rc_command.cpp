#include "arm_rc_command.h"

arm_rc_command_t::arm_rc_command_t(){}

arm_rc_command_t::~arm_rc_command_t(){}

void arm_rc_command_t::bind_dr16(pyro::rc_drv_t* dr16_drv)
{
    this->dr16_drv = dr16_drv;
}

pyro::dr16_drv_t::sw_state_t last_sw_r_state = pyro::dr16_drv_t::sw_state_t::SW_UP;

// pyro::dr16_drv_t::key_t

int aaa = 0;
pyro::dr16_drv_t::sw_state_t rec= pyro::dr16_drv_t::sw_state_t::SW_DOWN;

void arm_rc_command_t::update(specific_control_mode_t& specific_control_mode,
            transition_state_t& transition_state,user_command_t& user_command)
{
    // static 
    if(!dr16_drv->check_online())
    return;

    _rc_data = static_cast<const pyro::dr16_drv_t::dr16_ctrl_t *>(dr16_drv->read());

    // _rc_data->key.

    if(!(_rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_UP ||
    _rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_MID ||
    _rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_DOWN))
    return;

     if(_rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_UP)
     aaa|=1;
     else
     aaa&=~1;
     if(last_sw_r_state != pyro::dr16_drv_t::sw_state_t::SW_UP)
     {aaa|=2;
        rec = last_sw_r_state;
     }
     else
     aaa&=~2;

    if(_rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_UP && last_sw_r_state != pyro::dr16_drv_t::sw_state_t::SW_UP)
    {
        specific_control_mode = RESET_POSE_TRANSITION;
        transition_state = Transition_start;
    }
    else if(_rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_MID && last_sw_r_state != pyro::dr16_drv_t::sw_state_t::SW_MID)
    {
        specific_control_mode = NORMAL_POSE_TRANSITION;
        transition_state = Transition_start;
    }
    else if(_rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_DOWN && last_sw_r_state != pyro::dr16_drv_t::sw_state_t::SW_DOWN)
    {
        specific_control_mode = SELF_CONTROL_TRANSITION;
        transition_state = Transition_start;
    }
    
    last_sw_r_state = _rc_data->rc.s_r.state; 
}