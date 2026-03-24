#include "arm_rc_command.h"

arm_rc_command_t::arm_rc_command_t(){}

arm_rc_command_t::~arm_rc_command_t(){}

void arm_rc_command_t::bind_dr16(pyro::rc_drv_t* dr16_drv)
{
    this->dr16_drv = dr16_drv;
}


//记录一下上一次的开关，按键状态

pyro::dr16_drv_t::sw_state_t last_sw_r_state = pyro::dr16_drv_t::sw_state_t::SW_UP;
pyro::dr16_drv_t::sw_state_t last_sw_l_state = pyro::dr16_drv_t::sw_state_t::SW_UP;

uint8_t last_z_state = 0;

uint8_t last_x_state = 0;
uint8_t last_c_state = 0;
uint8_t last_v_state = 0;

uint8_t last_g_state = 0;

uint8_t last_r_state = 0;
uint8_t last_b_state = 0;
uint8_t last_f_state = 0;
int aaa = 0;
pyro::dr16_drv_t::sw_state_t rec= pyro::dr16_drv_t::sw_state_t::SW_DOWN;

void arm_rc_command_t::update(specific_control_mode_t& specific_control_mode,
            transition_state_t& transition_state,user_command_t& user_command)
{
    // static
    //如果不在线，不进行更新 
    if(!dr16_drv->check_online())
    return;

    _rc_data = static_cast<const pyro::dr16_drv_t::dr16_ctrl_t *>(dr16_drv->read());

    // _rc_data->key.

    //如果开关处于不正常的状态，不进行更新
    if(!(_rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_UP ||
    _rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_MID ||
    _rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_DOWN))
    return;

    


    //右开关从其他位置拨到上
    if(_rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_UP && last_sw_r_state != pyro::dr16_drv_t::sw_state_t::SW_UP)
    {
        specific_control_mode = RESET_POSE_TRANSITION;
        transition_state = Transition_start;
    }
    //右开关从其他位置拨到中
    else if(_rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_MID && last_sw_r_state != pyro::dr16_drv_t::sw_state_t::SW_MID)
    {
        specific_control_mode = NORMAL_POSE_TRANSITION;
        transition_state = Transition_start;
    }
    //右开关从其他位置拨到下
    else if(_rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_DOWN && last_sw_r_state != pyro::dr16_drv_t::sw_state_t::SW_DOWN)
    {
        specific_control_mode = SELF_CONTROL_TRANSITION;
        transition_state = Transition_start;
        // specific_control_mode = MOTION_Start;
        // user_command.selected_motion = arm_motion_e::arm_pop_energy_unit;
        // user_command.selected_motion = arm_motion_e::arm_grip_energy_unit_180;

        // user_command.magazine_target_pos = user_command.magazine_target_pos + pyro::PI/4;
        // if(user_command.magazine_target_pos>pyro::PI)
        // {
        //     user_command.magazine_target_pos = user_command.magazine_target_pos - 2*pyro::PI;
        // }
        // else if(user_command.magazine_target_pos<-pyro::PI)
        // {
        //     user_command.magazine_target_pos = user_command.magazine_target_pos + 2*pyro::PI;
        // }
    }

    //左开关从其他位置拨到中(当当前状态为正常姿态)
    if(_rc_data->rc.s_l.state == pyro::dr16_drv_t::sw_state_t::SW_MID && last_sw_l_state != pyro::dr16_drv_t::sw_state_t::SW_MID&&(specific_control_mode == NORMAL_POSE_TRANSITION || specific_control_mode == NORMAL_POSE))
    {
        specific_control_mode = GRAVITY_COMPENSATION;
        
    }
    //左开关从其他位置拨到中(当当前状态为进行动作状态)
    else if(_rc_data->rc.s_l.state == pyro::dr16_drv_t::sw_state_t::SW_MID && last_sw_l_state != pyro::dr16_drv_t::sw_state_t::SW_MID&&(specific_control_mode == MOTION ))
    {
        specific_control_mode = MOTION_PAUSE;
    }
    //左开关从其他位置拨到下(当当前状态为进行动作暂停状态)
    else if(_rc_data->rc.s_l.state == pyro::dr16_drv_t::sw_state_t::SW_DOWN && last_sw_l_state != pyro::dr16_drv_t::sw_state_t::SW_DOWN&&(specific_control_mode == MOTION_PAUSE ))
    {
        specific_control_mode = MOTION_CONTINUE;
    }
    //左开关从其他位置拨到上(当当前状态为进行动作暂停状态)
    else if(_rc_data->rc.s_l.state == pyro::dr16_drv_t::sw_state_t::SW_UP && last_sw_l_state != pyro::dr16_drv_t::sw_state_t::SW_UP&&(specific_control_mode == MOTION_PAUSE ))
    {
        specific_control_mode = GRAVITY_COMPENSATION;
        
    }

    //按下z键
    if(_rc_data->key.z.state && _rc_data->key.z.state!=last_z_state && !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        specific_control_mode = MOTION_Start;
        user_command.selected_motion = arm_motion_e::arm_grip_energy_unit_0;
    }

    //按下x键
    if(!_rc_data->key.x.state&&_rc_data->key.x.state!=last_x_state && !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        specific_control_mode = MOTION_Start;
        user_command.selected_motion = arm_motion_e::arm_grip_energy_unit_60;
    }

    //按下c键
    if(!_rc_data->key.c.state&&_rc_data->key.c.state!=last_c_state && !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        specific_control_mode = MOTION_Start;
        user_command.selected_motion = arm_motion_e::arm_grip_energy_unit_120;
    }

    //按下v键
    if(!_rc_data->key.v.state&&_rc_data->key.v.state!=last_v_state && !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        specific_control_mode = MOTION_Start;
        user_command.selected_motion = arm_motion_e::arm_grip_energy_unit_180;
    }

    //按下g键
    if(!_rc_data->key.g.state&&_rc_data->key.g.state!=last_g_state && !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        user_command.hold_gripper = !user_command.hold_gripper;
    }

    //按下r键
    if(!_rc_data->key.r.state&&_rc_data->key.r.state!=last_r_state && !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        
        user_command.magazine_target_pos = user_command.magazine_target_pos + pyro::PI/2;
        if(user_command.magazine_target_pos>pyro::PI)
        {
            user_command.magazine_target_pos = user_command.magazine_target_pos - 2*pyro::PI;
        }
        else if(user_command.magazine_target_pos<-pyro::PI)
        {
            user_command.magazine_target_pos = user_command.magazine_target_pos + 2*pyro::PI;
        }
    }

    //按下b键
    if(!_rc_data->key.b.state&&_rc_data->key.b.state!=last_b_state && !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        specific_control_mode = MOTION_Start;
        user_command.selected_motion = arm_motion_e::arm_push_energy_unit;
    }

    //按下f键
    if(!_rc_data->key.f.state&&_rc_data->key.f.state!=last_f_state && !(specific_control_mode == RESET_POSE || specific_control_mode == RESET_POSE_TRANSITION))
    {
        specific_control_mode = MOTION_Start;
        user_command.selected_motion = arm_motion_e::arm_pop_energy_unit;
    }

    user_command.gripper_increment = _rc_data->rc.wheel*0.005;
    
    //记录上一次的开关,按键状态
    last_sw_r_state = _rc_data->rc.s_r.state;
    last_sw_l_state = _rc_data->rc.s_l.state; 
    last_z_state = _rc_data->key.z.state;
    last_x_state = _rc_data->key.x.state;
    last_c_state = _rc_data->key.c.state;
    last_v_state = _rc_data->key.v.state;
    last_g_state = _rc_data->key.g.state;
    last_r_state = _rc_data->key.r.state;
    last_b_state = _rc_data->key.b.state;
    last_f_state = _rc_data->key.f.state;
}