#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "pyro_databoard.h"
#include "pyro_rc_hub.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_algo_pid.h"

extern pyro::databoard* global_databoard; 

namespace pyro
{
class wheel_control_t 
{ 
    
   typedef enum 
    {
        FORWARD,
        REVERSE
    }
    wheel_ward_t;
    protected:
        wheel_ward_t _ward;
        pyro::motor_base_t* _motor;
        pyro::pid_t* _rot_pid;

        float _target_rot;

        float _feedback_rot;

        float _control_value;

    public:
    
        wheel_control_t(pyro::motor_base_t* motor,  pyro::pid_t* rot_pid):_motor(motor),_rot_pid(rot_pid)
        {
            _target_rot = 0.0f;
            _ward = FORWARD;
        }
        ~wheel_control_t(){}
        void set_target(float& target) 
        {
            if(_ward == FORWARD)
            {
                _target_rot = target;
            }
            else
            {
                _target_rot = -target;
            }
        }
        void update() 
        {
            _motor->update_feedback();
            _feedback_rot = _motor->get_current_rotate();
        }
        float control(float dt)
        {   
            float control_value = 0.0f;
            control_value = _rot_pid->calculate(_target_rot,_feedback_rot);
            // _motor->send_torque(control_value);
            return control_value;
        }
        void set_forward() 
        {
            _ward = FORWARD;
        }
        void set_reverse() 
        {
            _ward = REVERSE;
        }
};
};

typedef enum
{
    ZERO_FORCE = 0,
    RC_CONTROL
}
chassis_mode_t;
chassis_mode_t chassis_mode = ZERO_FORCE;

static uint32_t rc_sw_l_topic_id,rc_sw_r_topic_id,rc_ch_lx_topic_id,rc_ch_ly_topic_id,rc_ch_rx_topic_id,rc_ch_ry_topic_id;

static uint32_t motor_torque_topic_id,motor_rotate_topic_id;

typedef struct
{
    uint32_t sw_l;
    uint32_t sw_r;
    float ch_lx;
    float ch_ly;
    float ch_rx;
    float ch_ry;
}rc_data_t;

rc_data_t rc_data;

uint32_t sw_l,sw_r;
float ch_lx,ch_ly,ch_rx,ch_ry;
uint32_t timestamp;

pyro::dji_m3508_motor_drv_t *wheel_motor_fl,*wheel_motor_fr,*wheel_motor_br,*wheel_motor_bl;
pyro::pid_t *rot_pid_fl,*rot_pid_fr,*rot_pid_br,*rot_pid_bl;
pyro::wheel_control_t *wheel_control_fl,*wheel_control_fr,*wheel_control_br,*wheel_control_bl;

float wheel_motor_current_position[4];
float wheel_motor_current_rotate[4];
float wheel_motor_current_torque[4];

bool chassis_rc_solve()
{
    pyro::topic::data_status_t rc_data_status;
    rc_data_status = global_databoard->read(rc_sw_l_topic_id,(pyro::genenral_data_t*)&(rc_data.sw_l),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
    {
        return false;
    }
    rc_data_status = global_databoard->read(rc_sw_r_topic_id,(pyro::genenral_data_t*)&(rc_data.sw_r),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
    {
        return false;
    }
    rc_data_status = global_databoard->read(rc_ch_lx_topic_id,(pyro::genenral_data_t*)&(rc_data.ch_lx),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
    {
        return false;
    }
    rc_data_status = global_databoard->read(rc_ch_ly_topic_id,(pyro::genenral_data_t*)&(rc_data.ch_ly),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
    {
        return false;
    }
    rc_data_status = global_databoard->read(rc_ch_rx_topic_id,(pyro::genenral_data_t*)&(rc_data.ch_rx),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
    {
        return false;
    }
    rc_data_status = global_databoard->read(rc_ch_ry_topic_id,(pyro::genenral_data_t*)&(rc_data.ch_ry),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
    {
        return false;
    }
    return true;
}
#include <stdio.h>
#include <math.h>

// 定义常量
#define SQRT2_DIV_2 0.70710678118f  // sqrt(2)/2
#include <math.h>

int mecanum_inverse_kinematics(
    const float target_vel[3],
    float wheel_vel[4]) 
{
    // 参数检查
    if (target_vel == NULL || wheel_vel == NULL) {
        return -1;
    }
    
    float vx = target_vel[0];
    float vy = target_vel[1];
    float omega = target_vel[2];
    
    // 计算旋转半径
    float R = 0.21 + 0.23;
    
    // 逆运动学公式：
    // 左前轮：vx + vy + omega*R
    // 右前轮：vx - vy - omega*R
    // 右后轮：vx + vy - omega*R
    // 左后轮：vx - vy + omega*R
    
    wheel_vel[0] = vx + vy + omega * R;  // 左前轮
    wheel_vel[1] = vx - vy - omega * R;  // 右前轮
    wheel_vel[2] = vx + vy - omega * R;  // 右后轮
    wheel_vel[3] = vx - vy + omega * R;  // 左后轮
    
    return 0;
}

float speed_vector[3];
float wheel_vel[4];
void chassis_set_control()
{
    if(!chassis_rc_solve())
        chassis_mode = ZERO_FORCE;
    if(static_cast<pyro::dr16_drv_t::sw_state_t>(rc_data.sw_r) == pyro::dr16_drv_t::sw_state_t::SW_MID || static_cast<pyro::dr16_drv_t::sw_state_t>(rc_data.sw_r) == pyro::dr16_drv_t::sw_state_t::SW_DOWN)
    {
        chassis_mode = RC_CONTROL;
    }
    else
    {
        chassis_mode = ZERO_FORCE;
    }
    speed_vector[0] = rc_data.ch_ly*800;
    speed_vector[1] = rc_data.ch_lx*800;
    speed_vector[2] = rc_data.ch_rx*1200;

    mecanum_inverse_kinematics(speed_vector,wheel_vel);
    wheel_control_fl->set_target(wheel_vel[0]);
    wheel_control_fr->set_target(wheel_vel[1]);
    wheel_control_br->set_target(wheel_vel[2]);
    wheel_control_bl->set_target(wheel_vel[3]);
}
void chassis_zero_force()
{
    wheel_motor_fl->send_torque(0.0f);
    wheel_motor_fr->send_torque(0.0f);
    wheel_motor_bl->send_torque(0.0f);
    wheel_motor_br->send_torque(0.0f);
}

float control_torque[4]={};
void chassis_rc_control()
{
    wheel_control_fl->update();
    wheel_control_fr->update();
    wheel_control_bl->update();
    wheel_control_br->update();

    control_torque[0] = wheel_control_fl->control(0.01f);
    control_torque[1] = wheel_control_fr->control(0.01f);
    control_torque[2] = wheel_control_bl->control(0.01f);
    control_torque[3] = wheel_control_br->control(0.01f);

    wheel_motor_fl->send_torque(control_torque[0]);
    wheel_motor_fr->send_torque(control_torque[1]);
    wheel_motor_bl->send_torque(control_torque[2]);
    wheel_motor_br->send_torque(control_torque[3]);
}

extern "C" void engineer_chassis_mission(void const *argument)
{
    rc_sw_l_topic_id = global_databoard->get_topic_id("rc_sw_l");
    rc_sw_r_topic_id = global_databoard->get_topic_id("rc_sw_r");
    rc_ch_lx_topic_id = global_databoard->get_topic_id("rc_ch_lx");
    rc_ch_ly_topic_id = global_databoard->get_topic_id("rc_ch_ly");
    rc_ch_rx_topic_id = global_databoard->get_topic_id("rc_ch_rx");
    rc_ch_ry_topic_id = global_databoard->get_topic_id("rc_ch_ry");

    motor_torque_topic_id = global_databoard->get_topic_id("motor_torque");
    motor_rotate_topic_id = global_databoard->get_topic_id("motor_rotate");

    wheel_motor_fl = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,pyro::can_hub_t::can1);
    rot_pid_fl = new pyro::pid_t(0.4f,0.0f,0.0f,0.0f,10.0f);
    wheel_control_fl = new pyro::wheel_control_t(wheel_motor_fl,rot_pid_fl);
    wheel_control_fl->set_forward();

    wheel_motor_fr = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,pyro::can_hub_t::can1);
    rot_pid_fr = new pyro::pid_t(0.4f,0.0f,0.0f,0.0f,10.0f);
    wheel_control_fr = new pyro::wheel_control_t(wheel_motor_fr,rot_pid_fr);
    wheel_control_fr->set_reverse();

    wheel_motor_br = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3,pyro::can_hub_t::can1);
    rot_pid_br = new pyro::pid_t(0.4f,0.0f,0.0f,0.0f,10.0f);
    wheel_control_br = new pyro::wheel_control_t(wheel_motor_br,rot_pid_br);
    wheel_control_br->set_reverse();

    wheel_motor_bl = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_4,pyro::can_hub_t::can1);
    rot_pid_bl = new pyro::pid_t(0.4f,0.0f,0.0f,0.0f,10.0f);
    wheel_control_bl = new pyro::wheel_control_t(wheel_motor_bl,rot_pid_bl);
    wheel_control_bl->set_forward();
    for(;;)
    {
        

        wheel_motor_fl->update_feedback();
        wheel_motor_fr->update_feedback();
        wheel_motor_bl->update_feedback();
        wheel_motor_br->update_feedback();

        wheel_motor_current_position[0] = wheel_motor_fl->get_current_position();
        wheel_motor_current_position[1] = wheel_motor_fr->get_current_position();
        wheel_motor_current_position[2] = wheel_motor_bl->get_current_position();
        wheel_motor_current_position[3] = wheel_motor_br->get_current_position();

        wheel_motor_current_rotate[0] = wheel_motor_fl->get_current_rotate();
        wheel_motor_current_rotate[1] = wheel_motor_fr->get_current_rotate();
        wheel_motor_current_rotate[2] = wheel_motor_bl->get_current_rotate();
        wheel_motor_current_rotate[3] = wheel_motor_br->get_current_rotate();

        wheel_motor_current_torque[0] = wheel_motor_fl->get_current_torque();
        wheel_motor_current_torque[1] = wheel_motor_fr->get_current_torque();
        wheel_motor_current_torque[2] = wheel_motor_bl->get_current_torque();
        wheel_motor_current_torque[3] = wheel_motor_br->get_current_torque();

        global_databoard->write_topic(motor_torque_topic_id,*((pyro::genenral_data_t*)&(wheel_motor_current_torque[0])));
        global_databoard->write_topic(motor_rotate_topic_id,*((pyro::genenral_data_t*)&(wheel_motor_current_rotate[0])));

        chassis_set_control();

        if(chassis_mode == ZERO_FORCE)
        {
            chassis_zero_force();
        }
        else if(chassis_mode == RC_CONTROL)
        {
            chassis_rc_control();
        }



        vTaskDelay(1);
    }
}
