#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "pyro_databoard.h"
#include "pyro_rc_hub.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_algo_pid.h"
#include "pyro_power_control_drv.h"
#include <math.h>

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
            if(_ward == FORWARD){_target_rot = target;}
            else{_target_rot = -target;}
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
        void set_forward() { _ward = FORWARD;}
        void set_reverse() {_ward = REVERSE;}
};
class axis_control_t 
{ 
        enum constraint_t
    {
        NO_CONSTRAINT,
        CONSTRAINT
    };
    protected:
        pyro::motor_base_t* _motor;
        pyro::pid_t* _pos_pid;
        pyro::pid_t* _rot_pid;

        constraint_t _constraint=NO_CONSTRAINT;

        float _target_pos;
        float _target_rot;

        float _feedback_pos;
        float _feedback_pos_offset;
        float _feedback_rot;

        float _control_value;

        float _upper_limit;
        float _lower_limit;

    public:
        axis_control_t(pyro::motor_base_t* motor, pyro::pid_t* pos_pid, pyro::pid_t* rot_pid):_motor(motor),_pos_pid(pos_pid),_rot_pid(rot_pid)
        {
            _target_pos = 0.0f;
            _target_rot = 0.0f;
        }
        ~axis_control_t(){}
        void set_target(float& target) 
        {
            _target_pos = target;
            if(_constraint==CONSTRAINT)
            {
                if(_target_pos>_upper_limit)
                {
                    _target_pos=_upper_limit;
                }
                else if(_target_pos<_lower_limit)
                {
                    _target_pos=_lower_limit;
                }
            }
            else
            {
                if(_target_pos>_upper_limit)
                {
                    _target_pos=_target_pos-2*pyro::PI;
                }
                else if(_target_pos<_lower_limit)
                {
                    _target_pos=_target_pos+2*pyro::PI;
                }
            }
            target = _target_pos;
        }
        void increment(float increment)
        {
            _target_pos = _target_pos+increment;
            if(_constraint==CONSTRAINT)
            {
                if(_target_pos>_upper_limit)
                {
                    _target_pos=_upper_limit;
                }
                else if(_target_pos<_lower_limit)
                {
                    _target_pos=_lower_limit;
                }
            }
            else
            {
                if(_target_pos>_upper_limit)
                {
                    _target_pos=_target_pos-2*pyro::PI;
                }
                else if(_target_pos<_lower_limit)
                {
                    _target_pos=_target_pos+2*pyro::PI;
                }
            }
        }
        void update() 
        {
            _motor->update_feedback();
            _feedback_pos = _motor->get_current_position()-_feedback_pos_offset;
            _feedback_rot = _motor->get_current_rotate();
            if(_feedback_pos>10)
            {
                _feedback_pos-=2*10;
            }
            else if(_feedback_pos<-10)
            {
                _feedback_pos+=2*10;
            }
        }
        inline float rot_angle_correction_pi(float feedback,float target,float max,float min)
        {
            if(feedback-target>PI)
		        return target-min+max;
	        else if(target-feedback>PI)
		        return target-max+min;
	        else 
		        return target;
        }
        void control(float dt)
        {   
            float control_value = 0.0f;
            if(_constraint==CONSTRAINT)
            {
                _target_rot=_pos_pid->calculate(_target_pos,_feedback_pos);
            }
            else
            {
                _target_rot = _pos_pid->calculate(rot_angle_correction_pi(_feedback_pos,_target_pos,PI,-PI),_feedback_pos);
                // _target_rot=_pos_pid->calculate(_target_pos,_feedback_pos);
            }
            control_value = _rot_pid->calculate(_target_rot,_feedback_rot);
            _motor->send_torque(control_value);
        }
        float get_position(){return _feedback_pos;}
        void enable_constraint(){_constraint=CONSTRAINT;}
        void disable_constraint(){_constraint=NO_CONSTRAINT;
         _upper_limit=pyro::PI;_lower_limit=-pyro::PI;}
        void set_upper_limit(float upper_limit){ _upper_limit=upper_limit;}
        void set_lower_limit(float lower_limit){_lower_limit=lower_limit;}

        void set_feedback_pos_offset(float offset){_feedback_pos_offset=offset;}
    
};

};

typedef enum
{
    ZERO_FORCE = 0,
    RC_CONTROL
}
chassis_mode_t;
chassis_mode_t chassis_mode = ZERO_FORCE,last_chassis_mode = ZERO_FORCE;

uint8_t joint_cali_flag = 0;
uint32_t joint_cali_tick_start,joint_cali_tick_count;



static uint32_t 
rc_sw_l_topic_id,
rc_sw_r_topic_id,
rc_ch_lx_topic_id,
rc_ch_ly_topic_id,
rc_ch_rx_topic_id,
rc_ch_ry_topic_id,
motor_torque_topic_id,
motor_rotate_topic_id;

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

uint32_t timestamp;

pyro::dji_m3508_motor_drv_t *wheel_motor_fl,*wheel_motor_fr,*wheel_motor_br,*wheel_motor_bl;
pyro::pid_t *rot_pid_fl,*rot_pid_fr,*rot_pid_br,*rot_pid_bl;
pyro::wheel_control_t *wheel_control_fl,*wheel_control_fr,*wheel_control_br,*wheel_control_bl;

pyro::dm_motor_drv_t *left_side_joint_motor_drv,*right_side_joint_motor_drv;
pyro::pid_t *left_side_joint_pos_pid,*right_side_joint_pos_pid;
pyro::pid_t *left_side_joint_rot_pid,*right_side_joint_rot_pid;
pyro::axis_control_t *left_side_joint_control,*right_side_joint_control;

float wheel_motor_current_position[4];
float wheel_motor_current_rotate[4];
float wheel_motor_current_torque[4];

float joint_motor_current_position[2];// l r
float joint_motor_current_rotate[2];//l r
float joint_motor_current_torque[2];//l r

bool chassis_rc_solve()
{
    pyro::topic::data_status_t rc_data_status;
    rc_data_status = global_databoard->read(rc_sw_l_topic_id,(pyro::genenral_data_t*)&(rc_data.sw_l),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
        return false;
    rc_data_status = global_databoard->read(rc_sw_r_topic_id,(pyro::genenral_data_t*)&(rc_data.sw_r),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
        return false;
    rc_data_status = global_databoard->read(rc_ch_lx_topic_id,(pyro::genenral_data_t*)&(rc_data.ch_lx),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
        return false;
    rc_data_status = global_databoard->read(rc_ch_ly_topic_id,(pyro::genenral_data_t*)&(rc_data.ch_ly),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
        return false;
    rc_data_status = global_databoard->read(rc_ch_rx_topic_id,(pyro::genenral_data_t*)&(rc_data.ch_rx),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
        return false;
    rc_data_status = global_databoard->read(rc_ch_ry_topic_id,(pyro::genenral_data_t*)&(rc_data.ch_ry),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
        return false;
    return true;
}

void joint_cali_start()
{
    joint_cali_flag = 1;
    joint_cali_tick_start = xTaskGetTickCount();
    joint_cali_tick_count = 0;
}

void joint_cali_stop()
{
    joint_cali_flag = 0;
    left_side_joint_control->set_feedback_pos_offset(left_side_joint_motor_drv->get_current_position());
    right_side_joint_control->set_feedback_pos_offset(right_side_joint_motor_drv->get_current_position());
    float t = 0;
    left_side_joint_control->set_target(t);
    t=0;
    right_side_joint_control->set_target(t);
}

void joint_cali_update()
{
    if(!joint_cali_flag)
        return;

    joint_cali_tick_count = xTaskGetTickCount()-joint_cali_tick_start;
    if(joint_cali_tick_count > 3000)
    {
        joint_cali_stop();
    }
}

void joint_cali_set_control()
{
    ;
}

void joint_normal_set_control(float increment)
{
    left_side_joint_control->increment(-increment);
    right_side_joint_control->increment(increment);
}


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

    if(last_chassis_mode != chassis_mode && chassis_mode == RC_CONTROL)
    {
        joint_cali_start();
    }

    if(joint_cali_flag)
    {
        joint_cali_set_control();
    }
    else
    {
        joint_normal_set_control(rc_data.ch_ry*0.005);
    }
    
    last_chassis_mode = chassis_mode;
}

void joint_cali_control()
{
    left_side_joint_motor_drv->send_torque(2);
    right_side_joint_motor_drv->send_torque(-2);
}

void joint_normal_control()
{

    left_side_joint_control->update();
    right_side_joint_control->update();
    left_side_joint_control->control(0.01f);
    right_side_joint_control->control(0.01f);
}

void joint_control()
{
    if(joint_cali_flag)
    {
        joint_cali_control();
    }
    else
    {
        joint_normal_control();
    }
}

void chassis_zero_force()
{
    wheel_motor_fl->send_torque(0.0f);
    wheel_motor_fr->send_torque(0.0f);
    wheel_motor_bl->send_torque(0.0f);
    wheel_motor_br->send_torque(0.0f);
}

pyro::power_control_drv_t& power_controller = pyro::power_control_drv_t::get_instance(4);

float control_torque[4]={};
pyro::power_control_drv_t::motor_data_t _motor_data[4];

void chassis_power_control()
{
    _motor_data[0].torque_cmd = control_torque[0];
    _motor_data[1].torque_cmd = control_torque[1];
    _motor_data[2].torque_cmd = control_torque[2];
    _motor_data[3].torque_cmd = control_torque[3];

    _motor_data[0].gyro = wheel_motor_current_rotate[0];
    _motor_data[1].gyro = wheel_motor_current_rotate[1];
    _motor_data[2].gyro = wheel_motor_current_rotate[2];
    _motor_data[3].gyro = wheel_motor_current_rotate[3];

    for(int i = 1; i <= 4; i++)
    {
        _motor_data[i-1].power_predict = power_controller.motor_power_predict(i,_motor_data[i-1].torque_cmd,_motor_data[i-1].gyro);
    }
    power_controller.calculate_restricted_torques(_motor_data,4,110);
    for(int i = 0; i < 4; i++)
    {
        control_torque[i] = _motor_data[i].restricted_torque;
    }
}
void chassis_rc_control()
{
    wheel_control_fl->update();
    wheel_control_fr->update();
    wheel_control_br->update();
    wheel_control_bl->update();

    control_torque[0] = wheel_control_fl->control(0.01f);
    control_torque[1] = wheel_control_fr->control(0.01f);
    control_torque[2] = wheel_control_br->control(0.01f);
    control_torque[3] = wheel_control_bl->control(0.01f);

    chassis_power_control();

    wheel_motor_fl->send_torque(control_torque[0]);
    wheel_motor_fr->send_torque(control_torque[1]);
    wheel_motor_br->send_torque(control_torque[2]);
    wheel_motor_bl->send_torque(control_torque[3]);
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

    pyro::power_control_drv_t::motor_coefficient_t motor_coefficient_fl;
    motor_coefficient_fl.k1 = 0.0115f;
    motor_coefficient_fl.k2 = 0.0391f;
    motor_coefficient_fl.k3 = 0.2739f;
    motor_coefficient_fl.k4 = -3.2137f;
    power_controller.set_motor_coefficient(1,motor_coefficient_fl);

    pyro::power_control_drv_t::motor_coefficient_t motor_coefficient_fr;
    motor_coefficient_fr.k1 = 0.0114f;
    motor_coefficient_fr.k2 = 0.0214f;
    motor_coefficient_fr.k3 = 0.2181f;
    motor_coefficient_fr.k4 = 0.4862f;
    power_controller.set_motor_coefficient(2,motor_coefficient_fr);

    pyro::power_control_drv_t::motor_coefficient_t motor_coefficient_bl;
    motor_coefficient_bl.k1 = 0.0111f;
    motor_coefficient_bl.k2 = 0.0430f;
    motor_coefficient_bl.k3 = 0.4013f;
    motor_coefficient_bl.k4 = -11.0010f;
    power_controller.set_motor_coefficient(4,motor_coefficient_bl);

    pyro::power_control_drv_t::motor_coefficient_t motor_coefficient_br;
    motor_coefficient_br.k1 = 0.0120f;
    motor_coefficient_br.k2 = 0.0353f;
    motor_coefficient_br.k3 = 0.3015f;
    motor_coefficient_br.k4 = -4.8325f;
    power_controller.set_motor_coefficient(3,motor_coefficient_br);

    left_side_joint_motor_drv = new pyro::dm_motor_drv_t(0x02,0x03,pyro::can_hub_t::can2);
    left_side_joint_motor_drv->set_position_range(-10,10);
    left_side_joint_motor_drv->set_rotate_range(-20,20);
    left_side_joint_motor_drv->set_torque_range(-20,20);

    left_side_joint_rot_pid = new pyro::pid_t(2.0f, 0.005f, 0.0012f, 0.5f, 20.0f);
    left_side_joint_pos_pid = new pyro::pid_t(14.0f, 0.005f, 0.0012f, 0.5f, 20.0f);

    left_side_joint_control = new pyro::axis_control_t(left_side_joint_motor_drv,left_side_joint_pos_pid,left_side_joint_rot_pid);
    left_side_joint_control->enable_constraint();
    left_side_joint_control->set_upper_limit(0);
    left_side_joint_control->set_lower_limit(-10);


    right_side_joint_motor_drv = new pyro::dm_motor_drv_t(0x00,0x01,pyro::can_hub_t::can2);
    right_side_joint_motor_drv->set_position_range(-10,10);
    right_side_joint_motor_drv->set_rotate_range(-20,20);
    right_side_joint_motor_drv->set_torque_range(-20,20);

    right_side_joint_rot_pid = new pyro::pid_t(2.0f, 0.005f, 0.0012f, 0.5f, 20.0f);
    right_side_joint_pos_pid = new pyro::pid_t(14.0f, 0.005f, 0.0012f, 0.5f, 20.0f);

    right_side_joint_control = new pyro::axis_control_t(right_side_joint_motor_drv,right_side_joint_pos_pid,right_side_joint_rot_pid);
    right_side_joint_control->enable_constraint();
    right_side_joint_control->set_upper_limit(10);
    right_side_joint_control->set_lower_limit(0);

    vTaskDelay(1000);
    left_side_joint_motor_drv->enable();
    vTaskDelay(1);
    right_side_joint_motor_drv->enable();
    vTaskDelay(1);
    for(;;)
    {
        left_side_joint_motor_drv->update_feedback();
        right_side_joint_motor_drv->update_feedback();

        joint_motor_current_position[0] = left_side_joint_motor_drv->get_current_position();
        joint_motor_current_position[1] = right_side_joint_motor_drv->get_current_position();

        joint_motor_current_rotate[0] = left_side_joint_motor_drv->get_current_rotate();
        joint_motor_current_rotate[1] = right_side_joint_motor_drv->get_current_rotate();

        joint_motor_current_torque[0] = left_side_joint_motor_drv->get_current_torque();

        joint_motor_current_torque[1] = right_side_joint_motor_drv->get_current_torque();

        wheel_motor_fl->update_feedback();
        wheel_motor_fr->update_feedback();
        wheel_motor_bl->update_feedback();
        wheel_motor_br->update_feedback();

        wheel_motor_current_position[0] = wheel_motor_fl->get_current_position();
        wheel_motor_current_position[1] = wheel_motor_fr->get_current_position();
        wheel_motor_current_position[2] = wheel_motor_br->get_current_position();
        wheel_motor_current_position[3] = wheel_motor_bl->get_current_position();

        wheel_motor_current_rotate[0] = wheel_motor_fl->get_current_rotate();
        wheel_motor_current_rotate[1] = wheel_motor_fr->get_current_rotate();
        wheel_motor_current_rotate[2] = wheel_motor_br->get_current_rotate();
        wheel_motor_current_rotate[3] = wheel_motor_bl->get_current_rotate();

        wheel_motor_current_torque[0] = wheel_motor_fl->get_current_torque();
        wheel_motor_current_torque[1] = wheel_motor_fr->get_current_torque();
        wheel_motor_current_torque[2] = wheel_motor_br->get_current_torque();
        wheel_motor_current_torque[3] = wheel_motor_bl->get_current_torque();

        global_databoard->write_topic(motor_torque_topic_id,*((pyro::genenral_data_t*)&(wheel_motor_current_torque[3])));
        global_databoard->write_topic(motor_rotate_topic_id,*((pyro::genenral_data_t*)&(wheel_motor_current_rotate[3])));

        joint_cali_update();
        
        chassis_set_control();


        if(chassis_mode == ZERO_FORCE)
        {
            chassis_zero_force();
            left_side_joint_motor_drv->send_torque(0);
            right_side_joint_motor_drv->send_torque(0);
        }
        else if(chassis_mode == RC_CONTROL)
        {
            chassis_rc_control();
            joint_control();
        }
        vTaskDelay(1);
    }
}
