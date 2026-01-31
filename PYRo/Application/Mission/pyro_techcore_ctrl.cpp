#include "pyro_core_config.h"

#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"

#include "pyro_algo_pid.h"

#include "pyro_rc_hub.h"
namespace pyro
{
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
        void update() 
        {
            _motor->update_feedback();
            _feedback_pos = _motor->get_current_position()-_feedback_pos_offset;
            _feedback_rot = _motor->get_current_rotate();
            if(_feedback_pos>PI)
            {
                _feedback_pos-=2*PI;
            }
            else if(_feedback_pos<-PI)
            {
                _feedback_pos+=2*PI;
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

pyro::dm_motor_drv_t* yaw_motor;
pyro::pid_t *yaw_rot_pid,*yaw_pos_pid;
pyro::axis_control_t* yaw_control;

pyro::dm_motor_drv_t* pitch_motor;
pyro::pid_t *pitch_rot_pid,*pitch_pos_pid;
pyro::axis_control_t* pitch_control;

pyro::dm_motor_drv_t* roll_motor;
pyro::pid_t *roll_rot_pid,*roll_pos_pid;
pyro::axis_control_t* roll_control;

pyro::dji_m3508_motor_drv_t* x_axis_motor;
pyro::dji_m3508_motor_drv_t* z_axis_motor;

pyro::rc_drv_t* dr16_drv;

float axis_angle[3]={};//YPR
float axis_angle_with_offset[3]={};
float axis_rotate[3]={};
float axis_torque[3]={};

float axis_target_angle[3]={};

float move_angle[2]={};//X,Z
float move_rotate[2]={};
float move_torque[2]={};


float control_current[2]={};//X,Z
typedef enum
{
    ZERO_FORCE,
    RESET_POSE,
    RC_CONTROL
}
control_mode_t;

control_mode_t control_mode=ZERO_FORCE;

const pyro::dr16_drv_t::dr16_ctrl_t *rc_data;

void techcore_setcontrol()
{
    rc_data = static_cast<const pyro::dr16_drv_t::dr16_ctrl_t *>(dr16_drv->read());
    if(rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_DOWN||rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_MID)
    {
        control_mode = RC_CONTROL;
    }
    else if(rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_UP)
    {
        control_mode = ZERO_FORCE;
    } 

    if(control_mode == RC_CONTROL)
    {
        control_current[0] = rc_data->rc.ch_lx*4;
        control_current[1] = rc_data->rc.ch_ly*4;
    
        axis_target_angle[0] = rc_data->rc.ch_rx*3;
        axis_target_angle[1] = rc_data->rc.ch_ry*3;
        axis_target_angle[2] = rc_data->rc.wheel*3;
    }
    else
    {
        control_current[0] = 0.0f;
        control_current[1] = 0.0f;
        axis_target_angle[0] = 0.0f;
        axis_target_angle[1] = 0.0f;
        axis_target_angle[2] = 0.0f;

    }
}
extern "C" void techcore_ctrl_task(void *param)
{
    osDelay(1000);

    dr16_drv =  pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16);
    dr16_drv->init();
    dr16_drv->enable();

    yaw_motor = new pyro::dm_motor_drv_t(0x15,0x16,pyro::can_hub_t::which_can::can1);
    yaw_motor->set_torque_range(-10.0f,10.0f);
    yaw_motor->set_position_range(-20.0f,20.0f);
    yaw_motor->set_position_range(-3.14f,3.14f);
    yaw_rot_pid = new pyro::pid_t(6.4f,0.0f,0.0f,0.0f,10.0f);
    yaw_pos_pid = new pyro::pid_t(3.5f,0.1f,0.04f,1.0f,20.0f);
    yaw_control = new pyro::axis_control_t(yaw_motor,yaw_pos_pid,yaw_rot_pid);
    yaw_control->set_feedback_pos_offset(-2.751);
    yaw_control->enable_constraint();
    yaw_control->set_upper_limit(1.57f);
    yaw_control->set_lower_limit(-1.57f);

    pitch_motor = new pyro::dm_motor_drv_t(0x11,0x12,pyro::can_hub_t::which_can::can1);
    pitch_motor->set_torque_range(-8.0f,8.0f);
    pitch_motor->set_position_range(-20.0f,20.0f);
    pitch_motor->set_position_range(-3.14f,3.14f);
    pitch_rot_pid = new pyro::pid_t(6.4f,0.1f,0.0f,1.0f,8.0f);
    pitch_pos_pid = new pyro::pid_t(3.5f,0.2f,0.08f,1.0f,20.0f);
    pitch_control = new pyro::axis_control_t(pitch_motor,pitch_pos_pid,pitch_rot_pid);
    pitch_control->enable_constraint();
    pitch_control->set_upper_limit(1.57);
    pitch_control->set_lower_limit(0);
    pitch_control->set_feedback_pos_offset(-1.92);

    roll_motor = new pyro::dm_motor_drv_t(0x13,0x14,pyro::can_hub_t::which_can::can1);
    roll_motor->set_torque_range(-10.0f,10.0f);
    roll_motor->set_position_range(-20.0f,20.0f);
    roll_motor->set_position_range(-3.14f,3.14f);
    roll_rot_pid = new pyro::pid_t(6.4f,0.0f,0.0f,0.0f,10.0f);
    roll_pos_pid = new pyro::pid_t(3.5f,0.1f,0.08f,1.0f,20.0f);
    roll_control = new pyro::axis_control_t(roll_motor,roll_pos_pid,roll_rot_pid);
    roll_control->enable_constraint();
    roll_control->set_upper_limit(1.57);
    roll_control->set_lower_limit(-1.57);
    roll_control->set_feedback_pos_offset(0.778);

    x_axis_motor = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,pyro::can_hub_t::which_can::can2);

    z_axis_motor = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,pyro::can_hub_t::which_can::can2);

    osDelay(1000);
    yaw_motor->enable();
    vTaskDelay(1);
    pitch_motor->enable();
    vTaskDelay(1);
    roll_motor->enable();
    vTaskDelay(1);
    for(;;)
    {
        yaw_motor->update_feedback();
        pitch_motor->update_feedback();
        roll_motor->update_feedback();

        yaw_control->update();
        pitch_control->update();
        roll_control->update();

        x_axis_motor->update_feedback();
        z_axis_motor->update_feedback();

        axis_angle[0] = yaw_motor->get_current_position();
        axis_angle[1] = pitch_motor->get_current_position();
        axis_angle[2] = roll_motor->get_current_position();

        axis_torque[0] = yaw_motor->get_current_torque();
        axis_torque[1] = pitch_motor->get_current_torque();
        axis_torque[2] = roll_motor->get_current_torque();

        axis_rotate[0] = yaw_motor->get_current_rotate();
        axis_rotate[1] = pitch_motor->get_current_rotate();
        axis_rotate[2] = roll_motor->get_current_rotate();

        axis_angle_with_offset[0] = yaw_control->get_position();
        axis_angle_with_offset[1] = pitch_control->get_position();
        axis_angle_with_offset[2] = roll_control->get_position();

        move_angle[0] = x_axis_motor->get_current_position();
        move_angle[1] = z_axis_motor->get_current_position();

        move_torque[0] = x_axis_motor->get_current_torque();
        move_torque[1] = z_axis_motor->get_current_torque();

        move_rotate[0] = x_axis_motor->get_current_rotate();
        move_rotate[1] = z_axis_motor->get_current_rotate();

        techcore_setcontrol();
        yaw_control->set_target(axis_target_angle[0]);
        pitch_control->set_target(axis_target_angle[1]);
        roll_control->set_target(axis_target_angle[2]);

        if(control_mode == ZERO_FORCE)
        {
        yaw_motor->send_torque(0.0f);
        pitch_motor->send_torque(0.0f);
        roll_motor->send_torque(0.0f);

        x_axis_motor->send_torque(0.0f);
        z_axis_motor->send_torque(0.0f);
        }
        else if(control_mode == RC_CONTROL)
        {
        // yaw_motor->send_torque(control_current[0]);
        // pitch_motor->send_torque(control_current[1]);
        // roll_motor->send_torque(control_current[0]);
        yaw_control->control(0);
        pitch_control->control(0);
        roll_control->control(0);

        x_axis_motor->send_torque(control_current[0]);
        z_axis_motor->send_torque(control_current[1]);
        }
        vTaskDelay(1);
    }
}