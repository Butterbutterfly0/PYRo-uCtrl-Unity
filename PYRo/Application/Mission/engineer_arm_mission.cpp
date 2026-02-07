#include "pyro_core_config.h"
#include "cmsis_os.h"

#include "pyro_dm_motor_drv.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_algo_pid.h"

#include "pyro_rc_hub.h"

extern "C" void engineer_arm_mission(void* args);
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
            // _control_value = 0.0f;
            if(_constraint==CONSTRAINT)
            {
                _target_rot=_pos_pid->calculate(_target_pos,_feedback_pos);
            }
            else
            {
                _target_rot = _pos_pid->calculate(rot_angle_correction_pi(_feedback_pos,_target_pos,PI,-PI),_feedback_pos);
                // _target_rot=_pos_pid->calculate(_target_pos,_feedback_pos);
            }
            _control_value = _rot_pid->calculate(_target_rot,_feedback_rot);
            _motor->send_torque(_control_value);
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

pyro::rc_drv_t* dr16_drv;

pyro::axis_control_t *axis1;
pyro::dm_motor_drv_t *axis1_motor;

pyro::axis_control_t *axis2;
pyro::dm_motor_drv_t *axis2_motor;

pyro::axis_control_t *axis3;
pyro::dm_motor_drv_t *axis3_motor;

pyro::axis_control_t *axis4;
pyro::dm_motor_drv_t *axis4_motor;

pyro::axis_control_t *axis5;
pyro::dm_motor_drv_t *axis5_motor;

pyro::axis_control_t *axis6;
pyro::pid_t *axis6_pos_pid ;
pyro::pid_t *axis6_rot_pid ;
pyro::dji_gm_6020_motor_drv_t *axis6_motor;
float axis6_target_pos,axis6_target_rot,axis6_target_torque,axis6_feedback_pos,axis6_feedback_rot;

pyro::dji_m2006_motor_drv_t *end_motor;

float motor_current_pos[7]={};
float motor_target_pos[7]={};
float motor_current_rot[7]={};
float motor_current_torque[7]={};

float end_targat_torque= 0;


typedef enum
{
    ZERO_FORCE,
    RESET_POSE,
    RC_CONTROL
}
control_mode_t;

control_mode_t control_mode=ZERO_FORCE;

void engineer_arm_init()
{
    dr16_drv =  pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16);

    pyro::pid_t *axis1_pos_pid = new pyro::pid_t(5,0.0,0.0,0.0,10);
    pyro::pid_t *axis1_rot_pid = new pyro::pid_t(2.2,0.0,0.0,0.0,20);
    axis1_motor = new pyro::dm_motor_drv_t(0x7, 0x6, pyro::can_hub_t::can1);
    axis1_motor->set_position_range(-pyro::PI, pyro::PI);
    axis1_motor->set_rotate_range(-10, 10); 
    axis1_motor->set_torque_range(-28, 28);
    axis1 = new pyro::axis_control_t(axis1_motor,axis1_pos_pid,axis1_rot_pid);
    axis1->enable_constraint();
    axis1->set_upper_limit(pyro::PI/2);
    axis1->set_lower_limit(-pyro::PI/2);

    pyro::pid_t *axis2_pos_pid = new pyro::pid_t(11,0,0.05,20.0,45);
    pyro::pid_t *axis2_rot_pid = new pyro::pid_t(13,0.4,0.03,20.0,54);
    axis2_motor = new pyro::dm_motor_drv_t(0x1, 0x0, pyro::can_hub_t::can1);
    axis2_motor->set_position_range(-pyro::PI, pyro::PI);
    axis2_motor->set_rotate_range(-45, 45); 
    axis2_motor->set_torque_range(-54, 54);
    axis2 = new pyro::axis_control_t(axis2_motor,axis2_pos_pid,axis2_rot_pid);
    axis2->enable_constraint();
    axis2->set_upper_limit(2.83);
    axis2->set_lower_limit(-0.1);
    axis2->set_feedback_pos_offset(-0.046355);

    pyro::pid_t *axis3_pos_pid = new pyro::pid_t(10,0.0,0.0,0.0,20);
    pyro::pid_t *axis3_rot_pid = new pyro::pid_t(9,0.0,0.0,0.0,20);
    axis3_motor = new pyro::dm_motor_drv_t(0x3, 0x2, pyro::can_hub_t::can1);
    axis3_motor->set_position_range(-pyro::PI, pyro::PI);
    axis3_motor->set_rotate_range(-20, 20); 
    axis3_motor->set_torque_range(-20, 20);
    axis3 = new pyro::axis_control_t(axis3_motor,axis3_pos_pid,axis3_rot_pid);
    axis3->enable_constraint();
    axis3->set_upper_limit(2.83);
    axis3->set_lower_limit(-0.01);
    axis3->set_feedback_pos_offset(-0.010884);

    pyro::pid_t *axis4_pos_pid = new pyro::pid_t(6.7,1.2,0.0,6,15);
    pyro::pid_t *axis4_rot_pid = new pyro::pid_t(1.2,0.1,0.001,1,5);
    axis4_motor = new pyro::dm_motor_drv_t(0x5, 0x4, pyro::can_hub_t::can2);
    axis4_motor->set_position_range(-pyro::PI, pyro::PI);
    axis4_motor->set_rotate_range(-20, 20); 
    axis4_motor->set_torque_range(-10, 10);
    axis4 = new pyro::axis_control_t(axis4_motor,axis4_pos_pid,axis4_rot_pid);
    axis4->enable_constraint();
    axis4->set_upper_limit(pyro::PI/2);
    axis4->set_lower_limit(-pyro::PI/2);
    axis4->set_feedback_pos_offset(1.05362129);


    // pyro::pid_t *axis5_pos_pid = new pyro::pid_t(6.3,0,0.0,10,30);
    // pyro::pid_t *axis5_rot_pid = new pyro::pid_t(1.0,0.1,0.00,4,8);

    pyro::pid_t *axis5_pos_pid = new pyro::pid_t(10.3,0.2,0.0,10,30);
    pyro::pid_t *axis5_rot_pid = new pyro::pid_t(1.0,0.1,0.00,4,8);
    axis5_motor = new pyro::dm_motor_drv_t(0x8, 0x9, pyro::can_hub_t::can2);
    axis5_motor->set_position_range(-pyro::PI, pyro::PI);
    axis5_motor->set_rotate_range(-30, 30); 
    axis5_motor->set_torque_range(-10, 10);
    axis5 = new pyro::axis_control_t(axis5_motor,axis5_pos_pid,axis5_rot_pid);
    axis5->enable_constraint();
    axis5->set_upper_limit(0.0);
    axis5->set_lower_limit(-0.66);
    axis5->set_feedback_pos_offset(-0.0663935);

    axis6_pos_pid = new pyro::pid_t(9,0.0,0.0,0.0,20);
    axis6_rot_pid = new pyro::pid_t(0.12,0.1,0.0,1,3);
    axis6_motor =  new pyro::dji_gm_6020_motor_drv_t(pyro::dji_motor_tx_frame_t::register_id_t::id_1,pyro::can_hub_t::can3);
    axis6 = new pyro::axis_control_t(axis6_motor,axis6_pos_pid,axis6_rot_pid);
    axis6->set_feedback_pos_offset(0.11121);

    end_motor = new pyro::dji_m2006_motor_drv_t(pyro::dji_motor_tx_frame_t::register_id_t::id_1,pyro::can_hub_t::can3);
}

void enigneer_arm_update()
{
    axis1_motor->update_feedback();
    axis2_motor->update_feedback();
    axis3_motor->update_feedback();
    axis4_motor->update_feedback();
    axis5_motor->update_feedback();
    axis6_motor->update_feedback();
    end_motor->update_feedback();

    motor_current_pos[0]=axis1_motor->get_current_position();
    motor_current_pos[1]=axis2_motor->get_current_position();
    motor_current_pos[2]=axis3_motor->get_current_position();
    motor_current_pos[3]=axis4_motor->get_current_position();
    motor_current_pos[4]=axis5_motor->get_current_position();
    motor_current_pos[5]=axis6_motor->get_current_position();
    motor_current_pos[6]=end_motor->get_current_position();

    motor_current_rot[0]=axis1_motor->get_current_rotate();
    motor_current_rot[1]=axis2_motor->get_current_rotate();
    motor_current_rot[2]=axis3_motor->get_current_rotate();
    motor_current_rot[3]=axis4_motor->get_current_rotate();
    motor_current_rot[4]=axis5_motor->get_current_rotate();
    motor_current_rot[5]=axis6_motor->get_current_rotate();
    motor_current_rot[6]=end_motor->get_current_rotate();

    motor_current_torque[0]=axis1_motor->get_current_torque();
    motor_current_torque[1]=axis2_motor->get_current_torque();
    motor_current_torque[2]=axis3_motor->get_current_torque();
    motor_current_torque[3]=axis4_motor->get_current_torque();
    motor_current_torque[4]=axis5_motor->get_current_torque();
    motor_current_torque[5]=axis6_motor->get_current_torque();
    motor_current_torque[6]=end_motor->get_current_torque();

    axis1->update();
    axis2->update();
    axis3->update();
    axis4->update();
    axis5->update();
    axis6->update();
}

const  pyro::dr16_drv_t::dr16_ctrl_t *rc_data;
void engineer_arm_set_control()
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

    if(control_mode==RC_CONTROL)
    {
        if(rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_MID)
        {
            motor_target_pos[0]+=rc_data->rc.ch_ly*0.01;
            motor_target_pos[1]+=rc_data->rc.ch_lx*0.01;
            motor_target_pos[2]+=rc_data->rc.ch_ry*0.01;
            motor_target_pos[3]+=rc_data->rc.ch_rx*0.01;

        }
        else if(rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_DOWN)
        {
            motor_target_pos[4]+=rc_data->rc.ch_ly*0.001;
            motor_target_pos[5]+=rc_data->rc.ch_lx*0.01;
        }
        end_targat_torque = rc_data->rc.wheel*1;
        axis1->set_target(motor_target_pos[0]);
        axis2->set_target(motor_target_pos[1]);
        axis3->set_target(motor_target_pos[2]);
        axis4->set_target(motor_target_pos[3]);
        axis5->set_target(motor_target_pos[4]);
        axis6->set_target(motor_target_pos[5]);

        
    }
    else
    {
        for(int i=0;i<7;i++)
        {
            motor_target_pos[i]=0;
        }
    }

}

void engineer_arm_zeroforce()
{
    axis1_motor->send_torque(0);
    axis2_motor->send_torque(0);
    axis3_motor->send_torque(0);
    axis4_motor->send_torque(0);
    axis5_motor->send_torque(0);
    axis6_motor->send_torque(0);
    end_motor->send_torque(0);
}

void engineer_arm_control()
{
    axis1->control(0.005);
    axis2->control(0.005);
    axis3->control(0.005);


    axis4->control(0.005);
    axis5->control(0.005);
    axis6->control(0.005);

    end_motor->send_torque(end_targat_torque);
}

void engineer_arm_mission(void* args)
{
    osDelay(10);

    engineer_arm_init();

    vTaskDelay(1500);

    axis1_motor->enable();
    vTaskDelay(1);
    axis2_motor->enable();
    vTaskDelay(1);
    axis3_motor->enable();
    vTaskDelay(1);
    axis4_motor->enable();
    vTaskDelay(1);
    axis5_motor->enable();
    vTaskDelay(1);

    for(;;)
    {
        enigneer_arm_update();
        engineer_arm_set_control();
        if(control_mode==ZERO_FORCE)
        {
            engineer_arm_zeroforce();
        }
        else if(control_mode==RC_CONTROL)
        {
            engineer_arm_control();
        }
        vTaskDelay(1);
    }
}

