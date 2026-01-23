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
        }
        void update() 
        {
            _motor->update_feedback();
            _feedback_pos = _motor->get_current_position();
            _feedback_rot = _motor->get_current_rotate();
        }
        void control(float dt)
        {   
            float control_value = 0.0f;
            _target_rot=_pos_pid->calculate(_feedback_pos,_target_pos);
            control_value = _rot_pid->calculate(_feedback_rot,_target_rot);
            _motor->send_torque(control_value);
        }
        void enable_constraint(){_constraint=CONSTRAINT;}
        void disable_constraint(){_constraint=NO_CONSTRAINT;}
        void set_upper_limit(float upper_limit){ _upper_limit=upper_limit;}
        void set_lower_limit(float lower_limit){_lower_limit=lower_limit;}
    
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
pyro::dji_gm_6020_motor_drv_t *axis6_motor;

pyro::dji_m2006_motor_drv_t *end_motor;

float motor_current_pos[7]={};
float motor_current_rot[7]={};
float motor_current_torque[7]={};

void engineer_arm_init()
{
    dr16_drv =  pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16);
    dr16_drv->init();
    dr16_drv->enable();

    axis1_motor = new pyro::dm_motor_drv_t(0x7, 0x6, pyro::can_hub_t::can1);
    axis1_motor->set_position_range(-pyro::PI, pyro::PI);
    axis1_motor->set_rotate_range(-10, 10); 
    axis1_motor->set_torque_range(-28, 28);

    axis2_motor = new pyro::dm_motor_drv_t(0x1, 0x0, pyro::can_hub_t::can1);
    axis2_motor->set_position_range(-pyro::PI, pyro::PI);
    axis2_motor->set_rotate_range(-45, 45); 
    axis2_motor->set_torque_range(-54, 54);

    axis3_motor = new pyro::dm_motor_drv_t(0x3, 0x2, pyro::can_hub_t::can1);
    axis3_motor->set_position_range(-pyro::PI, pyro::PI);
    axis3_motor->set_rotate_range(-20, 20); 
    axis3_motor->set_torque_range(-10, 10);

    axis4_motor = new pyro::dm_motor_drv_t(0x5, 0x4, pyro::can_hub_t::can2);
    axis4_motor->set_position_range(-pyro::PI, pyro::PI);
    axis4_motor->set_rotate_range(-20, 20); 
    axis4_motor->set_torque_range(-10, 10);

    axis5_motor = new pyro::dm_motor_drv_t(0x8, 0x9, pyro::can_hub_t::can2);
    axis5_motor->set_position_range(-pyro::PI, pyro::PI);
    axis5_motor->set_rotate_range(-30, 30); 
    axis5_motor->set_torque_range(-10, 10);

    axis6_motor =  new pyro::dji_gm_6020_motor_drv_t(pyro::dji_motor_tx_frame_t::register_id_t::id_1,pyro::can_hub_t::can2);

    end_motor = new pyro::dji_m2006_motor_drv_t(pyro::dji_motor_tx_frame_t::register_id_t::id_1,pyro::can_hub_t::can2);
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
}

const pyro::dr16_drv_t::dr16_ctrl_t *rc_data;
void engineer_arm_set_control()
{
    rc_data = static_cast<const pyro::dr16_drv_t::dr16_ctrl_t *>(dr16_drv->read());

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
        engineer_arm_zeroforce();
        vTaskDelay(5);
    }
}

