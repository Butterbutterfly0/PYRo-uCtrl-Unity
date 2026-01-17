#include "pyro_core_config.h"
#include "cmsis_os.h"

#include "pyro_dm_motor_drv.h"
#include "pyro_dji_motor_drv.h"

extern "C" void engineer_arm_mission(void* args);

pyro::dm_motor_drv_t *axis1_motor;

pyro::dm_motor_drv_t *axis2_motor;

pyro::dm_motor_drv_t *axis3_motor;

pyro::dm_motor_drv_t *axis4_motor;

pyro::dm_motor_drv_t *axis5_motor;

pyro::dji_gm_6020_motor_drv_t *axis6_motor;

pyro::dji_m2006_motor_drv_t *end_motor;

float motor_current_pos[7]={};
float motor_current_rot[7]={};
float motor_current_torque[7]={};

void engineer_arm_mission(void* args)
{
    osDelay(10);

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

        axis1_motor->send_torque(0);
        axis2_motor->send_torque(0);
        axis3_motor->send_torque(0);
        axis4_motor->send_torque(0);
        axis5_motor->send_torque(0);
        axis6_motor->send_torque(0);
        end_motor->send_torque(0);

        vTaskDelay(5);
    }


}

