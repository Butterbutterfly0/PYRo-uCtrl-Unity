#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "engineer_arm_planning_mission.h"
#include "pyro_rc_hub.h"
#include "timers.h"
#include "arm_pose_def.h"
#include <string.h>

control_target_param_t *control_target_param;
SemaphoreHandle_t rc_planning_sem;
control_target_param_t *control_target_param_buffer ;

typedef enum
{
    RESET_POSE_TRANSITION,
    RESET_POSE,
    NORMAL_POSE_TRANSITION,
    NORMAL_POSE,
    SELF_CONTROL_TRANSITION,
    SELF_CONTROL
}
specific_control_mode_t;

typedef enum
{
    Not_transition,
    Transition_start,
    Transition_running
}
transition_state_t;

typedef struct
{ 
    float transition_start_pos;
    float transition_end_pos;
    float transition_coefficient[4];
    float transition_target_pos;
    float transition_current_pos;
}
transition_param_t;

typedef struct
{
    uint32_t transition_start_Tick;
    float transition_total_time;
    float transition_current_time;

    transition_param_t axis_transition_param[6];
}
arm_transition_t;


specific_control_mode_t specific_control_mode = RESET_POSE;
transition_state_t transition_state = Not_transition;
arm_transition_t arm_transition_param;
TimerHandle_t transition_timer;

float end_target_torque = 0.0f;


static pyro::rc_drv_t* dr16_drv;
static const  pyro::dr16_drv_t::dr16_ctrl_t *rc_data;
void arm_rc_resolve()
{
    static pyro::dr16_drv_t::sw_state_t last_sw_r_state = pyro::dr16_drv_t::sw_state_t::SW_UP;

    rc_data = static_cast<const pyro::dr16_drv_t::dr16_ctrl_t *>(dr16_drv->read());

    if(rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_UP && last_sw_r_state != pyro::dr16_drv_t::sw_state_t::SW_UP)
    {
        specific_control_mode = RESET_POSE_TRANSITION;
        transition_state = Transition_start;
    }
    // else if(rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_UP)
    // {
    //     specific_control_mode = RESET_POSE;
    // }
    else if(rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_MID && last_sw_r_state != pyro::dr16_drv_t::sw_state_t::SW_MID)
    {
        specific_control_mode = NORMAL_POSE_TRANSITION;
        transition_state = Transition_start;
    }
    // else if(rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_MID)
    // {
    //     specific_control_mode = NORMAL_POSE;
    // }
    else if(rc_data->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_DOWN && last_sw_r_state != pyro::dr16_drv_t::sw_state_t::SW_DOWN)
    {
        specific_control_mode = SELF_CONTROL_TRANSITION;
        transition_state = Transition_start;
    }


    end_target_torque = rc_data->rc.wheel*2;
    

    last_sw_r_state = rc_data->rc.s_r.state;
}

void axis_transition_init(transition_param_t* param,float transition_period,float start_angle,float end_angle)
{
    param->transition_start_pos = start_angle;
    param->transition_end_pos = end_angle;
    param->transition_current_pos = start_angle;
    param->transition_target_pos = start_angle;
    param->transition_coefficient[0] = start_angle;
    param->transition_coefficient[1] = 0.0f;
    param->transition_coefficient[2] = 3*(end_angle-start_angle)/transition_period/transition_period;
    param->transition_coefficient[3] = 2*(start_angle-end_angle)/transition_period/transition_period/transition_period;
}

void arm_transition_init(arm_transition_t* param, float transition_period, float start_angle[6],float end_angle[6])
{
    param->transition_start_Tick = xTaskGetTickCount();
    param->transition_total_time = transition_period;
    param->transition_current_time = 0;
    for(int i=0;i<6;i++)
    {
        axis_transition_init(&param->axis_transition_param[i], transition_period, start_angle[i],end_angle[i]);
    }
}

void arm_transition_update(arm_transition_t* param)
{ 
    param->transition_current_time = (xTaskGetTickCount()-arm_transition_param.transition_start_Tick)/1000.0f;
    float _t = param->transition_current_time;
    float _t2 = _t*_t;
    float _t3 = _t2*_t;
    for(int i=0;i<6;i++)
    {
        param->axis_transition_param[i].transition_target_pos = param->axis_transition_param[i].transition_coefficient[0]+
        param->axis_transition_param[i].transition_coefficient[1]*_t+
        param->axis_transition_param[i].transition_coefficient[2]*_t2+
        param->axis_transition_param[i].transition_coefficient[3]*_t3;
    }
}

void arm_transition()
{
    float time_spend = 3;
    if(transition_state == Transition_start)
    {
        switch(specific_control_mode)
        {
            case RESET_POSE_TRANSITION:
            transition_state = Transition_running;
            time_spend = 3;
            arm_transition_init(&arm_transition_param, time_spend, control_target_param_buffer->axis_current_pos, arm_reset_pose);
            break;
            case NORMAL_POSE_TRANSITION:
            transition_state = Transition_running;
            time_spend = 1;
            arm_transition_init(&arm_transition_param, time_spend, control_target_param_buffer->axis_current_pos, arm_normal_pose);
            break;
        }
    }
    else if(transition_state == Transition_running)
    {
        arm_transition_update(&arm_transition_param);
        if(arm_transition_param.transition_current_time>=arm_transition_param.transition_total_time)
        {
            transition_state = Not_transition;
            switch(specific_control_mode)
            {
                case RESET_POSE_TRANSITION:
                specific_control_mode = RESET_POSE;
                break;
                case NORMAL_POSE_TRANSITION:
                specific_control_mode = NORMAL_POSE;
                break;
            }
        }
    }
}

void arm_planning_application()
{
    switch(specific_control_mode)
    {
        case RESET_POSE:
        control_target_param->control_mode = ZERO_FORCE;
        for(int i=0;i<6;i++)
        {
            control_target_param->axis_target_pos[i] = arm_reset_pose[i];
        }
        break;
        case NORMAL_POSE:
        control_target_param->control_mode = POSITION_CONTROL;
        for(int i=0;i<6;i++)
        {
            control_target_param->axis_target_pos[i] = arm_normal_pose[i];
        }
        break;
        case RESET_POSE_TRANSITION:
        case NORMAL_POSE_TRANSITION:
        control_target_param->control_mode = POSITION_CONTROL;
        for(int i=0;i<6;i++)
        {
            control_target_param->axis_target_pos[i] = arm_transition_param.axis_transition_param[i].transition_target_pos;
        }
        break;

    }
    control_target_param->end_target_torque = end_target_torque;
}


extern "C" void engineer_arm_planning_mission(void* args)
{
    control_target_param = new control_target_param_t;
    control_target_param->control_mode = ZERO_FORCE;
    control_target_param->end_target_torque = 0;
    for(int i=0;i<6;i++)
    {
        control_target_param->axis_target_pos[i]=0;
        control_target_param->axis_current_pos[i]=0;
    }
    control_target_param_buffer = new control_target_param_t;
    memcpy(control_target_param_buffer,control_target_param,sizeof(control_target_param_t));
    rc_planning_sem = xSemaphoreCreateMutex();  

    dr16_drv =  pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16);

    for(;;)
    {
        xSemaphoreTake(rc_planning_sem, portMAX_DELAY);
        memcpy(control_target_param_buffer->axis_current_pos,control_target_param->axis_current_pos,sizeof(float)*6);
        xSemaphoreGive(rc_planning_sem);
        arm_rc_resolve();
        arm_transition();
        xSemaphoreTake(rc_planning_sem, portMAX_DELAY);
        arm_planning_application();
        xSemaphoreGive(rc_planning_sem);
        vTaskDelay(1);
    }
}
