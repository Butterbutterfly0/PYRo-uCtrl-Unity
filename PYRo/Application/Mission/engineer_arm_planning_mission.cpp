#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "engineer_arm_planning_mission.h"
#include "pyro_rc_hub.h"
#include "timers.h"
#include <string.h>
#include "pyro_databoard.h"

#include "arm_pose_def.h"
#include "arm_transition_component.h"
#include "arm_fixed_motion_hub.h"
#include "arm_rc_command.h"
#include "arm_self_control_command.h"
#include "arm_self_control_command.h"



extern pyro::databoard* global_databoard;

control_target_param_t *control_target_param;
SemaphoreHandle_t rc_planning_sem;
control_target_param_t *control_target_param_buffer ;

specific_control_mode_t specific_control_mode = RESET_POSE;
transition_state_t transition_state = Not_transition;
arm_transition_t arm_transition_param;

float end_target_torque = 0.0f;


static pyro::rc_drv_t* dr16_drv;
static const  pyro::dr16_drv_t::dr16_ctrl_t *rc_data;


float self_control_pos[6] ={0.0,0.0,0.0,0.0,0.0,0.0};;

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
            case SELF_CONTROL_TRANSITION:
            transition_state = Transition_running;
            time_spend = 1;
            arm_transition_init(&arm_transition_param, time_spend, control_target_param_buffer->axis_current_pos, self_control_pos);
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
                case SELF_CONTROL_TRANSITION:
                specific_control_mode = SELF_CONTROL;
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
        case SELF_CONTROL:
        control_target_param->control_mode = POSITION_CONTROL;
        for(int i=0;i<6;i++)
        {
            control_target_param->axis_target_pos[i] = self_control_pos[i];
        }
        break;
        case RESET_POSE_TRANSITION:
        case NORMAL_POSE_TRANSITION:
        case SELF_CONTROL_TRANSITION:
        control_target_param->control_mode = POSITION_CONTROL;
        for(int i=0;i<6;i++)
        {
            control_target_param->axis_target_pos[i] = arm_transition_param.axis_transition_param[i].transition_target_pos;
        }
        break;

    }
    control_target_param->end_target_torque = end_target_torque;
}


class arm_planner_t
{
    public:
        void init();
        void update();
        void update_async();
        void transition_process();
        void fixed_motion_process();
        void planning_application();
        void application_async();
    private:
        float _axis_current_pos[6];

        specific_control_mode_t _specific_control_mode = RESET_POSE;
        transition_state_t _transition_state = Not_transition;
        arm_rc_command_t _arm_rc_command;
        arm_self_control_command _arm_self_control_command;
        motion_transition_t _motion_transition;
        arm_fixed_motion_group_t _arm_fixed_motion_group;

        user_command_t _user_command;
};

void arm_planner_t::init()
{
    _arm_rc_command.bind_dr16(pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16));
    while(global_databoard == nullptr)
    {
        vTaskDelay(1);
    }
    _arm_self_control_command.bind(global_databoard);
}

void arm_planner_t::update_async()
{
    xSemaphoreTake(rc_planning_sem, portMAX_DELAY);
    memcpy(control_target_param_buffer->axis_current_pos,control_target_param->axis_current_pos,sizeof(float)*6);
    xSemaphoreGive(rc_planning_sem);    
    memcpy(_axis_current_pos,control_target_param_buffer->axis_current_pos,sizeof(float)*6);
}
void arm_planner_t::update()
{
    update_async();

    _arm_rc_command.update(_specific_control_mode,_transition_state,_user_command);
    _arm_self_control_command.update();
}

void arm_planner_t::transition_process()
{
    float time_spend = 3;
    if(_transition_state == Transition_start)
    {
        switch(_specific_control_mode)
        {
            case RESET_POSE_TRANSITION:
            _transition_state = Transition_running;
            time_spend = 2;
            _motion_transition.init(time_spend,_axis_current_pos, arm_reset_pose);
            break;
            case NORMAL_POSE_TRANSITION:
            _transition_state = Transition_running;
            time_spend = 1;
            _motion_transition.init(time_spend,_axis_current_pos, arm_normal_pose);
            break;
            case SELF_CONTROL_TRANSITION:
            _transition_state = Transition_running;
            time_spend = 1;
            _motion_transition.init(time_spend,_axis_current_pos, self_control_pos);
            break;
        }
    }
    else if(_transition_state == Transition_running)
    {
        _motion_transition.interpolation_update();
        if(_motion_transition.transition_timeout())
        {
            _transition_state = Not_transition;
            switch(_specific_control_mode)
            {
                case RESET_POSE_TRANSITION:
                _specific_control_mode = RESET_POSE;
                break;
                case NORMAL_POSE_TRANSITION:
                _specific_control_mode = NORMAL_POSE;
                break;
                case SELF_CONTROL_TRANSITION:
                _specific_control_mode = SELF_CONTROL;
                break;
            }
        }
    }
}

void arm_planner_t::fixed_motion_process()
{
    float slice[7];
    if(_specific_control_mode == MOTION_Start)
    {
        _arm_fixed_motion_group.select_motion(_user_command.selected_motion);
        _arm_fixed_motion_group.start_motion(_axis_current_pos);
        _arm_fixed_motion_group.update_motion(0);
        _arm_fixed_motion_group.get_motion_slice(slice);
        _motion_transition.init(slice[0],_axis_current_pos,slice+1);
        _specific_control_mode = MOTION;
    }
    else if(_specific_control_mode == MOTION)
    {
        if(_arm_fixed_motion_group.update_motion(_motion_transition.get_transition_current_period()))
        {
            if(_arm_fixed_motion_group.motion_over())
            {
                _specific_control_mode = NORMAL_POSE_TRANSITION;
            }
            _arm_fixed_motion_group.get_motion_slice(slice);
            _motion_transition.init(slice[0],_axis_current_pos,slice+1);
        }
        else
        {
            _motion_transition.interpolation_update();
        }
    }
}

void arm_planner_t::planning_application()
{
   xSemaphoreTake(rc_planning_sem, portMAX_DELAY);
    application_async();
    xSemaphoreGive(rc_planning_sem);
}

void arm_planner_t::application_async()
{
    switch(_specific_control_mode)
    {
        case RESET_POSE:
        control_target_param->control_mode = ZERO_FORCE;
        // memcpy(control_target_param->axis_target_pos,arm_reset_pose,sizeof(float)*6);
        break;
        case NORMAL_POSE:
        control_target_param->control_mode = POSITION_CONTROL;
        memcpy(control_target_param->axis_target_pos,arm_normal_pose,sizeof(float)*6);
        break;
        case SELF_CONTROL:
        control_target_param->control_mode = POSITION_CONTROL;
        memcpy(control_target_param->axis_target_pos,_arm_self_control_command.get_self_control_command(),sizeof(float)*6);
        break;
        case RESET_POSE_TRANSITION:
        case NORMAL_POSE_TRANSITION:
        case SELF_CONTROL_TRANSITION:
        control_target_param->control_mode = POSITION_CONTROL;
        memcpy(control_target_param->axis_target_pos,_motion_transition.get_transition_interpolation_value(),sizeof(float)*6);
        break;
        default:
        control_target_param->control_mode = ZERO_FORCE;
    }
}

arm_planner_t arm_planner;
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
    
    arm_planner.init();

    for(;;)
    {
        arm_planner.update();
        arm_planner.fixed_motion_process();
        arm_planner.transition_process();
        arm_planner.planning_application();
        vTaskDelay(1);
    }
}
