#ifndef __ENGINEER_ARM_PLANNING_MISSION_H__
#define __ENGINEER_ARM_PLANNING_MISSION_H__

#include "semphr.h"

typedef enum
{
    ZERO_FORCE,
    POSITION_CONTROL
}
control_mode_t;

typedef struct 
{
    control_mode_t control_mode;
    float axis_current_pos[6];
    float axis_target_pos[6];
    float end_target_torque;
}
control_target_param_t;

typedef enum
{
    RESET_POSE_TRANSITION,
    RESET_POSE,
    NORMAL_POSE_TRANSITION,
    NORMAL_POSE,
    SELF_CONTROL_TRANSITION,
    SELF_CONTROL,
    MOTION_Start,
    MOTION,
    
}
specific_control_mode_t;

extern control_target_param_t *control_target_param;
extern SemaphoreHandle_t rc_planning_sem;

#endif