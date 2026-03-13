#ifndef __ARM_POSE_DEF_H__
#define __ARM_POSE_DEF_H__

#include <stdint.h>

extern float arm_reset_pose[6];
extern float arm_normal_pose[6];

typedef enum
{
    none_motion = -1,
    arm_grip_energy_unit_0 = 0,
    arm_grip_energy_unit_60,
    arm_grip_energy_unit_120,
    arm_grip_energy_unit_180,
    arm_grip_energy_unit_240,
    arm_grip_energy_unit_300,

    arm_push_energy_unit,
    arm_pop_energy_unit,


    arm_motion_max
}
arm_motion_e;

extern float arm_grip_energy_unit_0_motion[][8];
extern uint32_t arm_grip_energy_unit_0_motion_stage_num;
extern float arm_grip_energy_unit_60_motion[][8];
extern uint32_t arm_grip_energy_unit_60_motion_stage_num;
extern float arm_grip_energy_unit_120_motion[][8];
extern uint32_t arm_grip_energy_unit_120_motion_stage_num;
extern float arm_grip_energy_unit_180_motion[][8];
extern uint32_t arm_grip_energy_unit_180_motion_stage_num;
extern float arm_grip_energy_unit_240_motion[][8];
extern uint32_t arm_grip_energy_unit_240_motion_stage_num;
extern float arm_grip_energy_unit_300_motion[][8];
extern uint32_t arm_grip_energy_unit_300_motion_stage_num;
extern float arm_push_energy_unit_motion[][8];
extern uint32_t arm_push_energy_unit_motion_stage_num;
extern float arm_pop_energy_unit_motion[][8];
extern uint32_t arm_pop_energy_unit_motion_stage_num;

#endif