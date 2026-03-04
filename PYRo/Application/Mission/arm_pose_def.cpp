#include "arm_pose_def.h"

float arm_reset_pose[6] = {0.0, 1.0, 1.36, 0, 0.55, 0.0};

float arm_normal_pose[6] = {0.0, 0.004718, 0.6014, 0.0, 0, 0.0};

float arm_grip_energy_unit_0_motion[][7]=
{
    {1  , 0 , 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.5, 0.5 , 0.0, 0.0, 0.0, 0.0, 0.0},
};

uint32_t arm_grip_energy_unit_0_motion_stage_num = sizeof(arm_grip_energy_unit_0_motion)/sizeof(float)/7;

float arm_grip_energy_unit_60_motion[][7]=
{
    {1  , 0 , 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.5, 0.5 , 0.0, 0.0, 0.0, 0.0, 0.0},
};

uint32_t arm_grip_energy_unit_60_motion_stage_num = sizeof(arm_grip_energy_unit_60_motion)/sizeof(float)/7;

float arm_grip_energy_unit_120_motion[][7]=
{
    {1  , 0 , 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.5, 0.5 , 0.0, 0.0, 0.0, 0.0, 0.0},
};

uint32_t arm_grip_energy_unit_120_motion_stage_num = sizeof(arm_grip_energy_unit_120_motion)/sizeof(float)/7;

float arm_grip_energy_unit_180_motion[][7]=
{
    {1  , 0 , 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.5, 0.5 , 0.0, 0.0, 0.0, 0.0, 0.0},
};

uint32_t arm_grip_energy_unit_180_motion_stage_num = sizeof(arm_grip_energy_unit_180_motion)/sizeof(float)/7;

float arm_grip_energy_unit_240_motion[][7]=
{
    {1  , 0 , 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.5, 0.5 , 0.0, 0.0, 0.0, 0.0, 0.0},
};

uint32_t arm_grip_energy_unit_240_motion_stage_num = sizeof(arm_grip_energy_unit_240_motion)/sizeof(float)/7;

float arm_grip_energy_unit_300_motion[][7]=
{
    {1  , 0 , 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.5, 0.5 , 0.0, 0.0, 0.0, 0.0, 0.0},
};

uint32_t arm_grip_energy_unit_300_motion_stage_num = sizeof(arm_grip_energy_unit_300_motion)/sizeof(float)/7;
