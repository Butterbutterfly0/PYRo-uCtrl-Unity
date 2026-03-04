#ifndef __ARM_FIXED_MOTION_HUB_H__
#define __ARM_FIXED_MOTION_HUB_H__

#include <stdint.h>
#include "arm_transition_component.h"
#include <string.h>
#include "arm_pose_def.h"

class arm_fixed_motion_t
{
    public:
        arm_fixed_motion_t();
        ~arm_fixed_motion_t();
        void bind(float motion_slice[][7],uint32_t stage_num);
        void reset();
        void start_motion();
        bool update_motion(float current_period);
        void get_motion_slice(float dst[7]);
        transition_state_t get_motion_transition_state();
    private:
        float (*_motion_slice)[7];
        uint32_t _motion_total_stage;
        uint32_t _motion_current_stage;
        transition_state_t _motion_transition_state;
        float _now_slice[7];
};

class arm_fixed_motion_group_t
{
    public:
        arm_fixed_motion_group_t();
        ~arm_fixed_motion_group_t();
        void select_motion(arm_motion_e motion);
        void start_motion(float current_position[6]);
        bool update_motion(float current_period);
        void get_motion_slice(float xdata[7]);
        void add_motion(arm_motion_e motion_id,float motion_slice[][7],uint32_t stage_num);
        bool motion_over();
    private:
        arm_motion_e _now_motion_id;
        arm_fixed_motion_t * _now_motion;

        arm_fixed_motion_t _motion_list[16];
        arm_motion_e _motion_list_name[16];
        uint32_t _motion_list_num;
        motion_transition_t _motion_transition;
};

#endif