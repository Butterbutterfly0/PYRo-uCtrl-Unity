#ifndef __ARM_RC_COMMAND_H__
#define __ARM_RC_COMMAND_H__

#include "pyro_rc_hub.h"
#include "arm_transition_component.h"
#include "engineer_arm_planning_mission.h"
#include "arm_pose_def.h"


typedef struct{
    arm_motion_e selected_motion;
}
user_command_t;

// The class is used to individualize the rc logic part from the other part 
class arm_rc_command_t
{
    public:
        arm_rc_command_t();
        ~arm_rc_command_t();
        void bind_dr16(pyro::rc_drv_t* dr16_drv);
        void update(specific_control_mode_t& specific_control_mode,
            transition_state_t& transition_state,
        user_command_t& user_command);
    private:
        pyro::rc_drv_t* dr16_drv;
        const  pyro::dr16_drv_t::dr16_ctrl_t *_rc_data;
        
    };

#endif