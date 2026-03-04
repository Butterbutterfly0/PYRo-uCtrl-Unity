#ifndef __ARM_TRANSITION_COMPONENT_H__
#define __ARM_TRANSITION_COMPONENT_H__

#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"

typedef enum
{
    Not_transition,
    Transition_start,
    Transition_running,
    Transition_end
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

void axis_transition_init(transition_param_t* param,float transition_period,float start_angle,float end_angle);

void arm_transition_init(arm_transition_t* param, float transition_period, float start_angle[6],float end_angle[6]);

void arm_transition_update(arm_transition_t* param);

class value_interpolation_t
{
    public:
        value_interpolation_t();
        ~value_interpolation_t();
        void init(float transition_period,float start_value,float end_value);
        void interpolation_update(float xdata[3]);
        float get_transition_interpolation_value() const;
    private:
        float _interpolation_period;
        float _interpolation_start_value;
        float _interpolation_end_value;
        float _interpolation_coefficient[4];
        float _interpolation_value;
};


class motion_transition_t
{
    public:
        motion_transition_t();
        ~motion_transition_t();
        void init(float transition_period, float start_angle[6],float end_angle[6]);
        void interpolation_update();
        void get_transition_interpolation_value(float xdata[6]);
        float* get_transition_interpolation_value();
        float get_transition_current_period() const;
        float get_transition_total_period() const;
        bool transition_timeout() const;
    private:
        value_interpolation_t _axis_transition[6];
        uint32_t _transition_start_Tick;
        float _transition_total_period;
        float _transition_current_period;
        float _interpolation_value[6];
};

#endif