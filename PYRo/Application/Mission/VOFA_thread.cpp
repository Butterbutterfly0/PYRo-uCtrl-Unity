#include "pyro_core_config.h"
#include "pyro_uart_drv.h"
#include "pyro_databoard.h"

extern pyro::databoard* global_databoard;
static uint32_t yaw_angle_topic_id,pitch_angle_topic_id,roll_angle_topic_id;
float angle_data[3];
uint32_t timestamp;
extern "C" void VOFA_app_thread(void *pvParameters)
{
    yaw_angle_topic_id = global_databoard->get_topic_id("yaw_axis_angle");
    pitch_angle_topic_id = global_databoard->get_topic_id("pitch_axis_angle");
    roll_angle_topic_id = global_databoard->get_topic_id("roll_axis_angle");
    // global_databoard->get_topic_id("pitch_axis_angle");
    // global_databoard->get_topic_id("roll_axis_angle");
    for(;;)
    {
        timestamp+=yaw_angle_topic_id;
        timestamp+=pitch_angle_topic_id;
        timestamp+=roll_angle_topic_id;
        global_databoard->read(yaw_angle_topic_id,(pyro::genenral_data_t*)(angle_data),timestamp);
        global_databoard->read(pitch_angle_topic_id,(pyro::genenral_data_t*)(angle_data+1),timestamp);
        global_databoard->read(roll_angle_topic_id,(pyro::genenral_data_t*)(angle_data+2),timestamp);

        vTaskDelay(10);
    }
}