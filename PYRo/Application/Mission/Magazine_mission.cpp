#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "pyro_databoard.h"

extern pyro::databoard* global_databoard;

static uint32_t 
rc_sw_l_topic_id,
rc_sw_r_topic_id;

uint32_t magazine_sw_l,magazine_sw_r;
bool magazine_rc_solve()
{
    uint32_t timestamp;
    pyro::topic::data_status_t rc_data_status;
    rc_data_status = global_databoard->read(rc_sw_l_topic_id,(pyro::genenral_data_t*)&(magazine_sw_l),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
        return false;
    rc_data_status = global_databoard->read(rc_sw_r_topic_id,(pyro::genenral_data_t*)&(magazine_sw_l),timestamp);
    if(rc_data_status == pyro::topic::DATA_ERROR)
        return false;
    return true;
}

extern "C" void magazine_mission(void *argument)
{
    while(global_databoard == nullptr)
    {
        vTaskDelay(1);
    }

    rc_sw_l_topic_id = global_databoard->get_topic_id("rc_sw_l");
    rc_sw_r_topic_id = global_databoard->get_topic_id("rc_sw_r");

    for(;;)
    {
        vTaskDelay(1);
    }
}