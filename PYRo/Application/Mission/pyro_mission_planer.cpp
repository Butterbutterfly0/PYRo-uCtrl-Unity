#include "cmsis_os.h"

extern "C"
{
    extern void pyro_init_thread(void *argument);
    extern void techcore_ctrl_task(void *argument);
    extern void VOFA_app_thread(void *argument);

    void start_mission_planer_task(void const *argument)
    {
        xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
                    osDelay(100);
        xTaskCreate(techcore_ctrl_task, "pyro_techcore_ctrl_task", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        xTaskCreate(VOFA_app_thread, "VOFA_app_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}