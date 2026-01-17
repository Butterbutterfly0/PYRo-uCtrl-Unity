#include "cmsis_os.h"


extern "C" {
    extern void pyro_init_thread(void *argument);
    extern void engineer_arm_mission(void* args);

    void start_mission_planer_task(void const *argument)
    {
        xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelay(10);
        xTaskCreate(engineer_arm_mission, "engineer_arm_mission", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}