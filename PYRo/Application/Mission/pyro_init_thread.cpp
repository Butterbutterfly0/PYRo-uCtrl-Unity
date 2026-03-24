#include "pyro_can_drv.h"
#include "pyro_rc_hub.h"
#include "pyro_dwt_drv.h"
#include "pyro_databoard.h"

#include "tim.h"

extern "C"
{
    pyro::can_drv_t *can1_drv;
    pyro::can_drv_t *can2_drv;
    pyro::can_drv_t *can3_drv;

    pyro::databoard *global_databoard;

    void pyro_init_thread(void *argument)
    {
        pyro::dwt_drv_t::init(480); // Initialize DWT at 480 MHz

        pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart1)
            ->enable_rx_dma();
        pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart5)
            ->enable_rx_dma();
        pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart7)
            ->enable_rx_dma();
        pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart10)
            ->enable_rx_dma();

        pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->enable();
        pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03)->enable();

        pyro::can_hub_t::get_instance();
        can1_drv = new pyro::can_drv_t(&hfdcan1);
        can2_drv = new pyro::can_drv_t(&hfdcan2);
        can3_drv = new pyro::can_drv_t(&hfdcan3);
        can1_drv->init();
        can2_drv->init();
        can3_drv->init();
        can1_drv->start();
        can2_drv->start();
        can3_drv->start();

        global_databoard = new pyro::databoard();

        global_databoard->create_topic("selfcontrol axis1",pyro::data_type_t::FLOAT);
        global_databoard->create_topic("selfcontrol axis2",pyro::data_type_t::FLOAT);
        global_databoard->create_topic("selfcontrol axis3",pyro::data_type_t::FLOAT);
        global_databoard->create_topic("selfcontrol axis4",pyro::data_type_t::FLOAT);
        global_databoard->create_topic("selfcontrol axis5",pyro::data_type_t::FLOAT);
        global_databoard->create_topic("selfcontrol axis6",pyro::data_type_t::FLOAT);

        global_databoard->create_topic("axis1_current_pos",pyro::data_type_t::FLOAT);
        global_databoard->create_topic("axis2_current_pos",pyro::data_type_t::FLOAT);
        global_databoard->create_topic("axis3_current_pos",pyro::data_type_t::FLOAT);
        global_databoard->create_topic("axis4_current_pos",pyro::data_type_t::FLOAT);
        global_databoard->create_topic("axis5_current_pos",pyro::data_type_t::FLOAT);
        global_databoard->create_topic("axis6_current_pos",pyro::data_type_t::FLOAT);

        global_databoard->create_topic("axis1_target_pos",pyro::data_type_t::FLOAT);
        global_databoard->create_topic("axis2_target_pos",pyro::data_type_t::FLOAT);
        global_databoard->create_topic("axis3_target_pos",pyro::data_type_t::FLOAT);
        global_databoard->create_topic("axis4_target_pos",pyro::data_type_t::FLOAT);
        global_databoard->create_topic("axis5_target_pos",pyro::data_type_t::FLOAT);
        global_databoard->create_topic("axis6_target_pos",pyro::data_type_t::FLOAT);

        global_databoard->create_topic("zero_force",pyro::data_type_t::UNSIGNED_INT);
        global_databoard->create_topic("magazine_angle",pyro::data_type_t::FLOAT);

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

        HAL_TIM_Base_Start(&htim2);
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 499);

        vTaskDelete(nullptr);
    }
}