#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "pyro_uart_drv.h"
#include "pyro_core_dma_heap.h"
#include "string.h"
#include "pyro_powermeter.h"
#include "pyro_databoard.h"

extern pyro::databoard *global_databoard;
pyro::uart_drv_t *VOFA_uart;

class VOFA_Host
{
    private:
    pyro::uart_drv_t *_uart;
    float _justfloatbuffer[16];
    uint8_t _add_num;
    uint8_t *_send_buffer;
    public:
    VOFA_Host(pyro::uart_drv_t *uart):_uart(uart){
        _add_num = 0;
    }
    ~VOFA_Host(){}
    void addItem(float item)
    {
        _justfloatbuffer[_add_num] = item;
        _add_num++;
    }   
    void reset()
    {
        _add_num = 0;
    }
    void render_and_send()
    {
        static const uint8_t tail[4] = {0x00,0x00,0x80,0x7f};
        if(_send_buffer != nullptr)
        {
            vPortDmaFree(_send_buffer);
        }
        _send_buffer = (uint8_t *)pvPortDmaMalloc(sizeof(float) * (_add_num+1));
        memcpy(_send_buffer,_justfloatbuffer,sizeof(float)*_add_num);
        memcpy(_send_buffer+(_add_num)*sizeof(float),tail,sizeof(tail));
        _uart->write(_send_buffer,(_add_num+1)*sizeof(float));
    }
};
VOFA_Host *vofa;
pyro::powermeter_drv_t *powermeter;
pyro::powermeter_data powermeter_data;

static uint32_t motor_torque_topic_id,motor_rotate_topic_id;
extern "C" void VOFA_Thread(void *argument)
{
    vTaskDelay(500);
    motor_torque_topic_id = global_databoard->get_topic_id("motor_torque");
    motor_rotate_topic_id = global_databoard->get_topic_id("motor_rotate");
    VOFA_uart = pyro::uart_drv_t::get_instance(pyro::uart_drv_t::which_uart::uart10);
    vofa = new VOFA_Host(VOFA_uart);
    powermeter = new pyro::powermeter_drv_t(0x212,pyro::can_hub_t::can2);
    powermeter->init();
    for(;;)
    {
        uint32_t timestamp;
        powermeter->get_data(powermeter_data);
        float motor_torque,motor_rotate;
        global_databoard->read(motor_torque_topic_id,(pyro::genenral_data_t*)&(motor_torque),timestamp);
        global_databoard->read(motor_rotate_topic_id,(pyro::genenral_data_t*)&(motor_rotate),timestamp);
        vofa->reset();
        vofa->addItem(powermeter_data.power);
        vofa->addItem(motor_torque);
        vofa->addItem(motor_rotate);
        vofa->render_and_send();
        vTaskDelay(1);
    }

}