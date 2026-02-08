#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "pyro_databoard.h"
#include "pyro_uart_drv.h"
#include <string.h>
#include "queue.h"

typedef struct __attribute__((packed))
{
    uint16_t frame_header;
    float axis1;
    float axis2;
    float axis3;
    float axis4;
    float axis5;
    float axis6;
    uint16_t crc16;
}
datalink_frame_t;

datalink_frame_t self_control_frame;


extern pyro::databoard* global_databoard;

pyro::uart_drv_t* self_control_uart_drv;

uint8_t self_control_buf[128];

QueueSetHandle_t self_control_queue;

static uint16_t crc16_append(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for(uint8_t i = 0; i < len; i++)
    {
        crc = ((crc^data[i])&0x00FF)|(crc&0xff00);
        for(uint8_t j = 0; j < 8; j++)
        {
           if(crc&1)
           {
                crc >>= 1;
                crc ^= 0xA001;
           }
           else
           {
                crc >>= 1;
           }
        }
    }
    return crc;
}

bool self_control_callback(uint8_t *buf, uint16_t len,BaseType_t xHigherPriorityTaskWoken)
{
    if( len != 0 )
    {
        xQueueSendFromISR(self_control_queue, buf,  NULL);
        return true;
    }
    return false;
}

extern "C" void self_control_mission(void* args)
{
    while(global_databoard == nullptr)
    {
        vTaskDelay(1);
    }

    self_control_queue = xQueueCreate(10, sizeof(datalink_frame_t));

    self_control_uart_drv = pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart10);
    self_control_uart_drv->add_rx_event_callback(self_control_callback, 2);

    for(;;)
    {   
        xQueueReceive(self_control_queue, self_control_buf,  portMAX_DELAY);
        uint16_t crc = crc16_append(((uint8_t*)&self_control_buf)+2, sizeof(datalink_frame_t)-4);
        if( crc == ((datalink_frame_t*)self_control_buf)->crc16 && ((datalink_frame_t*)self_control_buf)->frame_header == 0x55AA)
        {
            memcpy(&self_control_frame, self_control_buf, sizeof(datalink_frame_t));
        }
        
        vTaskDelay(10);
    }
}