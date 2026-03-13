#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "pyro_databoard.h"
#include "pyro_uart_drv.h"
#include <string.h>
#include "queue.h"

extern "C"
{
    #include "CRC8_CRC16.h"
}


typedef struct __attribute__((packed))
{
    uint8_t SOF;
    uint16_t dataLenth;
    uint8_t  seq;
    uint8_t  crc8;
}
tFrameHeader;

typedef struct __attribute__((packed))
{
    tFrameHeader frame_header;
    uint16_t cmd_id;
    float data[7];
    uint8_t used_data[2];
    uint16_t crc16;
}
referee_datalink_frame_t;

referee_datalink_frame_t self_control_frame;


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
    if( len == sizeof(referee_datalink_frame_t) )
    {
        xQueueSendFromISR(self_control_queue, buf,  NULL);
        return true;
    }
    return false;
}

static uint32_t selfcontrol_axis1_id,selfcontrol_axis2_id,selfcontrol_axis3_id,selfcontrol_axis4_id,selfcontrol_axis5_id,selfcontrol_axis6_id;

extern "C" void self_control_mission(void* args)
{
    while(global_databoard == nullptr)
    {
        vTaskDelay(1);
    }

    self_control_queue = xQueueCreate(10, sizeof(referee_datalink_frame_t));

    self_control_uart_drv = pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart1);
    self_control_uart_drv->add_rx_event_callback(self_control_callback, 2);

    selfcontrol_axis1_id = global_databoard->get_topic_id("selfcontrol axis1");
    selfcontrol_axis2_id = global_databoard->get_topic_id("selfcontrol axis2");
    selfcontrol_axis3_id = global_databoard->get_topic_id("selfcontrol axis3");
    selfcontrol_axis4_id = global_databoard->get_topic_id("selfcontrol axis4");
    selfcontrol_axis5_id = global_databoard->get_topic_id("selfcontrol axis5");
    selfcontrol_axis6_id = global_databoard->get_topic_id("selfcontrol axis6");

    for(;;)
    {   
        xQueueReceive(self_control_queue, self_control_buf,  portMAX_DELAY);
        // uint16_t crc = crc16_append(((uint8_t*)&self_control_buf)+2, sizeof(referee_datalink_frame_t)-4);
        if( verify_CRC8_check_sum(self_control_buf,sizeof(tFrameHeader))&& verify_CRC16_check_sum(self_control_buf,sizeof(referee_datalink_frame_t)) && ((referee_datalink_frame_t*)self_control_buf)->frame_header.SOF == 0xA5 && ((referee_datalink_frame_t*)self_control_buf)->cmd_id == 0x302 && ((referee_datalink_frame_t*)self_control_buf)->frame_header.dataLenth == 30 )
        {
            memcpy(&self_control_frame, self_control_buf, sizeof(referee_datalink_frame_t));
            float temp_f;
            temp_f = ((float)self_control_frame.data[0]);
            global_databoard->write_topic(selfcontrol_axis1_id,*((pyro::genenral_data_t*)&(temp_f)));
            temp_f = ((float)self_control_frame.data[1]);
            global_databoard->write_topic(selfcontrol_axis2_id,*((pyro::genenral_data_t*)&(temp_f)));
            temp_f = ((float)self_control_frame.data[2]);
            global_databoard->write_topic(selfcontrol_axis3_id,*((pyro::genenral_data_t*)&(temp_f)));
            temp_f = ((float)self_control_frame.data[3]);
            global_databoard->write_topic(selfcontrol_axis4_id,*((pyro::genenral_data_t*)&(temp_f)));
            temp_f = ((float)self_control_frame.data[4]);
            global_databoard->write_topic(selfcontrol_axis5_id,*((pyro::genenral_data_t*)&(temp_f)));
            temp_f = ((float)self_control_frame.data[5]);
            global_databoard->write_topic(selfcontrol_axis6_id,*((pyro::genenral_data_t*)&(temp_f)));
        }
        
        vTaskDelay(10);
    }
}