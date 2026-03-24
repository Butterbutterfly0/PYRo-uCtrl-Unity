#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "pyro_databoard.h"
#include "pyro_rc_hub.h"
#include "string.h"
#include "pyro_uart_drv.h"
#include "usart.h"

extern pyro::databoard* global_databoard;
static pyro::rc_drv_t* dr16_drv;

typedef struct __attribute__((packed))
{
    uint16_t frame_header;
    uint8_t sw_l;
    uint8_t sw_r;
    int16_t chassis_vx;
    int16_t chassis_vy;
    int16_t chassis_wz;
    int16_t rc_ch_ry;
    uint8_t zero_force;
    float magazine_angle;
    uint16_t crc16;
} upper_board_tx_frame_t;

upper_board_tx_frame_t upper_board_tx_frame;
__attribute__((section(".dma_heap"))) uint8_t upper_board_tx_buffer[sizeof(upper_board_tx_frame_t)+1];

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

const static  pyro::dr16_drv_t::dr16_ctrl_t *rc_data;

uint32_t zero_force_id = 0;
uint32_t magazine_angle_id = 0;

void upper_board_tx_frame_update()
{
    rc_data = static_cast<const pyro::dr16_drv_t::dr16_ctrl_t *>(dr16_drv->read()); 
    uint32_t zero_force = 0;
    uint32_t timestamp = 0;
    float magazine_angle = 0;
    upper_board_tx_frame.sw_l = (uint8_t)(rc_data->rc.s_l.state);
    upper_board_tx_frame.sw_r = (uint8_t)(rc_data->rc.s_r.state);
    float speed_gain = rc_data->key.ctrl.state ? 0.05 : 1;
    if(rc_data->key.a.state == 1 || rc_data->key.d.state == 1)
    {
        upper_board_tx_frame.chassis_vx = (int16_t)(rc_data->key.d.state - rc_data->key.a.state)*1000*speed_gain;
    }
    else
    {
        upper_board_tx_frame.chassis_vx = (int16_t)(rc_data->rc.ch_lx*1000);
    }

    if(rc_data->key.w.state == 1 || rc_data->key.s.state == 1)
    {
        upper_board_tx_frame.chassis_vy = (int16_t)(rc_data->key.w.state - rc_data->key.s.state)*1000*speed_gain;
    }
    else
    {
        upper_board_tx_frame.chassis_vy = (int16_t)(rc_data->rc.ch_ly*1000);
    }
    
    if(rc_data->key.q.state == 1 || rc_data->key.e.state == 1)
    {
        upper_board_tx_frame.chassis_wz = (int16_t)((rc_data->key.e.state-rc_data->key.q.state)*1000*speed_gain);
    }
    else
    {
         upper_board_tx_frame.chassis_wz = (int16_t)(rc_data->rc.ch_rx*1000);
    }
   
    upper_board_tx_frame.rc_ch_ry = (int16_t)(rc_data->rc.ch_ry*1000);
    global_databoard->read(zero_force_id,(pyro::genenral_data_t*)&zero_force,timestamp);
    upper_board_tx_frame.zero_force = (uint8_t)zero_force;
    global_databoard->read(magazine_angle_id,(pyro::genenral_data_t*)(&upper_board_tx_frame.magazine_angle),timestamp);
    upper_board_tx_frame.crc16 = crc16_append(((uint8_t*)&upper_board_tx_frame)+2, sizeof(upper_board_tx_frame_t)-4);
    memcpy(upper_board_tx_buffer, &upper_board_tx_frame, sizeof(upper_board_tx_frame_t));
}

pyro::uart_drv_t* interboard_communication_uart_drv;
extern "C" void interboard_communication_mission(void* args)
{
    // osDelay(10);
    dr16_drv = pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16);
    while(global_databoard == nullptr)
    {
        vTaskDelay(1);
    }

    zero_force_id = global_databoard->get_topic_id("zero_force");
    magazine_angle_id = global_databoard->get_topic_id("magazine_angle");

    upper_board_tx_frame.frame_header = 0xffA5;
    interboard_communication_uart_drv=pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart7);

    for(;;)
    {
        upper_board_tx_frame_update();
        interboard_communication_uart_drv->write(upper_board_tx_buffer, sizeof(upper_board_tx_frame_t));
        vTaskDelay(10);
    }
    
}