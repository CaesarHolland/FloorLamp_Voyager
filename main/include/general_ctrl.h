/*
    总控制器模块，提供以下功能函数：
    1. 提供一个基于事件循环的可以接收参数来调节指定物理量的函数
*/
#ifndef __GENERAL_CTRL_H
#define __GENERAL_CTRL_H
#include "floorlamp.h"
#include "driver/ledc.h"

#define DUTY_THRESHOLD          CONFIG_DUTY_THRESHOLD
#define DUTY_MAX_THRESHOLD      CONFIG_DUTY_MAX_THRESHOLD

/*
    @var global varibles
*/
extern esp_event_loop_handle_t  g_controller_loop_handler;
extern esp_event_base_t         DEVICE_AD;
extern esp_event_base_t         DEVICE_MODE;


typedef struct
{
    int public_duty_ire;
    int private_duty_ire;
    int zen_duty_ire;
} DUTY_IRE_t;

typedef struct
{
    int public_duty;
    int private_duty;
    int zen_duty;
    int mode;
    int human_detect_flag;
    int human_detect_distance;
    int env_light_detect_flag;
    int frequency;
    int switch_status;
}DEVICE_Info_t;

typedef enum
{
    LM_IRE_AD=0,
    FRE_AD,
    ORDINARY_MODE,
    ZEN_MODE,
    CANDLE_MODE,
    HUMAN_DETECT,
    ENV_LIGHT_DETECT,
    DISTANCE_AD,
    ENV_LM_AD,
    SWITCH_AD,
};

/*
    @brief:
*/
void controller_init(void);

/*
    @brief:
*/
// void ledc_init(void);

/*
    @brief: 该函数是编码器初始化函数；需要注意两点：
            1. 编码器感受到变化之后不会立刻更新值，会延迟500ms，然后读取当前值更新，更新方法为发布【LM_AD】事件。
            2. 当当前亮度值为0也就是灯光关闭时，这时正向拧编码器其不会当成亮度调节信号，而是将灯光初始化到上一次
               关闭时的亮度，并延迟2000ms，延迟后再调节旋钮的话就会被认为是改变亮度了。
*/
// void encoder_init(void);

/*
    @brief: 总控制器任务，接收调节事件，进行相应调节
*/
// void general_controller_task(void *handler_arg, esp_event_base_t base, int32_t id, void *event_data);


/*
    @Get&Set functions:
*/
DEVICE_Info_t get_device_status(void);
DUTY_IRE_t get_lumen_duty(void);
int get_mode(void);
int get_frequency(void);
// void get_distance(void);
// void get_human_detect_status(void);
// void get_env_light_detect_status(void);

/*
    reset device to default value
*/
void resetDevice(void);



#endif
