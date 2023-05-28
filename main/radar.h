#ifndef __RADAR_H
#define __RADAR_H
#include "floorlamp.h"

// IO PINS:
#define RADAR_TXD       CONFIG_RADAR_TXD
#define RADAR_RXD       CONFIG_RADAR_RXD
#define RADAR_OUT_IO    CONFIG_RADAR_OUT
#define RADAR_RTS       UART_PIN_NO_CHANGE
#define RADAR_CTS       UART_PIN_NO_CHANGE

#define RADAR_UART_NUM UART_NUM_1

#define RADAR_BUF_SIZE (1024)
#define RADAR_RD_BUF_SIZE (BUF_SIZE)

// RADAR CONTROL COMMAND CODES:
#define SET_DISTANCE_LEVEL          0x02
#define GET_DISTANCE_LEVEL          0x03
#define SET_DELAY_TIME              0x04
#define GET_DELAY_TIME              0x05
#define ENABLE_LIGHT_SENSING        0x16
#define GET_LIGHT_SENSING_STATUS    0x17
#define SET_LIGHT_THRESHOLD         0x06
#define GET_LIGHT_THRESHOLD         0x07
#define GET_CUR_LIGHT_VALUE         0x2e
#define ENABLE_DUL_TRIGGER          0x18
#define GET_DUL_TRIGGER_STATUS      0x19
#define SET_TRIGGER_LEVEL           0x30
#define GET_TRIGGER_LEVEL           0x31
#define SET_OUTPUT_MODE             0x1c
#define GET_OUTPUT_MODE             0x1d
#define GET_TRIGGER_STATUS          0x1f
// #define SET_LIGHTON_PWM_DUTY        0x0B
// #define GET_LIGHTON_PWM_DUTY        0x1e
// #define SET_LIGHTOFF_PWM_DUTY       0x2a
// #define GET_LIGHTOFF_PWM_DUTY       0x2b
#define ENABLE_GRADIENT             0x2c
#define GET_GRADIENT_STATUS         0x2d
#define ENABLE_RADAR                0xD1
#define GET_RADAR_STATUS            0xD0
#define SAVE_RADAR_CONFIG           0x08
#define GET_RADAR_SAVE_STATUS       0x09
#define ENABLE_DIRECT_OUTPUT        0xA
#define SYSTEM_RESET                0x13

/*
// RADAR CONTROL COMMAND PARAMETERS:
// SET_DISTANCE_LEVEL, COMMAND FORMAT:58 02 01 00 6A 00. The forth byte is the parameter, range is 0~31.
#define SET_DISTANCE_LEVEL_PARAM {0x58, 0x02, 0x01, 0x00, 0x6A, 0x00}
// SET_DISTANCE_LEVEL, RESPONSE FRAME FORMAT:59 02 01 00 5C 00.
#define SET_DISTANCE_LEVEL_RESP {0x59, 0x02, 0x01, 0x00, 0x5C, 0x00}
// GET_DISTANCE_LEVEL, COMMAND FORMAT:58 03 00 5B 00.
#define GET_DISTANCE_LEVEL_PARAM {0x58, 0x03, 0x00, 0x5B, 0x00}
// GET_DISTANCE_LEVEL, RESPONSE FRAME FORMAT:59 03 01 00 6C 00. The forth byte is the query result.
#define GET_DISTANCE_LEVEL_RESP {0x59, 0x03, 0x01, 0x00, 0x6C, 0x00}
// SET_DELAY_TIME, COMMAND FORMAT:58 04 02 00 00 00 00.
// define a HEX array as the parameter of LIGHT_SENSING: 58 16 01 01 70 00
#define LIGHT_SENSING_PARAM_OPEN {0x58, 0x16, 0x01, 0x01, 0x70, 0x00}       // open light sensing
// define a HEX array as the parameter of LIGHT_SENSING: 58 16 01 00 6F 00
#define LIGHT_SENSING_PARAM_CLOSE {0x58, 0x16, 0x01, 0x00, 0x6F, 0x00}      // close light sensing
*/

esp_err_t radar_init(int baudrate);

/*
    format of uart data:
    - Head(Byte_0): Control frame header, the value is 0x58
    - Command code(Byte_1): Command code, the value is defined in the structure of command_code
    - Parameter Length(Byte_2): Length of the parameter
    - Parameter(Byte_3~Byte_n): Parameter, the length is defined by the Byte_2
    - Check Code(Byte_n+1): Occupies two bytes, the value is the sum of the previous bytes
*/
esp_err_t radar_ctrl();


/*
    @brief: set the distance level of the radar, the range of distance value is 0~31
*/
esp_err_t set_distance_level(uint8_t distance_level);

/*
    @brief: get the distance level of the radar, the range of distance value is 0~31.
    @return: the distance level of the radar, uint8_t
*/
uint8_t get_distance_level();

/*
    @brief: set the delay time of the radar, the range of delay time is 1~65535, unit is second
    @param: para1:time(low byte), para2:time(high byte)
*/
esp_err_t set_delay_time(uint8_t para1, uint8_t para2);

/*
    @brief: get the delay time of the radar, the range of delay time is 1~65535, unit is second
    @return: the delay time of the radar, uint16_t
*/
uint16_t get_delay_time();

/*
    @brief: open or close the light sensing function of the radar
    @param: 0x01:open, 0x00:close
*/
esp_err_t enable_light_sensing(uint8_t para);

/*
    @brief: get the status of the light sensing function of the radar
    @return: 0x01:open, 0x00:close
*/
uint8_t get_light_sensing_status();

/*
    @brief: set the threshold of the light sensing function of the radar. range is 0~2799. (radar start to work when the light value is beyond the threshold)
    @param: light_threshold, uint16_t
*/
esp_err_t set_light_threshold(uint16_t light_threshold);

/*
    @brief: get the threshold of the light sensing function of the radar. range is 0~2799. (radar start to work when the light value is beyond the threshold)
    @return: light_threshold, uint16_t
*/
uint16_t get_light_threshold();

/*
    @brief: get the current light value of the radar.
    @return: light_value, uint16_t
*/
uint16_t get_cur_light_value();

/*
    @brief: open or close the DUL trigger function of the radar
    @param: 0x01:open, 0x00:close
*/
esp_err_t enable_dul_trigger(uint8_t para);

/*
    @brief: get the status of the DUL trigger function of the radar
    @return: 0x01:open, 0x00:close
*/
uint8_t get_dul_trigger_status();

/*
    @brief: set the level after triggered, high or low
    @param: 0x01:high available, 0x00:low available
*/
esp_err_t set_dul_trigger_level(uint8_t para);

/*
    @brief: get the level after triggered, high or low
    @return: 0x01:high available, 0x00:low available
*/
uint8_t get_dul_trigger_level();

/*
    @brief: set output mode, PWM of PLC
    @param: 0x01:PWM, 0x00:PLC
*/
esp_err_t set_output_mode(uint8_t para);

/*
    @brief: get output mode, PWM of PLC
    @return: 0x01:PWM, 0x00:PLC
*/
uint8_t get_output_mode();

/*
    @brief: get trigger status(if the radar is triggered)
    @return: 0x01:triggered, 0x00:not triggered
*/
uint8_t get_trigger_status();

/*
    @brief: turn on of turn off the gredienter function. (only available when the output mode is PWM)
    @param: 0x01:turn on, 0x00:turn off
*/
esp_err_t enable_gredienter(uint8_t para);

/*
    @brief: get the status of the gredienter function. (only available when the output mode is PWM)
    @return: 0x01:turn on, 0x00:turn off
*/
uint8_t get_gredienter_status();

/*
    @brief: turn on of turn off the radar.
    @param: 0x01:turn on, 0x00:turn off
*/
esp_err_t enable_radar(uint8_t para);

/*
    @brief: get the status of the radar.
    @return: 0x01:turn on, 0x00:turn off
*/
uint8_t get_radar_status();

/*
    @brief: save the current configuration to the radar.
    @param: 0x01:save, 0x00:not save
    @advise: delay 1s as least after save_config() function before send other command to the radar.
*/
esp_err_t save_config(uint8_t para);

/*
    @brief: get save status of the radar.
    @return: 0x01:save is on, 0x00:save is off
*/
uint8_t get_save_status();

/*
    @brief: enable direct_output, which means the judged signal will be send directly from the OUTPUT Pin of the radar.
    @param: 0x01:enable, 0x00:disable
*/
esp_err_t enable_direct_output(uint8_t para);

/*
    @brief: system reset
    @param: reset_mode: 0x01:sys_reset
*/
esp_err_t system_reset(uint8_t reset_mode);



#endif
