#include "radar.h"
#include "driver/uart.h"
#include "general_ctrl.h"

static const char * TAG = "RADAR";
static QueueHandle_t uart1_queue;

esp_err_t radar_init(int baudrate)
{
    const int uart1_buffer_size = 1024;

    uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
        // .rx_flow_ctrl_thresh = 122,
    };
    // 2.1 Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(RADAR_UART_NUM, &uart_config));

    // 2.2 Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(RADAR_UART_NUM, RADAR_TXD, RADAR_RXD, RADAR_RTS, RADAR_CTS));

    // 2.3 Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(RADAR_UART_NUM, uart1_buffer_size,
                                        uart1_buffer_size, 10, &uart1_queue, 0));

    // 2.4 Enable UART interrupt
    // ESP_ERROR_CHECK(uart_enable_intr_mask(UART_NUM_2, true));
    

    // uart_enable_pattern_det_baud_intr(UART_NUM_2, '+', PATTERN_CHR_NUM, 1, 1, 1);

    // start uart event task
    // xTaskCreate(uart_event_task, "uart_event_task", 1024*12, NULL, 12, NULL);

    gpio_config_t radar_out_io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << RADAR_OUT_IO),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    // init gpio according to the configuration structure
    gpio_config(&radar_out_io_conf);
    
    return ESP_OK;
}

