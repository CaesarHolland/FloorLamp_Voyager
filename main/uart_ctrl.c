#include "uart_ctrl.h"
#include "driver/uart.h"
#include "general_ctrl.h"

static char * TAG = "UART";
static const int duty_step = 100;
static QueueHandle_t uart2_queue;


void uart_event_task(void *pvParameters)
{
    uart_event_t uart2_event;
    size_t buffered_size;
    char* dtmp = (char*) malloc(ASR_RD_BUF_SIZE);     // the buffer of reading data

    while (1)
    {
        // the task will keep blocked until the queue is not empty
        if (xQueueReceive(uart2_queue, (void *)&uart2_event, (TickType_t)portMAX_DELAY))
        {
            memset(dtmp, 0, ASR_RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", ASR_UART_NUM);
            switch (uart2_event.type)
            {
                // queue received data:
                case UART_DATA:
                    uart_read_bytes(ASR_UART_NUM, dtmp, uart2_event.size, portMAX_DELAY);
                    // @todo: pass the data to controller function
                    int data = atoi(dtmp);
                    ESP_LOGI(TAG, "[UART DATA]: %d", data);
                    switch (data)
                    {
                        case 1:
                        {
                            // lighter
                            DUTY_IRE_t dutyIre = {
                                .public_duty_ire = duty_step,
                                .private_duty_ire = duty_step,
                                .zen_duty_ire = duty_step
                            };
                            esp_event_post_to(g_controller_loop_handler, DEVICE_AD, LM_IRE_AD, &dutyIre, sizeof(dutyIre)+1, portMAX_DELAY);
                            break;
                        }

                        case 2:
                        {
                            // darker
                            DUTY_IRE_t dutyIre = {
                                .public_duty_ire = -duty_step,
                                .private_duty_ire = -duty_step,
                                .zen_duty_ire = -duty_step
                            };
                            esp_event_post_to(g_controller_loop_handler, DEVICE_AD, LM_IRE_AD, &dutyIre, sizeof(dutyIre)+1, portMAX_DELAY);
                            break;
                        }

                        case 3:
                        {
                            // switch off
                            int flag = 0;
                            esp_event_post_to(g_controller_loop_handler, DEVICE_AD, SWITCH_AD, &flag, sizeof(flag)+1, portMAX_DELAY);
                            break;
                        }

                        case 4:
                        {
                            // switch on
                            int flag = 1;
                            esp_event_post_to(g_controller_loop_handler, DEVICE_AD, SWITCH_AD, &flag, sizeof(flag)+1, portMAX_DELAY);
                            break;
                        }

                        case 5:
                        {
                            // human detect on
                            int flag = 1;
                            esp_event_post_to(g_controller_loop_handler, DEVICE_MODE, HUMAN_DETECT, &flag, sizeof(flag)+1, portMAX_DELAY);
                            break;
                        }

                        case 6:
                        {
                            // human detect off
                            int flag = 0;
                            esp_event_post_to(g_controller_loop_handler, DEVICE_MODE, HUMAN_DETECT, &flag, sizeof(flag)+1, portMAX_DELAY);
                            break;
                        }

                        case 7:
                            // zen mode
                            esp_event_post_to(g_controller_loop_handler, DEVICE_MODE, ZEN_MODE, NULL, 0, portMAX_DELAY);
                            break;
                        case 8:
                            // ordinary mode
                            esp_event_post_to(g_controller_loop_handler, DEVICE_MODE, ORDINARY_MODE, NULL, 0, portMAX_DELAY);
                            break;

                        case 9:
                            // ordinary mode
                            esp_event_post_to(g_controller_loop_handler, DEVICE_MODE, CANDLE_MODE, NULL, 0, portMAX_DELAY);
                            break;

                        default:
                            break;
                    }                    

                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(ASR_UART_NUM);
                    xQueueReset(uart2_queue);
                    break;

                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider increasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(ASR_UART_NUM);
                    xQueueReset(uart2_queue);
                    break;

                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;

                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;

                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                
                default:
                    ESP_LOGI(TAG, "uart event type: %d", uart2_event.type);
                    break;
            }
        }
    }

    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
    
}

esp_err_t serial_init(int baudrate)
{
    const int uart2_buffer_size = 1024;

    uart_config_t uart_config = {
        .baud_rate  = baudrate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
        // .rx_flow_ctrl_thresh = 122,
    };
    // 2.1 Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(ASR_UART_NUM, &uart_config));

    // 2.2 Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(ASR_UART_NUM, ASR_TX, ASR_RX, ASR_RTS, ASR_CTS));

    // 2.3 Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(ASR_UART_NUM, uart2_buffer_size,
                                        uart2_buffer_size, 10, &uart2_queue, 0));

    // 2.4 Enable UART interrupt
    // ESP_ERROR_CHECK(uart_enable_intr_mask(UART_NUM_2, true));
    

    // uart_enable_pattern_det_baud_intr(UART_NUM_2, '+', PATTERN_CHR_NUM, 1, 1, 1);

    // start uart event task
    xTaskCreate(uart_event_task, "uart_event_task", 1024*12, NULL, 12, NULL);
    
    return ESP_OK;
}



