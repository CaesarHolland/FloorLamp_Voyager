#include "general_ctrl.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "hal/timer_types.h"

/*
    LEDC_PWM:
*/
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_PUBLIC_CH          LEDC_CHANNEL_0
#define LEDC_PRIVATE_CH         LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (512)   // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (20000) // Frequency in Hertz. Set frequency at 20 kHz


/* 
    BULB AND LED OUTPUT IO:
*/
#define LEDC_PUBLIC_OUTPUT_IO   CONFIG_PUBLIC_OUTPUT_IO    // Define the output GPIO
#define LEDC_PRIVATE_OUTPUT_IO  CONFIG_PRIVATE_OUTPUT_IO    // Define the output GPIO

/*
    RELAY IO:
*/
#define LIGHT_BULB_RELAY_IO     CONFIG_LIGHT_BULB_RELAY
#define LIGHT_LED_RELAY_IO      CONFIG_LIGHT_LED_RELAY
#define LIGHT_RELAY_IO_SEL      ((1ULL<<LIGHT_BULB_RELAY_IO) | (1ULL<<LIGHT_LED_RELAY_IO))

/*
    ENCODER IO:
*/
#define ENCODER_CLK             CONFIG_ENCODER_CLK
#define ENCODER_DT              CONFIG_ENCODER_DT
#define ENCODER_IO_SEL          ((1ULL<<ENCODER_CLK) | (1ULL<<ENCODER_DT))

/*
    EVENT GROUP:
*/
#define ENCODER_ISR_EVENT       BIT0
#define STORE_ISR_EVENT         BIT1
static EventGroupHandle_t       encoder_event_group;

/*
    INTERRUPT:
*/
#define ESP_INTR_FLAG_DEFAULT   0

/*
    HARDWARE TIMER:
*/
#define NVS_UPDATE_INTERVAL     CONFIG_NVS_UPDATE_INTERVAL
#define TIMER_INTERVAL_SEC      (NVS_UPDATE_INTERVAL*60) // 10 minutes in seconds
#define TIMER_SCALE             (TIMER_BASE_CLK / TIMER_DIVIDER) // convert counter value to seconds
#define TIMER_DIVIDER           16 // hardware timer clock divider
#define TIMER_GROUP             TIMER_GROUP_0
#define TIMER_IDX               TIMER_0

/*
    GLOBAL VARIBALES:
*/
esp_event_loop_handle_t         g_controller_loop_handler;
esp_event_base_t                DEVICE_AD   = "DEVICE_AD";
esp_event_base_t                DEVICE_MODE = "DEVICE_MODE";

/*
    NVS:
*/
static nvs_handle_t             device_info_handle;

/*
    ENCODER VARIBALES:
*/
static int                      g_counter = DUTY_THRESHOLD;
static int                      g_currentStateCLK;
static int                      g_lastStateCLK;
static int                      g_currentDir;
static int                      g_speedRate = DUTY_MAX_THRESHOLD/64;
// static QueueHandle_t            gpio_evt_queue = NULL;


static DEVICE_Info_t            g_current_DEVICE_status;

/*
    STATIC FUNCTIONS:
*/
static void encoder_task(void *ptr);
static void IRAM_ATTR update_encoder_isr_handler(void *arg);
static void encoder_init(void);
static void brightness_adj(int public_duty, int private_duty);
static void ledc_init(void);
static void general_controller_handler(void *handler_arg, esp_event_base_t base, int32_t id, void *event_data);
static void storage_timer_init();
static void IRAM_ATTR storage_isr_handler(void *ptr);
static void storage_daemon_task(void *ptr);
static void mode_relay_change(int flag);



void controller_init(void)
{
// init default params:
    esp_err_t err;
    err = nvs_open("device_info", NVS_READWRITE, &device_info_handle);
    // check if namespace "device info" exist, if not exist then create the namespace, otherwise read info directly:
    if (err == ESP_ERR_NVS_NOT_FOUND || ESP_ERR_NVS_PART_NOT_FOUND)
    {
        // create new namespace, then init g_default_DEVICE_status as 0 then write it into the namespace:
        g_current_DEVICE_status.public_duty             = DUTY_THRESHOLD+g_speedRate;
        g_current_DEVICE_status.private_duty            = DUTY_THRESHOLD+g_speedRate;
        g_current_DEVICE_status.zen_duty                = DUTY_THRESHOLD+g_speedRate;
        g_current_DEVICE_status.mode                    = ORDINARY_MODE;
        g_current_DEVICE_status.human_detect_flag       = 0;
        g_current_DEVICE_status.human_detect_distance   = 0;    // @todo: need to measure a suitable value as the default value, it should not be 0.
        g_current_DEVICE_status.env_light_detect_flag   = 0;    // @todo: need to measure a suitable value as the default value, it should not be 0.
        g_current_DEVICE_status.frequency               = 0;
        g_current_DEVICE_status.switch_status           = 0;

        ESP_ERROR_CHECK(nvs_set_blob(device_info_handle, "dev_info", &g_current_DEVICE_status, sizeof(g_current_DEVICE_status)));
        ESP_ERROR_CHECK(nvs_commit(device_info_handle));
    }
    else if (err == ESP_OK)
    {
        static DEVICE_Info_t g_current_DEVICE_status;
        ESP_ERROR_CHECK(nvs_get_blob(device_info_handle, "dev_info", &g_current_DEVICE_status, sizeof(g_current_DEVICE_status)));
    }
    // nvs_close(device_info_handle);

// init ledc_PWM
    ledc_init();

// create controller event loop
    ESP_LOGE("MAIN", "Create Event loop");
    esp_event_loop_args_t controller_loop_args = {
        .queue_size      = 10,                                      // sizeo of queue
        .task_name       = "controller_task",                         
        .task_priority   = uxTaskPriorityGet(NULL),                 // get the priority of the current task
        .task_stack_size = 1024*4,                                   
        .task_core_id    = tskNO_AFFINITY,
    };

// create controller event loop:
    ESP_ERROR_CHECK(esp_event_loop_create(&controller_loop_args, &g_controller_loop_handler));
// register event for general_controller_handler handle:
    esp_event_handler_register_with(g_controller_loop_handler, DEVICE_AD, LM_IRE_AD, general_controller_handler, NULL);
    esp_event_handler_register_with(g_controller_loop_handler, DEVICE_AD, FRE_AD, general_controller_handler, NULL);
    esp_event_handler_register_with(g_controller_loop_handler, DEVICE_AD, SWITCH_AD, general_controller_handler, NULL);
    esp_event_handler_register_with(g_controller_loop_handler, DEVICE_MODE, ORDINARY_MODE, general_controller_handler, NULL);
    esp_event_handler_register_with(g_controller_loop_handler, DEVICE_MODE, ZEN_MODE, general_controller_handler, NULL);
    esp_event_handler_register_with(g_controller_loop_handler, DEVICE_MODE, CANDLE_MODE, general_controller_handler, NULL);
    esp_event_handler_register_with(g_controller_loop_handler, DEVICE_MODE, HUMAN_DETECT, general_controller_handler, NULL);
    esp_event_handler_register_with(g_controller_loop_handler, DEVICE_MODE, ENV_LIGHT_DETECT, general_controller_handler, NULL);
    esp_event_handler_register_with(g_controller_loop_handler, DEVICE_MODE, DISTANCE_AD, general_controller_handler, NULL);
    esp_event_handler_register_with(g_controller_loop_handler, DEVICE_MODE, ENV_LM_AD, general_controller_handler, NULL);

// init event group
    encoder_event_group = xEventGroupCreate();
    if (encoder_event_group == NULL) {
        ESP_LOGE("EVENT_GROUP", "assert failed");
    }
    else {
        ESP_LOGE("EVENT_GROUP", "assert success");
    }

// init encoder
    encoder_init();

// init storage daemon
    // storage_timer_init();
}

static void ledc_init(void)
{
// 1. RELAY config:
    //zero-initialize the config structure.
    gpio_config_t io_conf = {
        //disable interrupt
        .intr_type = GPIO_INTR_DISABLE,
        //set as output mode
        .mode = GPIO_MODE_OUTPUT,
        //bit mask of the pins that you want to set,e.g.GPIO18/19
        .pin_bit_mask = LIGHT_RELAY_IO_SEL,
        //disable pull-down mode
        .pull_down_en = 0,
        //disable pull-up mode
        .pull_up_en = 0,
    };
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(LIGHT_BULB_RELAY_IO, 1);
    gpio_set_level(LIGHT_LED_RELAY_IO, 0);


// 2. PWM config:
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t bulb_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&bulb_timer));

    // right bulb config:
    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t rbulb_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_PUBLIC_CH,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_PRIVATE_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&rbulb_channel));

    // left bulb and ledring config:
    ledc_channel_config_t lbulb_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_PUBLIC_CH,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_PUBLIC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&lbulb_channel));

    brightness_adj(0, 0);
}


/*
    @brief: 此函数只负责根据给的两个值来改变两个通道的输出占空比，并在收到两个值都为零的数据时（代表关灯）切换继电器状态。
*/
static void brightness_adj(int public_duty, int private_duty)
{
    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_PUBLIC_CH, public_duty));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_PRIVATE_CH, private_duty));

    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_PUBLIC_CH));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_PRIVATE_CH));

    if (public_duty==0 && private_duty==0) {
        mode_relay_change(ORDINARY_MODE);
        g_current_DEVICE_status.switch_status = 0;
    }
}

/*
    @brief: this function will switch the relay to the required status according to flag, and update g_current_DEVICE_status.
*/
static void mode_relay_change(int flag) {
    if (flag == ORDINARY_MODE)
    {
        // ORDINARY_MODE
        gpio_set_level(LIGHT_BULB_RELAY_IO, 1);
        gpio_set_level(LIGHT_LED_RELAY_IO, 0);
        g_current_DEVICE_status.mode = ORDINARY_MODE;
    }
    else if (flag == ZEN_MODE)
    {
        // ZEN_MODE
        gpio_set_level(LIGHT_BULB_RELAY_IO, 0);
        gpio_set_level(LIGHT_LED_RELAY_IO, 1);
        g_current_DEVICE_status.mode = ZEN_MODE;
    }
    else if (flag == CANDLE_MODE)
    {
        // CANDLE_MODE
        gpio_set_level(LIGHT_BULB_RELAY_IO, 1);
        gpio_set_level(LIGHT_LED_RELAY_IO, 0);
        g_current_DEVICE_status.mode = CANDLE_MODE;
    }
}


static void general_controller_handler(void *handler_arg, esp_event_base_t base, int32_t id, void *event_data)
{
    char *TAG = "GENERAL_CONTROLLER_TASK";
    // ESP_LOGE(TAG, "CONTROLLER IS RUNNING");

    if (base == DEVICE_AD)
    {
        switch (id)
        {
            case LM_IRE_AD:
            {
                // 这里传来的参数是增量，所以该task负责根据传递增量自行结合当前量判断，且默认传来的是合法的
                ESP_LOGI(TAG, "GET LM_IRE_AD event");
                DUTY_IRE_t *duty_ire = (DUTY_IRE_t *)event_data;
                 
                int pubd = (g_current_DEVICE_status.public_duty + duty_ire->public_duty_ire);
                int prid = g_current_DEVICE_status.private_duty + duty_ire->private_duty_ire;
                int zend = g_current_DEVICE_status.zen_duty + duty_ire->zen_duty_ire;

                if (g_current_DEVICE_status.mode == ORDINARY_MODE) {
                    if (pubd <= DUTY_THRESHOLD) {
                        // LIGHT OFF
                        brightness_adj(0, 0);
                        ESP_LOGI("CONTROLELR", "LIGHT OFF -- pubd:%d, prid:%d, zend:%d, st:%d, mode:%d", pubd, prid, zend, g_current_DEVICE_status.switch_status, g_current_DEVICE_status.mode);
                        // timer_pause(TIMER_GROUP, TIMER_IDX);    // Pausing timer when the light is off.
                    }
                    else if (pubd <= DUTY_MAX_THRESHOLD){
                        // LUMEN CHANGING REGULARLY
                        brightness_adj(pubd, prid);
                        // the current value need to be updated
                        g_current_DEVICE_status.public_duty = pubd;
                        g_current_DEVICE_status.private_duty = prid;
                        ESP_LOGI("CONTROLELR", "REGU -- pubd:%d, prid:%d, zend:%d, st:%d, mode:%d", pubd, prid, zend, g_current_DEVICE_status.switch_status, g_current_DEVICE_status.mode);
                    }
                }
                else if (g_current_DEVICE_status.mode == ZEN_MODE) {
                    if (zend <= DUTY_THRESHOLD) {
                        // LIGHT OFF
                        brightness_adj(0, 0);
                        ESP_LOGI("CONTROLELR", "ZEN LIGHT OFF -- pubd:%d, prid:%d, zend:%d, st:%d, mode:%d", pubd, prid, zend, g_current_DEVICE_status.switch_status, g_current_DEVICE_status.mode);
                        // timer_pause(TIMER_GROUP, TIMER_IDX);    // Pausing timer when the light is off.
                    }
                    else {
                        brightness_adj(zend, 0);
                        g_current_DEVICE_status.zen_duty = zend;
                        ESP_LOGI("CONTROLELR", "ZEN REGU -- pubd:%d, prid:%d, zend:%d, st:%d, mode:%d", pubd, prid, zend, g_current_DEVICE_status.switch_status, g_current_DEVICE_status.mode);
                    }
                }
                break;
            }

            case FRE_AD:
                ESP_LOGI(TAG, "GET FRE_AD event");
                // @todo: create a Closed-loop variation curve, the cycle speed depends on a parameter, which is our handle

                break;
            
            case DISTANCE_AD:
                ESP_LOGI(TAG, "GET DISTANCE_AD event");

                break;

            case ENV_LM_AD:
                ESP_LOGI(TAG, "GET ENV_LM_AD event");

                break;
            
            case SWITCH_AD:
            {
                ESP_LOGI(TAG, "GET SWITCH_AD event");
                int *flag = (int *)event_data;
                if (*flag) {
                    // SWITCH ON
                    // timer_start(TIMER_GROUP, TIMER_IDX);
                    brightness_adj(g_current_DEVICE_status.public_duty, g_current_DEVICE_status.private_duty);
                    g_current_DEVICE_status.switch_status = 1;
                    ESP_LOGI("CONTROLLER", "SWITCH ON -- pub_duty:%d, prid:%d, zend:%d, st:%d", 
                                    g_current_DEVICE_status.public_duty,
                                    g_current_DEVICE_status.private_duty,
                                    g_current_DEVICE_status.zen_duty,
                                    g_current_DEVICE_status.switch_status);
                }
                else {
                    // SWITCH OFF
                    brightness_adj(0, 0);
                    ESP_LOGI("CONTROLLER", "SWITCH OFF -- pub_duty:%d, prid:%d, zend:%d, st:%d", 
                                    g_current_DEVICE_status.public_duty,
                                    g_current_DEVICE_status.private_duty,
                                    g_current_DEVICE_status.zen_duty,
                                    g_current_DEVICE_status.switch_status);
                    // timer_pause(TIMER_GROUP, TIMER_IDX);    // Pausing timer when the light is off.
                }
                break;
            }
            
            default:
                break;
        }
    }
    else if (base == DEVICE_MODE)
    {
        switch (id)
        {
        case ORDINARY_MODE:
            ESP_LOGI(TAG, "GET ORDINARY_MODE event");
            // turn on LIGHT_RELAY1 and turn off LIGHT_RELAY2
            mode_relay_change(ORDINARY_MODE);
            brightness_adj(g_current_DEVICE_status.public_duty, g_current_DEVICE_status.private_duty);
            break;
        
        case ZEN_MODE:
            ESP_LOGI(TAG, "GET ZEN_MODE event");
            // turn off LIGHT_RELAY1 and turn on LIGHT_RELAY2
            mode_relay_change(ZEN_MODE);
            brightness_adj(g_current_DEVICE_status.zen_duty, 0);
            break;
        
        case CANDLE_MODE:
            ESP_LOGI(TAG, "GET CANDLE_MODE event");
            mode_relay_change(CANDLE_MODE);
            break;
        
        case HUMAN_DETECT:
        {
            ESP_LOGI(TAG, "GET HUMAN_DETECT event");
            int *flag = (int *)event_data;
            if (*flag) {
                // HUMAN_DETECT ON
                g_current_DEVICE_status.human_detect_flag = 1;
            }
            else {
                // HUMAN_DETECT OFF
                g_current_DEVICE_status.human_detect_flag = 0;
            }
            break;
        }

        case ENV_LIGHT_DETECT:
        {
            ESP_LOGI(TAG, "GET ENV_LIGHT_DETECT event");
            int *flag = (int *)event_data;
            if (*flag) {
                // ENV_LIGHT_DETECT ON
                g_current_DEVICE_status.env_light_detect_flag = 1;
            }
            else {
                // ENV_LIGHT_DETECT OFF
                g_current_DEVICE_status.env_light_detect_flag = 0;
            }
            break;
        }
        
        default:
            break;
        }
    }

}


/*
    @brief: 
        Some pre-operation need to be done before an event posted:
        - When the current status is light-off, then 
*/
static void encoder_task(void *ptr)
{
    DUTY_IRE_t dutyIre = {
        .public_duty_ire = 0,
        .private_duty_ire = 0,
        .zen_duty_ire = 0,
    };
    size_t size = sizeof(dutyIre)+1;
    while (1) {
        if (xEventGroupWaitBits(encoder_event_group, ENCODER_ISR_EVENT, pdTRUE, pdFALSE, portMAX_DELAY)) {
            if (g_currentDir != 0) {
                ESP_LOGE("ENCODER_TASK", "g_counter: %d,   direction: %d", g_counter, g_currentDir);
                if (!g_current_DEVICE_status.switch_status && g_currentDir == 1) {
                    // means LIGHT ON action:
                    int flag=1;
                    ESP_ERROR_CHECK(esp_event_post_to(g_controller_loop_handler, DEVICE_AD, SWITCH_AD, &flag, 4, portMAX_DELAY));
                    vTaskDelay(1500/portTICK_RATE_MS);
                    g_counter = g_current_DEVICE_status.private_duty;
                }
                else {
                    dutyIre.public_duty_ire = g_counter-g_current_DEVICE_status.public_duty;
                    dutyIre.private_duty_ire = g_counter-g_current_DEVICE_status.private_duty;
                    dutyIre.zen_duty_ire = g_counter-g_current_DEVICE_status.zen_duty;

                    // ESP_ERROR_CHECK(esp_event_post_to(g_controller_loop_handler, DEVICE_AD, LM_IRE_AD, &dutyIre, size, portMAX_DELAY));
                    if (g_current_DEVICE_status.mode == ZEN_MODE) {
                        brightness_adj(g_counter, 0);
                    }
                    else {
                        brightness_adj(g_counter, g_counter);
                    }
                    vTaskDelay(20/portTICK_RATE_MS);
                }

            }
        }
    }
    vTaskDelete(NULL);
}


static void IRAM_ATTR update_encoder_isr_handler(void *arg)
{
    // range of g_counter: [DUTY_THRESHOLD, DUTY_MAX_THRESHOLD]
    g_currentStateCLK = gpio_get_level(ENCODER_CLK);
    if (g_currentStateCLK != g_lastStateCLK && g_currentStateCLK) {
        if (gpio_get_level(ENCODER_DT) != g_currentStateCLK) {
            // decrease
            g_counter >= (DUTY_THRESHOLD+g_speedRate) ? g_counter-=g_speedRate : (g_counter=DUTY_THRESHOLD);
            g_currentDir = -1;
        }
        else {
            // increase
            g_counter <= (DUTY_MAX_THRESHOLD-g_speedRate) ? g_counter+=g_speedRate : (g_counter=DUTY_MAX_THRESHOLD);
            g_currentDir = 1;
        }
    }
    g_lastStateCLK = g_currentStateCLK;

    
    xEventGroupSetBits(encoder_event_group, ENCODER_ISR_EVENT);
    // xQueueSendFromISR(gpio_evt_queue, &g_counter, NULL);
}



static void encoder_init(void)
{
    ESP_LOGE("ENCODER_INIT", "ENCODER init running");
// 1. GPIO config:
    //zero-initialize the config structure.
    gpio_config_t io_conf = {
        //disable interrupt
        .intr_type = GPIO_INTR_ANYEDGE,
        //set as output mode
        .mode = GPIO_MODE_INPUT,
        //bit mask of the pins that you want to set,e.g.GPIO18/19
        .pin_bit_mask = ENCODER_IO_SEL,
        //disable pull-down mode
        .pull_down_en = 0,
        //disable pull-up mode
        .pull_up_en = 1,
    };
    //configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_LOGI("ENCODER", "after gpio_config");
    //create a queue to handle gpio event from isr
    // gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER_CLK, update_encoder_isr_handler, (void *)ENCODER_CLK));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER_DT, update_encoder_isr_handler, (void *)ENCODER_DT));

    xTaskCreate(encoder_task, "encoder_task", 1024*12, NULL, 1, NULL);
}



/*
    @brief:
        
        The timer will be paused when the light is off.
*/
static void storage_timer_init(void) {
    ESP_LOGE("STORAGE_TIMER_INIT", "RUNNING");
    timer_config_t timerConfig = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = 1,
        .auto_reload = 1,
    };
    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &timerConfig));
    ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL));
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_SEC * TIMER_SCALE));
    ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, TIMER_0));
    ESP_ERROR_CHECK(timer_isr_register(TIMER_GROUP_0, TIMER_0, storage_isr_handler, (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL));
    // ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, TIMER_0));

    xTaskCreate(storage_daemon_task, "storage_daemon", 1024*2, NULL, 10, NULL);
}

static void storage_daemon_task(void *ptr) {
    ESP_LOGE("STORAGE_DAEMON", "RUNNING");
    while (1)
    {
        if (xEventGroupWaitBits(encoder_event_group, STORE_ISR_EVENT, pdFALSE, pdFALSE, portMAX_DELAY))
        {
            ESP_LOGI("STORAGE_DAEMON_TASK", "NVS UPDATE");
            // ESP_ERROR_CHECK(nvs_set_blob(device_info_handle, "dev_info", &g_current_DEVICE_status, sizeof(g_current_DEVICE_status)));
            // ESP_ERROR_CHECK(nvs_commit(device_info_handle));
        }
    }
    vTaskDelete(NULL);
}

static void IRAM_ATTR storage_isr_handler(void *ptr)
{
    xEventGroupSetBits(encoder_event_group, STORE_ISR_EVENT);
}



DEVICE_Info_t get_device_status(void)
{
    return g_current_DEVICE_status;
}

DUTY_IRE_t get_lumen_duty(void)
{
    DUTY_IRE_t duty = {
        .public_duty_ire = g_current_DEVICE_status.public_duty,
        .private_duty_ire = g_current_DEVICE_status.private_duty,
    };
    return duty;
}

int get_mode(void)
{
    return g_current_DEVICE_status.mode;
}

int get_frequency(void)
{
    return g_current_DEVICE_status.frequency;
}
