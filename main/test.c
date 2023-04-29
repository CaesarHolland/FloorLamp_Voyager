#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define EC11_A_PIN 30
#define EC11_B_PIN 15

#define COUNTER_MIN 0
#define COUNTER_MAX 1024

static xQueueHandle gpio_evt_queue = NULL;
static int counter = 0;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void ec11_task(void *arg)
{
    uint32_t io_num;
    int level_a, level_b;
    while(1)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            level_a = gpio_get_level(EC11_A_PIN);
            level_b = gpio_get_level(EC11_B_PIN);

            if (io_num == EC11_A_PIN && level_a == 1) {
                if (level_b == 1) {
                    counter++;
                    if (counter > COUNTER_MAX)
                        counter = COUNTER_MAX;
                }
                else {
                    counter--;
                    if (counter < COUNTER_MIN)
                        counter = COUNTER_MIN;
                }
            }
        }
        printf("couunter: %d\n", counter);
    }
}

void app_main()
{

}

// #include "driver/timer.h"
// #include "esp_log.h"

// #define TIMER_INTERVAL_SEC (10*60) // 10 minutes in seconds
// #define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // convert counter value to seconds
// #define TIMER_DIVIDER 16 // hardware timer clock divider
// #define TIMER_GROUP TIMER_GROUP_0
// #define TIMER_IDX TIMER_0

// static const char* TAG = "timer_example";

// void IRAM_ATTR timer_group0_isr(void *para){
//     int timer_idx = (int) para;
//     uint32_t intr_status = TIMERG0.int_st_timers.val;
//     if((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
//         TIMERG0.hw_timer[timer_idx].update = 1;
//         TIMERG0.int_clr_timers.t0 = 1;
//         uint64_t timer_counter_value = ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32 | TIMERG0.hw_timer[timer_idx].cnt_low;
//         double time = (double) timer_counter_value / (double) TIMER_SCALE;
//         ESP_LOGI(TAG, "Time: %.8f s", time);
//         timer_counter_value += (uint64_t) (TIMER_INTERVAL_SEC * TIMER_SCALE);
//         TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
//         TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
//         TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;
//     }
// }

// void example_tg0_timer_init(int timer_idx){
//     timer_config_t config = {
//         .divider = TIMER_DIVIDER,
//         .counter_dir = TIMER_COUNT_UP,
//         .counter_en = TIMER_PAUSE,
//         .alarm_en = 1,
//         .auto_reload = 1,
//     };
//     timer_init(TIMER_GROUP_0, timer_idx, &config);
//     timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);
//     timer_set_alarm_value(TIMER_GROUP_0, timer_idx, TIMER_INTERVAL_SEC * TIMER_SCALE);
//     timer_enable_intr(TIMER_GROUP_0, timer_idx);
//     timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
//     timer_start(TIMER_GROUP_0, timer_idx);
// }

// void app_main(){
//     example_tg0_timer_init(TIMER_0);
// }