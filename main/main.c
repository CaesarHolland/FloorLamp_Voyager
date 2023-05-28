#include "floorlamp.h"
#include "myhttp.h"
#include "mywifi.h"
#include "uart_ctrl.h"
#include "general_ctrl.h"

const char *default_partition = "nvs";

void SystemInit(void);
void SystemInfo(void);

/*
    @brief: main task, call SystemInit function
*/
void app_main(void)
{
    SystemInit();
    vTaskDelete(NULL);
}


/*
    @brief: 
*/
void SystemInit(void)
{
// step1: 选择服务器启动模式--【station模式】【AP模式】并初始化到相应WiFi模式
    // 每次启动时先从nvs里找AP列表，如果不存在的话，就打开AP模式，使用默认IP；
    // 初始化默认NVS分区
    esp_err_t err;
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // 拿到wifi_list命名空间的handle
    nvs_handle_t wifi_list_handle;
    err = nvs_open("wifi_list", NVS_READONLY, &wifi_list_handle);

    if (err != ESP_OK || err == (ESP_ERR_NVS_NOT_FOUND || ESP_ERR_NVS_PART_NOT_FOUND)) {
        // namespace not exist or something, enter AP mode, change light mode:
        ESP_LOGI("SystemInit", "enter ap mode---");
        xTaskCreate(wifi_init_AP, "wifi_init_AP", 1024*12, NULL, 1, NULL);
    }
    else if (err == ESP_OK) {
        ESP_LOGI("SystemInit", "enter sta mode---");
        xTaskCreate(wifi_init_sta, "wifi_init_sta", 1024*12, &wifi_list_handle, 1, NULL);
    }
    

// step2: 初始化UART串口
    ESP_ERROR_CHECK(serial_init(115200));
    
// step3: 初始化编码器控制


// step4: 初始化pwm控制
    controller_init();

    vTaskDelete(NULL);
}


/*
    @brief: tool function, used for debug to check system information
*/
// void SystemInfo(void)
// {
//     /* Print chip information */
//     esp_chip_info_t chip_info;
//     esp_chip_info(&chip_info);
//     printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
//            CONFIG_IDF_TARGET,
//            chip_info.cores,
//            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
//            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

//     printf("silicon revision %d, ", chip_info.revision);

//     printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
//            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

//     printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
// }

