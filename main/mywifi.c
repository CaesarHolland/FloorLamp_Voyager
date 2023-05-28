#include "mywifi.h"
#include "myhttp.h"
#include "mymdns.h"


static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

/*
    @brief: WiFi事件中心，监听各类WiFi事件并作出相应处理
*/
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    char *TAG = "wifi_event_handler";
    
// WIFI_EVENT_STA_START
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }

// WIFI_EVENT_STA_DISCONNECTED
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num ++;
            ESP_LOGI(TAG, "retry to connect to AP");
        }
        else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGE(TAG, "connect to the AP fail");
    }

// IP_EVENT_STA_GOT_IP
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }

// WIFI_EVENT_AP_STACONNECTED
    else if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }

// WIFI_EVENT_AP_STADISCONNECTED
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}




/*
    @brief: enter AP mode, set webpage as [Initializer panel]
    @param:
*/
void wifi_init_AP(void *ptr)
{
    const char *TAG = "wifi_init_ap";

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "Groundlamp",
            .ssid_len = 11,
            .channel = 6,
            .password = "groundlamp",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    if (strlen((char *)(wifi_config.ap.password)) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             "Groundlamp", "groundlamp", 6);
    
    static httpd_handle_t server = NULL;
    server = start_webserver();

    // 启动mDNS服务
    start_mdns_service();
    
    vTaskDelete(NULL);
}


/* 
    @brief: enter STA mode, set webpage as [Dashboard panel]
    @param: wifi_list_handle
*/
void wifi_init_sta(void *ptr)
{
    // notice that the wifi_list_handle is in READONLY mode
    nvs_handle_t wifi_list_handle = *((nvs_handle_t *)ptr);

    // initialize global handle of default event group
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    const char *TAG = "wifi_init_sta";
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t sta_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&sta_cfg));

    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_t instance_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, 
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // start traverse wifi_list to find an available AP
    nvs_iterator_t wifilist_it = NULL;
    nvs_entry_find("nvs", "wifi_list", NVS_TYPE_BLOB, &wifilist_it);
    while (wifilist_it != NULL)
    {
        nvs_entry_info_t info;
        nvs_entry_info(wifilist_it, &info);
        AP_Info_t ap_info;
        size_t required_size = 0;
        nvs_get_blob(wifi_list_handle, info.key, &ap_info, &required_size);

        // 现在得到了一条WiFi连接信息，尝试连接，如果失败则继续遍历列表
        wifi_config_t sta_config = {
            .sta = {
                .ssid = {ap_info.ssid},
                .password = {ap_info.pwd},
                .scan_method = WIFI_FAST_SCAN,
                .sort_method = DEFAULT_SORT_METHOD,
                .threshold.rssi = DEFAULT_RSSI,
                .threshold.authmode = DEFAULT_AUTHMODE,
            },
        };


        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        ESP_LOGE("wifi_init_sta", "wifi_init_sta finished.");

        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECT_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

        if (bits & WIFI_CONNECT_BIT) {
            ESP_LOGI(TAG, "connected to AP SSID:%s,   password:%s", ap_info.ssid, ap_info.pwd);
            break;
        }
        else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGI(TAG, "Failed to connect to AP SSID:%s,   password:%s", ap_info.ssid, ap_info.pwd);
        }
        else {
            ESP_LOGE(TAG, "UNEXPECTED EVENT");
        }

        wifilist_it = nvs_entry_next(&wifilist_it);
    }

    // initialize http server:



    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);

    nvs_close(wifi_list_handle);

    // 启动mDNS服务
    start_mdns_service();

    vTaskDelete(NULL);
}
