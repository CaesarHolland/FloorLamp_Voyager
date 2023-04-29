#include "mymdns.h"
#include "mdns.h"
// #include "netdb.h"

#define HOSTNAME    CONFIG_FLOORLAMP_MDNS_HOST_NAME
static const char *TAG = "mymdns";

void start_mdns_service()
{
    ESP_LOGI(TAG, "MDNS is starting...");
    // 初始化 mDNS 服务
    esp_err_t err = mdns_init();
    if (err) {
        ESP_LOGE(TAG, "MDNS Init failed: %d", err);
        return;
    }

    // 设置 hostname
    ESP_ERROR_CHECK(mdns_hostname_set(HOSTNAME));
    // 设置默认实例
    ESP_ERROR_CHECK(mdns_instance_name_set("mymDNSInstance"));

    mdns_txt_item_t serviceTxtData[4] = {
        {"board","esp32"},
        {"dev_name", "Voyager"},
        {"u","user"},
        {"p","password"}
    };

    ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 81, serviceTxtData, 4));
    ESP_LOGI(TAG, "hostname: %s", HOSTNAME);
}

