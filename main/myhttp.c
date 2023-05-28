#include "myhttp.h"
#include "dashboard.h"
#include "general_ctrl.h"
#include "cJSON.h"

static const char * TAG = "MYHTTP";

static esp_err_t initPanel_get_handler(httpd_req_t *req)
{
    /* 发送回简单的响应数据包 */
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    return httpd_resp_send(req, (const char *)dashboard_html_gz, dashboard_html_gz_len);
}


static esp_err_t AD_event_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "get into AD_event_get_handler");
    ESP_LOGI(TAG, "get uri: %s", req->uri);

    size_t buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        char *buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            // buf 变量现在包含整个查询字符串
            ESP_LOGI(TAG, "after process for uri: %s", buf);    // such as "lm=900"
            char *paramArr = strtok(buf, "=");                  // such as "lm"
            ESP_LOGI(TAG, "PARAM_NAME: %s", paramArr);
            if (paramArr != NULL)
            {
                if (strcmp("lm", paramArr) == 0) {
                    paramArr = strtok(NULL, "=");
                    int val = atoi(paramArr);
                    DUTY_IRE_t dutyIre = {
                        .public_duty_ire = val,
                        .private_duty_ire = val,
                    };
                    ESP_ERROR_CHECK(esp_event_post_to(g_controller_loop_handler, DEVICE_AD, LM_IRE_AD, &dutyIre, sizeof(dutyIre)+1, portMAX_DELAY));
                    
                    httpd_resp_set_type(req, "application/json");
                    cJSON *root = cJSON_CreateObject();
                    DUTY_IRE_t duty = get_lumen_duty();
                    cJSON_AddNumberToObject(root, "leftDuty", duty.public_duty_ire);
                    cJSON_AddNumberToObject(root, "rightDuty", duty.private_duty_ire);
                    const char *sys_info = cJSON_Print(root);
                    httpd_resp_sendstr(req, sys_info);
                    free((void *)sys_info);
                    cJSON_Delete(root);
                }
                else if (strcmp("freq", paramArr) == 0) {
                    paramArr = strtok(NULL, "=");
                    int val = atoi(paramArr);
                    ESP_LOGI(TAG, "freq PARAM_VAL: %d", val);

                    httpd_resp_set_type(req, "application/json");
                    cJSON *root = cJSON_CreateObject();
                    cJSON_AddNumberToObject(root, "freq", get_frequency());
                    const char *sys_info = cJSON_Print(root);
                    httpd_resp_sendstr(req, sys_info);
                    free((void *)sys_info);
                    cJSON_Delete(root);
                }
                else if (strcmp("dist", paramArr) == 0) {
                    paramArr = strtok(NULL, "=");
                    int val = atoi(paramArr);
                    ESP_LOGI(TAG, "distance PARAM_VAL: %d", val);
                    httpd_resp_send(req, NULL, 0);
                }
                else if (strcmp("envlm", paramArr) == 0) {
                    paramArr = strtok(NULL, "=");
                    int val = atoi(paramArr);
                    ESP_LOGI(TAG, "envlm PARAM_VAL: %d", val);
                    httpd_resp_send(req, NULL, 0);
                }
                else if (strcmp("switch", paramArr) == 0) {         // /api/ad?switch=1
                    paramArr = strtok(NULL, "=");
                    int val = atoi(paramArr);
                    ESP_ERROR_CHECK(esp_event_post_to(g_controller_loop_handler, DEVICE_AD, SWITCH_AD, &val, sizeof(val)+1, portMAX_DELAY));
                    httpd_resp_send(req, NULL, 0);
                }
                else {
                    // no legal param detected
                    ESP_LOGI(TAG, "PARAM error");
                    httpd_resp_send_404(req);
                }
            }
        }
        free(buf);
    }
    return ESP_OK;
}

static esp_err_t MODE_event_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "get into MODE_event_get_handler");
    ESP_LOGI(TAG, "get uri: %s", req->uri);

    size_t buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        char *buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            // buf 变量现在包含整个查询字符串
            ESP_LOGI(TAG, "after process for uri: %s", buf);    // such as "ord=1"
            char *paramArr = strtok(buf, "=");                  // such as "ord"
            ESP_LOGI(TAG, "PARAM_NAME: %s", paramArr);
            if (paramArr != NULL)
            {
                if (strcmp("ord", paramArr) == 0) {
                    ESP_ERROR_CHECK(esp_event_post_to(g_controller_loop_handler, DEVICE_MODE, ORDINARY_MODE, NULL, 0, portMAX_DELAY));
                    httpd_resp_send(req, NULL, 0);
                }
                else if (strcmp("zen", paramArr) == 0) {
                    ESP_ERROR_CHECK(esp_event_post_to(g_controller_loop_handler, DEVICE_MODE, ZEN_MODE, NULL, 0, portMAX_DELAY));
                    httpd_resp_send(req, NULL, 0);
                }
                else if (strcmp("cand", paramArr) == 0) {
                    ESP_ERROR_CHECK(esp_event_post_to(g_controller_loop_handler, DEVICE_MODE, CANDLE_MODE, NULL, 0, portMAX_DELAY));
                    httpd_resp_send(req, NULL, 0);
                }
                else if (strcmp("hd", paramArr) == 0) {
                    paramArr = strtok(NULL, "=");
                    int val = atoi(paramArr);
                    ESP_ERROR_CHECK(esp_event_post_to(g_controller_loop_handler, DEVICE_MODE, HUMAN_DETECT, &val, sizeof(val)+1, portMAX_DELAY));
                    httpd_resp_send(req, NULL, 0);
                }
                else if (strcmp("envld", paramArr) == 0) {
                    ESP_ERROR_CHECK(esp_event_post_to(g_controller_loop_handler, DEVICE_MODE, ENV_LIGHT_DETECT, NULL, 0, portMAX_DELAY));
                    httpd_resp_send(req, NULL, 0);
                }
                else {
                    // no legal param detected
                    ESP_LOGI(TAG, "PARAM error");
                    httpd_resp_send_404(req);
                }
            }
        }
        free(buf);
    }
    return ESP_OK;
}

/*
    2. DUTY_THRESHOLD & DUTY_MAX_THRESHOLD
*/
static esp_err_t RANGE_get_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "min", DUTY_THRESHOLD);
    cJSON_AddNumberToObject(root, "max", DUTY_MAX_THRESHOLD);
    const char *sys_info = cJSON_Print(root);
    httpd_resp_sendstr(req, sys_info);
    free((void *)sys_info);
    cJSON_Delete(root);
    return ESP_OK;
}

/*
    start and stop functions:
*/
httpd_handle_t start_webserver(void)
{
    /* 生成默认的配置参数 */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    /* 置空 esp_http_server 的实例句柄 */
    httpd_handle_t server = NULL;

    /* 启动 httpd server */
    if (httpd_start(&server, &config) == ESP_OK)
    {
        const httpd_uri_t AD_event_get = {
            .uri = "/api/ad",
            .method = HTTP_GET,
            .handler = AD_event_get_handler,
            .user_ctx = "get ad req",
        };

        const httpd_uri_t MODE_event_get = {
            .uri = "/api/mode",
            .method = HTTP_GET,
            .handler = MODE_event_get_handler,
            .user_ctx = "get mode req",
        };

        const httpd_uri_t INFO_event_get = {
            .uri = "/api/range",
            .method = HTTP_GET,
            .handler = RANGE_get_handler,
            .user_ctx = "get range req",
        };
        
        // const httpd_uri_t initPannel_get = {
        //     .uri = "/uri",
        //     .method = HTTP_GET,
        //     .handler = initPanel_get_handler,
        //     .user_ctx = "hello world",
        // };

        /* 注册 URI 处理程序 */
        // httpd_register_uri_handler(server, &initPannel_get);
        httpd_register_uri_handler(server, &AD_event_get);
        httpd_register_uri_handler(server, &MODE_event_get);
        httpd_register_uri_handler(server, &INFO_event_get);
    }
    /* 如果服务器启动失败，返回的句柄是 NULL */
    return server;
}

void stop_webserver(httpd_handle_t server)
{
    if (server)
    {
        httpd_stop(server);
    }
}
