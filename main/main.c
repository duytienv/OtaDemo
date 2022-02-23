/**
 * @file main.c
 */

#include "driver/gpio.h"
#include "dht11.h"


#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "cJSON.h"

#include "main.h"
#include "wifi.h"

#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "mqtt_client.h"

#include "nvs.h"
#include "nvs_flash.h"


#include "esp_timer.h"
#include "rom/ets_sys.h"

static gpio_num_t dht_gpio;
static int64_t last_read_time = -2000000;
static struct dht11_reading last_read;

static int _waitOrTimeout(uint16_t microSeconds, int level) {
    int micros_ticks = 0;
    while(gpio_get_level(dht_gpio) == level) { 
        if(micros_ticks++ > microSeconds) 
            return DHT11_TIMEOUT_ERROR;
        ets_delay_us(1);
    }
    return micros_ticks;
}

static int _checkCRC(uint8_t data[]) {
    if(data[4] == (data[0] + data[1] + data[2] + data[3]))
        return DHT11_OK;
    else
        return DHT11_CRC_ERROR;
}

static void _sendStartSignal() {
    gpio_set_direction(dht_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(dht_gpio, 0);
    ets_delay_us(20 * 1000);
    gpio_set_level(dht_gpio, 1);
    ets_delay_us(40);
    gpio_set_direction(dht_gpio, GPIO_MODE_INPUT);
}

static int _checkResponse() {
    /* Wait for next step ~80us*/
    if(_waitOrTimeout(80, 0) == DHT11_TIMEOUT_ERROR)
        return DHT11_TIMEOUT_ERROR;

    /* Wait for next step ~80us*/
    if(_waitOrTimeout(80, 1) == DHT11_TIMEOUT_ERROR) 
        return DHT11_TIMEOUT_ERROR;

    return DHT11_OK;
}

static struct dht11_reading _timeoutError() {
    struct dht11_reading timeoutError = {DHT11_TIMEOUT_ERROR, -1, -1};
    return timeoutError;
}

static struct dht11_reading _crcError() {
    struct dht11_reading crcError = {DHT11_CRC_ERROR, -1, -1};
    return crcError;
}

void DHT11_init(gpio_num_t gpio_num) {
    /* Chờ 1 giây để thiết bị vượt qua trạng thái không ổn định ban đầu */

    dht_gpio = gpio_num;
}

struct dht11_reading DHT11_read() {
    /* Tried to sense too son since last read (dht11 needs ~2 seconds to make a new read) */
    if(esp_timer_get_time() - 2000000 < last_read_time) {
        return last_read;
    }

    last_read_time = esp_timer_get_time();

    uint8_t data[5] = {0,0,0,0,0};

    _sendStartSignal();

    if(_checkResponse() == DHT11_TIMEOUT_ERROR)
        return last_read = _timeoutError();
    
    /* Read response */
    for(int i = 0; i < 40; i++) {
        /* Initial data */
        if(_waitOrTimeout(50, 0) == DHT11_TIMEOUT_ERROR)
            return last_read = _timeoutError();
                
        if(_waitOrTimeout(70, 1) > 28) {
            /* Bit received was a 1 */
            data[i/8] |= (1 << (7-(i%8)));
        }
    }

    if(_checkCRC(data) != DHT11_CRC_ERROR) {
        last_read.status = DHT11_OK;
        last_read.temperature = data[2];
        last_read.humidity = data[0];
        return last_read;
    } else {
        return last_read = _crcError();
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}



extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

/*! lưu các GT bit dung trong ứng dụng*/
static EventGroupHandle_t event_group;

/*! lưu OTA config dc cập nhật từ Thingsboard*/
static struct shared_keys
{
    char targetFwServerUrl[256];
    char targetFwVer[128];
} shared_attributes;

/*! bộ nhớ đệm lưu messgase */
static char mqtt_msg[512];

static esp_mqtt_client_handle_t mqtt_client;

static void parse_ota_config(const cJSON *object)
{
    if (object != NULL)
    {
        cJSON *server_url_response = cJSON_GetObjectItem(object, TB_SHARED_ATTR_FIELD_TARGET_FW_URL);
        if (cJSON_IsString(server_url_response) && (server_url_response->valuestring != NULL) && strlen(server_url_response->valuestring) < sizeof(shared_attributes.targetFwServerUrl))
        {
            memcpy(shared_attributes.targetFwServerUrl, server_url_response->valuestring, strlen(server_url_response->valuestring));
            shared_attributes.targetFwServerUrl[sizeof(shared_attributes.targetFwServerUrl) - 1] = 0;
            ESP_LOGI(TAG, "Received firmware URL: %s", shared_attributes.targetFwServerUrl);
        }

        cJSON *target_fw_ver_response = cJSON_GetObjectItem(object, TB_SHARED_ATTR_FIELD_TARGET_FW_VER);
        if (cJSON_IsString(target_fw_ver_response) && (target_fw_ver_response->valuestring != NULL) && strlen(target_fw_ver_response->valuestring) < sizeof(shared_attributes.targetFwVer))
        {
            memcpy(shared_attributes.targetFwVer, target_fw_ver_response->valuestring, strlen(target_fw_ver_response->valuestring));
            shared_attributes.targetFwVer[sizeof(shared_attributes.targetFwVer) - 1] = 0;
            ESP_LOGI(TAG, "Received firmware version: %s", shared_attributes.targetFwVer);
        }
    }
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    assert(event != NULL);

    switch (event->event_id)
    {
    case MQTT_EVENT_ANY:
        break; 
    case MQTT_EVENT_CONNECTED:
        xEventGroupClearBits(event_group, MQTT_DISCONNECTED_EVENT);
        xEventGroupSetBits(event_group, MQTT_CONNECTED_EVENT);
        ESP_LOGD(TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        xEventGroupClearBits(event_group, MQTT_CONNECTED_EVENT);
        xEventGroupSetBits(event_group, MQTT_DISCONNECTED_EVENT);
        ESP_LOGD(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGD(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGD(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGD(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGD(TAG, "MQTT_EVENT_DATA, msg_id=%d, %s", event->msg_id, event->topic);
        if (event->data_len >= (sizeof(mqtt_msg) - 1))
        {
            ESP_LOGE(TAG, "Received MQTT message size [%d] more than expected [%d]", event->data_len, (sizeof(mqtt_msg) - 1));
            return ESP_FAIL;
        }

        if (strcmp(TB_ATTRIBUTES_RESPONSE_TOPIC, event->topic) == 0)
        {
            memcpy(mqtt_msg, event->data, event->data_len);
            mqtt_msg[event->data_len] = 0;
            cJSON *attributes = cJSON_Parse(mqtt_msg);
            if (attributes != NULL)
            {
                cJSON *shared = cJSON_GetObjectItem(attributes, "shared");
                parse_ota_config(shared);
            }

            char *attributes_string = cJSON_Print(attributes);
            cJSON_Delete(attributes);
            ESP_LOGD(TAG, "Shared attributes response: %s", attributes_string);
            // Free is intentional, it's client responsibility to free the result of cJSON_Print
            free(attributes_string);
            xEventGroupSetBits(event_group, OTA_CONFIG_FETCHED_EVENT);
        }
        else if (strcmp(TB_ATTRIBUTES_TOPIC, event->topic) == 0)
        {
            memcpy(mqtt_msg, event->data, MIN(event->data_len, sizeof(mqtt_msg)));
            mqtt_msg[event->data_len] = 0;
            cJSON *attributes = cJSON_Parse(mqtt_msg);
            parse_ota_config(attributes);
            char *attributes_string = cJSON_Print(attributes);
            cJSON_Delete(attributes);
            ESP_LOGD(TAG, "Shared attributes were updated on ThingsBoard: %s", attributes_string);
            // Free is intentional, it's client responsibility to free the result of cJSON_Print
            free(attributes_string);
            xEventGroupSetBits(event_group, OTA_CONFIG_UPDATED_EVENT);
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGD(TAG, "MQTT_EVENT_ERROR");
        break;
    case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGD(TAG, "MQTT_EVENT_BEFORE_CONNECT");
        break; 
    }
    return ESP_OK;
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    assert(evt != NULL);

    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        if (!esp_http_client_is_chunked_response(evt->client))
        {
            // ghi data
            ESP_LOGD(TAG, "%.*s", evt->data_len, (char *)evt->data);
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

/**
 * @brief Main application task, it sends counter value to ThingsBoard telemetry MQTT topic.
 *
 * @param pvParameters con trỏ chỉ đến các arguments
 */


/**
 * @brief kiểm tra nhãn phân chia hiện tại của vùng chứa
 *
 * @param running_partition_label nhãn phân chia hiện hành
 * @param config_name thay đổi config to Kconfig Thingsboard
 * @return true nếu nhãn hiện tại là 'factory'
 * @return false nếu nhãn hiện tại không phải là 'factory'
 */
static bool partition_is_factory(const char *running_partition_label, const char *config_name)
{
    if (strcmp(FACTORY_PARTITION_LABEL, running_partition_label) == 0)
    {
        ESP_LOGW(TAG, "Factory partition is running. %s from config is saving to the flash memory", config_name);
        return true;
    }
    else
    {
        ESP_LOGE(TAG, "%s wasn't found, running partition is not '%s'", config_name, FACTORY_PARTITION_LABEL);
        APP_ABORT_ON_ERROR(ESP_FAIL);
        return false;
    }
}

/**
 * @brief Get MQTT broker URL to use in MQTT client.
 *        If the flash memory is empty and running partition is 'factory'
 *        then MQTT broker URL specified in ThingsBoard OTA configuration submenu will be saved to NVS.
 *        If running partition is not 'factory' ('ota_0' or 'ota_1') then MQTT broker URL from NVS is used.
 *        The application stops if running partition is not 'factory' and MQTT broker URL was not found in NVS.
 *
 * @param running_partition_label Current running partition label
 * @return const char* MQTT broker URL
 */
static const char *get_mqtt_url(const char *running_partition_label)
{
    nvs_handle handle;
    APP_ABORT_ON_ERROR(nvs_open(NVS_KEY_MQTT_URL, NVS_READWRITE, &handle));

    static char mqtt_url[MAX_LENGTH_TB_URL + 1];
    size_t mqtt_url_len = sizeof(mqtt_url);

    esp_err_t result_code = nvs_get_str(handle, NVS_KEY_MQTT_URL, mqtt_url, &mqtt_url_len);
    if (result_code == ESP_OK)
    {
        ESP_LOGI(TAG, "MQTT URL from flash memory: %s", mqtt_url);
    }
    else if (result_code == ESP_ERR_NVS_NOT_FOUND)
    {
        if (partition_is_factory(running_partition_label, "MQTT URL"))
        {
            APP_ABORT_ON_ERROR(nvs_set_str(handle, NVS_KEY_MQTT_URL, CONFIG_MQTT_BROKER_URL));
            APP_ABORT_ON_ERROR(nvs_commit(handle));
            APP_ABORT_ON_ERROR(nvs_get_str(handle, NVS_KEY_MQTT_URL, mqtt_url, &mqtt_url_len));
        }
    }
    else
    {
        ESP_LOGE(TAG, "Unable to to get MQTT URL from NVS");
        APP_ABORT_ON_ERROR(ESP_FAIL);
    }

    nvs_close(handle);
    return mqtt_url;
}

/**
 * @brief Get MQTT broker port to use in MQTT client.
 *        If the flash memory is empty and running partition is 'factory'
 *        then MQTT broker port specified in ThingsBoard OTA configuration submenu will be saved to NVS.
 *        If running partition is not 'factory' ('ota_0' or 'ota_1') then MQTT broker port from NVS is used.
 *        The application stops if running partition is not 'factory' and MQTT broker port was not found in NVS.
 *
 * @param running_partition_label Current running partition label
 * @return const char* MQTT broker port
 */
static uint32_t get_mqtt_port(const char *running_partition_label)
{
    nvs_handle handle;
    APP_ABORT_ON_ERROR(nvs_open(NVS_KEY_MQTT_PORT, NVS_READWRITE, &handle));

    uint32_t mqtt_port;

    esp_err_t result_code = nvs_get_u32(handle, NVS_KEY_MQTT_PORT, &mqtt_port);
    if (result_code == ESP_OK)
    {
        ESP_LOGI(TAG, "MQTT port from flash memory: %d", mqtt_port);
    }
    else if (result_code == ESP_ERR_NVS_NOT_FOUND)
    {
        if (partition_is_factory(running_partition_label, "MQTT port"))
        {
            APP_ABORT_ON_ERROR(nvs_set_u32(handle, NVS_KEY_MQTT_PORT, CONFIG_MQTT_BROKER_PORT));
            APP_ABORT_ON_ERROR(nvs_commit(handle));
            APP_ABORT_ON_ERROR(nvs_get_u32(handle, NVS_KEY_MQTT_PORT, &mqtt_port));
        }
    }
    else
    {
        ESP_LOGE(TAG, "Unable to to get MQTT port from NVS");
        APP_ABORT_ON_ERROR(ESP_FAIL);
    }

    nvs_close(handle);
    return mqtt_port;
}

/**
 * @brief Get MQTT access token to use in MQTT client.
 *        If the flash memory is empty and running partition is 'factory'
 *        then MQTT broker access token specified in ThingsBoard OTA configuration submenu will be saved to NVS.
 *        If running partition is not 'factory' ('ota_0' or 'ota_1') then MQTT broker access token from NVS is used.
 *        The application stops if running partition is not 'factory' and MQTT broker access token was not found in NVS.
 *
 * @param running_partition_label Current running partition label
 * @return const char* MQTT broker access token
 */
static const char *get_mqtt_access_token(const char *running_partition_label)
{
    nvs_handle handle;
    APP_ABORT_ON_ERROR(nvs_open(NVS_KEY_MQTT_ACCESS_TOKEN, NVS_READWRITE, &handle));

    static char access_token[MAX_LENGTH_TB_ACCESS_TOKEN + 1];
    size_t access_token_len = sizeof(access_token);

    esp_err_t result_code = nvs_get_str(handle, NVS_KEY_MQTT_ACCESS_TOKEN, access_token, &access_token_len);
    if (result_code == ESP_OK)
    {
        ESP_LOGI(TAG, "MQTT access token from flash memory: %s", access_token);
    }
    else if (result_code == ESP_ERR_NVS_NOT_FOUND)
    {
        if (partition_is_factory(running_partition_label, "MQTT access token"))
        {
            APP_ABORT_ON_ERROR(nvs_set_str(handle, NVS_KEY_MQTT_ACCESS_TOKEN, CONFIG_MQTT_ACCESS_TOKEN));
            APP_ABORT_ON_ERROR(nvs_commit(handle));
            APP_ABORT_ON_ERROR(nvs_get_str(handle, NVS_KEY_MQTT_ACCESS_TOKEN, access_token, &access_token_len));
        }
    }
    else
    {
        ESP_LOGE(TAG, "Unable to to get MQTT access token from NVS");
        APP_ABORT_ON_ERROR(ESP_FAIL);
    }

    nvs_close(handle);
    return access_token;
}

static void mqtt_app_start(const char *running_partition_label)
{
    assert(running_partition_label != NULL);

    const char *mqtt_url = get_mqtt_url(running_partition_label);
    const uint32_t mqtt_port = get_mqtt_port(running_partition_label);
    const char *mqtt_access_token = get_mqtt_access_token(running_partition_label);

    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = mqtt_url,
        .event_handle = mqtt_event_handler,
        .port = mqtt_port,
        .username = mqtt_access_token
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    APP_ABORT_ON_ERROR(esp_mqtt_client_start(mqtt_client));

    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

static bool fw_versions_are_equal(const char *current_ver, const char *target_ver)
{
    assert(current_ver != NULL && target_ver != NULL);

    if (strcmp(current_ver, target_ver) == 0)
    {
        ESP_LOGW(TAG, "Skipping OTA, firmware versions are equal - current: %s, target: %s", FIRMWARE_VERSION, shared_attributes.targetFwVer);
        return true;
    }
    return false;
}

static bool ota_params_are_specified(struct shared_keys ota_config)
{
    if (strlen(ota_config.targetFwServerUrl) == 0)
    {
        ESP_LOGW(TAG, "Firmware URL is not specified");
        return false;
    }

    if (strlen(ota_config.targetFwVer) == 0)
    {
        ESP_LOGW(TAG, "Target firmware version is not specified");
        return false;
    }

    return true;
}

static void start_ota(const char *current_ver, struct shared_keys ota_config)
{
    assert(current_ver != NULL);

    if (!fw_versions_are_equal(current_ver, ota_config.targetFwVer) && ota_params_are_specified(ota_config))
    {
        ESP_LOGW(TAG, "Starting OTA, firmware versions are different - current: %s, target: %s", current_ver, ota_config.targetFwVer);
        ESP_LOGI(TAG, "Target firmware version: %s", ota_config.targetFwVer);
        ESP_LOGI(TAG, "Firmware URL: %s", ota_config.targetFwServerUrl);
        esp_http_client_config_t config = {
            .url = ota_config.targetFwServerUrl,
            .cert_pem = (char *)server_cert_pem_start,
            .event_handler = _http_event_handler,
        };
        esp_err_t ret = esp_https_ota(&config);
        if (ret == ESP_OK)
        {
            esp_restart();
        }
        else
        {
            ESP_LOGE(TAG, "Firmware Upgrades Failed");
        }
    }
}

static enum state connection_state(BaseType_t actual_event, const char *current_state_name)
{
    assert(current_state_name != NULL);

    if (actual_event & WIFI_DISCONNECTED_EVENT)
    {
        ESP_LOGE(TAG, "%s state, Wi-Fi not connected, wait for the connect", current_state_name);
        return STATE_WAIT_WIFI;
    }

    if (actual_event & MQTT_DISCONNECTED_EVENT)
    {
        ESP_LOGW(TAG, "%s state, MQTT not connected, wait for the connect", current_state_name);
        return STATE_WAIT_MQTT;
    }

    return STATE_CONNECTION_IS_OK;
}

/**
 * @brief OTA task, it handles the shared attributes updates and starts OTA if the config received from ThingsBoard is valid.
 *
 * @param pvParameters Pointer to the task arguments
 */
static void ota_task(void *pvParameters)
{
    enum state current_connection_state = STATE_CONNECTION_IS_OK;
    enum state state = STATE_INITIAL;
    BaseType_t ota_events;
    BaseType_t actual_event = 0x00;
    char running_partition_label[sizeof(((esp_partition_t *)0)->label)];

    while (1)
    {
        if (state != STATE_INITIAL && state != STATE_APP_LOOP)
        {
            if (state != STATE_APP_LOOP)
            {
                xEventGroupClearBits(event_group, OTA_TASK_IN_NORMAL_STATE_EVENT);
            }

            actual_event = xEventGroupWaitBits(event_group,
                                               WIFI_CONNECTED_EVENT | WIFI_DISCONNECTED_EVENT | MQTT_CONNECTED_EVENT | MQTT_DISCONNECTED_EVENT | OTA_CONFIG_FETCHED_EVENT,
                                               false, false, portMAX_DELAY);
        }

        switch (state)
        {
        case STATE_INITIAL:
        {
            // Initialize NVS.
            esp_err_t err = nvs_flash_init();
            if (err == ESP_ERR_NVS_NO_FREE_PAGES)
            {
                // OTA app partition table has a smaller NVS partition size than the non-OTA
                // partition table. This size mismatch may cause NVS initialization to fail.
                // If this happens, we erase NVS partition and initialize NVS again.
                APP_ABORT_ON_ERROR(nvs_flash_erase());
                err = nvs_flash_init();
            }
            APP_ABORT_ON_ERROR(err);

            const esp_partition_t *running_partition = esp_ota_get_running_partition();
            strncpy(running_partition_label, running_partition->label, sizeof(running_partition_label));
            ESP_LOGI(TAG, "Running partition: %s", running_partition_label);

            initialise_wifi(running_partition_label);
            state = STATE_WAIT_WIFI;
            break;
        }
        case STATE_WAIT_WIFI:
        {
            if (actual_event & WIFI_DISCONNECTED_EVENT)
            {
                ESP_LOGW(TAG, "WAIT_WIFI state, Wi-Fi not connected, wait for the connect");
                state = STATE_WAIT_WIFI;
                break;
            }

            if (actual_event & WIFI_CONNECTED_EVENT)
            {
                mqtt_app_start(running_partition_label);
                state = STATE_WAIT_MQTT;
                break;
            }

            ESP_LOGE(TAG, "WAIT_WIFI state, unexpected event received: %d", actual_event);
            state = STATE_INITIAL;
            break;
        }
        case STATE_WAIT_MQTT:
        {
            current_connection_state = connection_state(actual_event, "WAIT_MQTT");
            if (current_connection_state != STATE_CONNECTION_IS_OK)
            {
                state = current_connection_state;
                break;
            }

            if (actual_event & (WIFI_CONNECTED_EVENT | MQTT_CONNECTED_EVENT))
            {
                ESP_LOGI(TAG, "Connected to MQTT broker %s, on port %d", CONFIG_MQTT_BROKER_URL, CONFIG_MQTT_BROKER_PORT);

                // Send the current firmware version to ThingsBoard
                cJSON *current_fw = cJSON_CreateObject();
                cJSON_AddStringToObject(current_fw, TB_CLIENT_ATTR_FIELD_CURRENT_FW, FIRMWARE_VERSION);
                char *current_fw_attribute = cJSON_PrintUnformatted(current_fw);
                cJSON_Delete(current_fw);
                esp_mqtt_client_publish(mqtt_client, TB_ATTRIBUTES_TOPIC, current_fw_attribute, 0, 1, 0);
                // Free is intentional, it's client responsibility to free the result of cJSON_Print
                free(current_fw_attribute);

                // Send the shared attributes keys to receive their values
                esp_mqtt_client_subscribe(mqtt_client, TB_ATTRIBUTES_SUBSCRIBE_TO_RESPONSE_TOPIC, 1);
                esp_mqtt_client_publish(mqtt_client, TB_ATTRIBUTES_REQUEST_TOPIC, TB_SHARED_ATTR_KEYS_REQUEST, 0, 1, 0);
                ESP_LOGI(TAG, "Waiting for shared attributes response");

                state = STATE_WAIT_OTA_CONFIG_FETCHED;
                break;
            }

            ESP_LOGE(TAG, "WAIT_MQTT state, unexpected event received: %d", actual_event);
            state = STATE_INITIAL;
            break;
        }
        case STATE_WAIT_OTA_CONFIG_FETCHED:
        {
            current_connection_state = connection_state(actual_event, "WAIT_OTA_CONFIG_FETCHED");
            if (current_connection_state != STATE_CONNECTION_IS_OK)
            {
                state = current_connection_state;
                break;
            }

            if (actual_event & (WIFI_CONNECTED_EVENT | MQTT_CONNECTED_EVENT))
            {
                if (actual_event & OTA_CONFIG_FETCHED_EVENT)
                {
                    ESP_LOGI(TAG, "Shared attributes were fetched from ThingsBoard");
                    xEventGroupClearBits(event_group, OTA_CONFIG_FETCHED_EVENT);
                    state = STATE_OTA_CONFIG_FETCHED;
                    break;
                }

                state = STATE_WAIT_OTA_CONFIG_FETCHED;
                break;
            }

            ESP_LOGE(TAG, "WAIT_OTA_CONFIG_FETCHED state, unexpected event received: %d", actual_event);
            state = STATE_INITIAL;
            break;
        }
        case STATE_OTA_CONFIG_FETCHED:
        {
            current_connection_state = connection_state(actual_event, "OTA_CONFIG_FETCHED");
            if (current_connection_state != STATE_CONNECTION_IS_OK)
            {
                state = current_connection_state;
                break;
            }

            if (actual_event & (WIFI_CONNECTED_EVENT | MQTT_CONNECTED_EVENT))
            {

                start_ota(FIRMWARE_VERSION, shared_attributes);
                esp_mqtt_client_subscribe(mqtt_client, TB_ATTRIBUTES_TOPIC, 1);
                ESP_LOGI(TAG, "Subscribed to shared attributes updates");
                state = STATE_APP_LOOP;
                break;
            }
            ESP_LOGE(TAG, "OTA_CONFIG_FETCHED state, unexpected event received: %d", actual_event);
            state = STATE_INITIAL;
            break;
        }
        case STATE_APP_LOOP:
        {
            current_connection_state = connection_state(actual_event, "APP_LOOP");
            if (current_connection_state != STATE_CONNECTION_IS_OK)
            {
                state = current_connection_state;
                break;
            }

            if (actual_event & (WIFI_CONNECTED_EVENT | MQTT_CONNECTED_EVENT))
            {
                ota_events = xEventGroupWaitBits(event_group, OTA_CONFIG_UPDATED_EVENT, false, true, 0);
                if ((ota_events & OTA_CONFIG_UPDATED_EVENT))
                {
                    start_ota(FIRMWARE_VERSION, shared_attributes);
                }
                xEventGroupClearBits(event_group, OTA_CONFIG_UPDATED_EVENT);
                xEventGroupSetBits(event_group, OTA_TASK_IN_NORMAL_STATE_EVENT);
                state = STATE_APP_LOOP;
                break;
            }

            ESP_LOGE(TAG, "APP_LOOP state, unexpected event received: %d", actual_event);
            state = STATE_INITIAL;
            break;
        }
        default:
        {
            ESP_LOGE(TAG, "Unexpected state");
            state = STATE_INITIAL;
            break;
        }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void readDHT11(gpio_num_t gpio_num){
    DHT11_init(gpio_num);
    uint8_t counter = 0;
    while (1)
    {
        uint8_t t=DHT11_read().temperature;
        uint8_t h=DHT11_read().humidity;


        xEventGroupWaitBits(event_group, OTA_TASK_IN_NORMAL_STATE_EVENT, false, true, portMAX_DELAY);
        cJSON *temp = cJSON_CreateObject();
        cJSON_AddNumberToObject(temp, "teamp", t);
        char *post_temp = cJSON_PrintUnformatted(temp);
        esp_mqtt_client_publish(mqtt_client, TB_TELEMETRY_TOPIC, post_temp, 0, 1, 0);
        cJSON_Delete(temp);
        free(post_temp);

        cJSON *hum = cJSON_CreateObject();
        cJSON_AddNumberToObject(hum, "hum", h);
        char *post_hum = cJSON_PrintUnformatted(hum);
        esp_mqtt_client_publish(mqtt_client, TB_TELEMETRY_TOPIC, post_hum, 0, 1, 0);
        cJSON_Delete(hum);
        free(post_hum);

        counter = counter < 1 ? counter + 1 : 0;

        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "counter", counter);
        char *post_data = cJSON_PrintUnformatted(root);
        esp_mqtt_client_publish(mqtt_client, TB_TELEMETRY_TOPIC, post_data, 0, 1, 0);
        cJSON_Delete(root);
        // Free is intentional, it's client responsibility to free the result of cJSON_Print
        free(post_data);



        printf("Temperature is %d \n", t);
        printf("Humidity is %d\n", h);
        vTaskDelay(1000 / portTICK_PERIOD_MS);       
    }
}


void app_main()
{
    event_group = xEventGroupCreate();
    xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
    readDHT11(GPIO_NUM_4);    /* code */ 
}

void notify_wifi_connected()
{
    xEventGroupClearBits(event_group, WIFI_DISCONNECTED_EVENT);
    xEventGroupSetBits(event_group, WIFI_CONNECTED_EVENT);
}

void notify_wifi_disconnected()
{
    xEventGroupClearBits(event_group, WIFI_CONNECTED_EVENT);
    xEventGroupSetBits(event_group, WIFI_DISCONNECTED_EVENT);
}
