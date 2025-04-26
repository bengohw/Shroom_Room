/* Temperature Sensor Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*
* Modified by: Benjamin Goh
* Modified on: 23 April 2025
*/

// Include necessary components
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>

#include <app_network.h>
#include <app_insights.h>

#include "app_priv.h"

static const char *TAG = "app_main";

esp_rmaker_device_t *temp_sensor_device;
esp_rmaker_device_t *humidity_sensor_device;
esp_rmaker_device_t *co2_sensor_device;
esp_rmaker_device_t *fan_device;
esp_rmaker_device_t *humidifier_device;
esp_rmaker_device_t *switch_device; // controls whether the monotub is in auto or manual mode

/* Callback to handle commands received from the RainMaker cloud */
static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
    const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (ctx) {
        ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }
    const char *device_name = esp_rmaker_device_get_name(device);
    const char *param_name = esp_rmaker_param_get_name(param);
    if (strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME) == 0) { 
        ESP_LOGI(TAG, "Received value = %s for %s - %s", // for debugging
            val.val.b? "true" : "false", device_name, param_name);
        app_driver_set_state(device_name, val.val.b); // callback function to set relevant device state
    } else {
        /* Silently ignoring invalid params */
        return ESP_OK;
    }
    esp_rmaker_param_update(param, val);
    return ESP_OK;
}

// void app_main() must be defined because it acts as the entry point for the user's application
void app_main()
{
    /* Initialise Application specific hardware drivers and
     * set initial state.
     */
    app_driver_init();

    /* Initialise NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    /* Initialise Wi-Fi. Note that, this should be called before esp_rmaker_node_init()
     */
    app_network_init();
    
    /* Initialise the ESP RainMaker Agent.
     * Note that this should be called after app_network_init() but before app_network_start()
     * */
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "Shroom Room :D", "Automated Monotub");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    /* Create the sensor devices and add the relevant parameters*/
    temp_sensor_device = esp_rmaker_temp_sensor_device_create("Temperature Sensor", NULL, 0);
    esp_rmaker_node_add_device(node, temp_sensor_device);

    humidity_sensor_device = esp_rmaker_temp_sensor_device_create("Humidity Sensor", NULL, 0);
    esp_rmaker_node_add_device(node, humidity_sensor_device);

    co2_sensor_device = esp_rmaker_temp_sensor_device_create("CO2 Sensor", NULL, 0);
    esp_rmaker_node_add_device(node, co2_sensor_device);

    /* Create the actuator devices (fan and humidifier) and add the relevant parameters*/
    switch_device = esp_rmaker_switch_device_create("Auto Mode", NULL, DEFAULT_SWITCH_POWER);
    esp_rmaker_device_add_cb(switch_device, write_cb, NULL);
    esp_rmaker_node_add_device(node, switch_device);
    
    fan_device = esp_rmaker_switch_device_create("Fan", NULL, DEFAULT_FAN_POWER);
    esp_rmaker_device_add_cb(fan_device, write_cb, NULL);
    esp_rmaker_node_add_device(node, fan_device);

    humidifier_device = esp_rmaker_switch_device_create("Humidifier", NULL, DEFAULT_HUMIDIFIER_POWER);
    esp_rmaker_device_add_cb(humidifier_device, write_cb, NULL);
    esp_rmaker_node_add_device(node, humidifier_device);

    /* Enable OTA */
    esp_rmaker_ota_enable_default();

    /* Enable Insights. Requires CONFIG_ESP_INSIGHTS_ENABLED=y */
    app_insights_enable();

    /* Start the ESP RainMaker Agent */
    esp_rmaker_start();

    /* Start the Wi-Fi.
     * If the node is provisioned, it will start connection attempts,
     * else, it will start Wi-Fi provisioning. The function will return
     * after a connection has been successfully established
     */
    err = app_network_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wifi. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }
}
