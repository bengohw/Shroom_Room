/*  Temperature Sensor demo implementation using RGB LED and timer

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*
* Modified by: Benjamin Goh
* Modified on: 23 April 2025
*/

#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <sdkconfig.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h> 
#include <esp_rmaker_standard_params.h> 

#include <app_reset.h>
#include "app_priv.h"

#include "scd4x.h" // include the scd4x component
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* This is the button that is used for toggling the power */
#define BUTTON_GPIO          CONFIG_EXAMPLE_BOARD_BUTTON_GPIO
#define BUTTON_ACTIVE_LEVEL  0

static TimerHandle_t sensor_timer;

#define WIFI_RESET_BUTTON_TIMEOUT       3
#define FACTORY_RESET_BUTTON_TIMEOUT    10

#define TEMPERATURE_OFFSET          (0.0) // from calibration, default value is 4.0
#define SENSOR_ALTITUDE             (0)

#define INIT_DELAY 5 // initialisation delay sensor for scd40 in seconds, 
                     // must be at least 5 to avoid errors in sensor readings

static const char *SENSORS_TAG = "sensors";

char scale = SCALE_CELSIUS; // alternative options are SCALE_FAHRENHEIT and SCALE_KELVIN
float temperature = 0.0;
float humidity = 0.0;
float co2_level = 0.0;

static bool switch_power_state = DEFAULT_SWITCH_POWER;
static bool fan_power_state = DEFAULT_FAN_POWER;
static bool humidifier_power_state = DEFAULT_HUMIDIFIER_POWER;

// Used to turn the actuators on/off
static void set_power_state(int output_pin, bool target)
{
    gpio_set_level(output_pin, target);
}

// Code to update SCD40 gas sensor measurements and control actuators in auto mode
static void app_sensor_update(TimerHandle_t handle)
{
    scd4x_start_periodic_measurement(); // from scd4x component to start measurements

    scd4x_sensors_values_t sensors_values = {
        .co2 = 0x00,
        .temperature = 0x00,
        .humidity = 0x00
    };
    
    vTaskDelay(INIT_DELAY * 1000 / portTICK_PERIOD_MS);

    if(scd4x_read_measurement(&sensors_values) != ESP_OK) {
        ESP_LOGE(SENSORS_TAG, "Sensors read measurement error!");
    }

    float temperature = sensors_values.temperature - TEMPERATURE_OFFSET;
    float humidity = sensors_values.humidity;
    float co2_level = sensors_values.co2;

    scd4x_stop_periodic_measurement(); // from scd4x component to stop measurements

    // Update readings to ESP Rainmaker
    esp_rmaker_param_update_and_report(
        esp_rmaker_device_get_param_by_type(temp_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),
        esp_rmaker_float(temperature)
    );

    esp_rmaker_param_update_and_report(
        esp_rmaker_device_get_param_by_type(humidity_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),
        esp_rmaker_float(humidity)
    );

    esp_rmaker_param_update_and_report(
        esp_rmaker_device_get_param_by_type(co2_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),
        esp_rmaker_float(co2_level)
    );

    // For debugging with serial monitor
    ESP_LOGI(SENSORS_TAG, "CO₂ %4.0f ppm - Temperature %2.1f °%c - Humidity %2.1f%%",
        co2_level, temperature, scale, humidity);

    // The following will only run if the monotub is in auto mode
    // Purpose is to let actuators autonomously maintain required CO2 and humidity levels

    // 1. Adjust CO2 first (if switch is in auto mode)
    if (switch_power_state == 1 && co2_level >= CO2_UPPER_THRESH){
        set_power_state(FAN_PIN, 1);
        vTaskDelay(FAN_ON_DURATION * 1000 / portTICK_PERIOD_MS);
        set_power_state(FAN_PIN, 0);
        fan_power_state = 0;
    } 

    // 2. Only adjust humidity after CO2 is below threshold (and if switch is in auto mode)
    if (switch_power_state == 1 && co2_level < CO2_UPPER_THRESH){
        if (humidity >= HUMIDITY_UPPER_THRESH){ // turn on fan to further lower humidity
            set_power_state(FAN_PIN, 1);
            vTaskDelay(FAN_ON_DURATION * 1000 / portTICK_PERIOD_MS);
            set_power_state(FAN_PIN, 0);
            fan_power_state = 0;
        } else if (humidity <= HUMIDITY_LOWER_THRESH) {    // turn on humidifier to increase humidity
            set_power_state(HUMIDIFIER_PIN, 1);
            vTaskDelay(HUMIDIFIER_ON_DURATION * 1000 / portTICK_PERIOD_MS);
            set_power_state(HUMIDIFIER_PIN, 0);
            humidifier_power_state = 0;
        }
    }
}

// This function initialises the necessary configurations for the SCD40 sensor
esp_err_t app_sensor_init(void){
    // Initialisation of I2C and other configurations for sensor
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // enable internal pull-ups
        .scl_io_num = I2C_MASTER_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    // Error checks to ensure that GPIOs are correctly configured for I2C
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, i2c_config.mode,
                    I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

    esp_log_level_set(SENSORS_TAG, ESP_LOG_INFO);

    #if defined(SENSORS_SCALE_F) // Adjusting temperature calculation based on units
    scale = SCALE_FAHRENHEIT;
    #elif defined(SENSORS_SCALE_K)
    scale = SCALE_KELVIN;
    #endif

    // Initialisation tasks for the sensor and to calculate the relevant measurement offsets
    vTaskDelay(INIT_DELAY * 1000 / portTICK_PERIOD_MS);
    ESP_LOGI(SENSORS_TAG, "Sensor serial number 0x%012llX", scd4x_get_serial_number());

    vTaskDelay(INIT_DELAY * 1000 / portTICK_PERIOD_MS);
    float temperature_offset = scd4x_get_temperature_offset();

    vTaskDelay(INIT_DELAY * 1000 / portTICK_PERIOD_MS);
    uint16_t sensor_altitude = scd4x_get_sensor_altitude();

    // Error checks to determine if temperature offset and sensor altitude calculations were successful
    // If the calculations are successful, the relevant temperature offset and altitude values are updated
    // If not successful, the temperature offset and altitude stays at the default values
    if(temperature_offset != SCD41_READ_ERROR && sensor_altitude != SCD41_READ_ERROR) {

        if(temperature_offset != TEMPERATURE_OFFSET) {
            ESP_LOGW(SENSORS_TAG, "Temperature offset calibration from %.1f °%c to %.1f °%c",
                     temperature_offset, scale, TEMPERATURE_OFFSET, scale);

            vTaskDelay(INIT_DELAY * 1000/ portTICK_PERIOD_MS);
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_set_temperature_offset(TEMPERATURE_OFFSET));

            vTaskDelay(INIT_DELAY * 1000 / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_persist_settings());

            vTaskDelay(INIT_DELAY * 1000 / portTICK_PERIOD_MS);
            temperature_offset = scd4x_get_temperature_offset();
        }

        if(sensor_altitude != SENSOR_ALTITUDE) {
            ESP_LOGW(SENSORS_TAG, "Sensor altitude calibration from %d m to %d m",
                     sensor_altitude, SENSOR_ALTITUDE);

            vTaskDelay(INIT_DELAY * 1000 / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_set_sensor_altitude(SENSOR_ALTITUDE));

            vTaskDelay(INIT_DELAY * 1000 / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_persist_settings());

            vTaskDelay(INIT_DELAY * 1000 / portTICK_PERIOD_MS);
            sensor_altitude = scd4x_get_sensor_altitude();
        }
        ESP_LOGI(SENSORS_TAG, "Temperature offset %.1f °%c - Sensor altitude %d %s",
                 temperature_offset, scale, sensor_altitude, scale == SCALE_CELSIUS ? "m" : "ft");
    } else {
        ESP_LOGE(SENSORS_TAG, "Sensor offset/altitude read error!");
    }

    // Creates a software instance timer object that exists independently
    sensor_timer = xTimerCreate(
        "app_sensor_update_tm", 
        (REPORTING_PERIOD * 1000) / portTICK_PERIOD_MS, // timer period
        pdTRUE, 
        NULL, 
        app_sensor_update  // static callback function
    );

    if (sensor_timer) {
        xTimerStart(sensor_timer, 0);
        return ESP_OK;
    }

    return ESP_FAIL;
}

// Initialise the app drivers
void app_driver_init()
{
    app_reset_button_register(app_reset_button_create(BUTTON_GPIO, BUTTON_ACTIVE_LEVEL),
                WIFI_RESET_BUTTON_TIMEOUT, FACTORY_RESET_BUTTON_TIMEOUT);

    // Configure GPIO for actuators to output mode to control mechanical relays
    gpio_reset_pin(FAN_PIN);
    gpio_reset_pin(HUMIDIFIER_PIN);
    
    gpio_set_direction(FAN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(HUMIDIFIER_PIN, GPIO_MODE_OUTPUT);
    
    gpio_set_level(FAN_PIN, DEFAULT_FAN_POWER); 
    gpio_set_level(HUMIDIFIER_PIN, DEFAULT_HUMIDIFIER_POWER);

    // Initialise and configure the SCD40 sensor
    app_sensor_init();
}

/* Callback function to set the states of the auto/manual mode and actuators based 
 * on the user input on Rainmaker. 
 * Each strcmp() function checks which device to adjust based on the user input on the 
 * Rainmaker application.
*/ 
int IRAM_ATTR app_driver_set_state(const char *device_name, bool state)
{
    if (strcmp(device_name, "Auto Mode") == 0 && switch_power_state != state){
        switch_power_state = state;
    } else if (strcmp(device_name, "Fan") == 0 && switch_power_state == 0 && fan_power_state != state){
        fan_power_state = state;
        set_power_state(FAN_PIN, fan_power_state);
    } else if (strcmp(device_name, "Humidifier") == 0 && switch_power_state == 0 && humidifier_power_state != state){
        humidifier_power_state = state;
        set_power_state(HUMIDIFIER_PIN, humidifier_power_state);
    }
    return ESP_OK;
}

