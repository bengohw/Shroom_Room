/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*
* Modified by: Benjamin Goh
* Modified on: 23 April 2025
*/

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h> // to use functions such as strcmp() in app_main.c and app_driver.c

#define DEFAULT_SWITCH_POWER     1 // auto mode activated
#define DEFAULT_FAN_POWER        0 // fan is off
#define DEFAULT_HUMIDIFIER_POWER 0 // humidifier is off

// Threshold values macros for automation
#define CO2_UPPER_THRESH   900 // in ppm, ideally under 1000ppm during fruiting
#define HUMIDITY_UPPER_THRESH    95 // in %, when above - need to lower humidity
#define HUMIDITY_LOWER_THRESH    85 // in %, when below - need to increase humidity

// Timing macros
#define REPORTING_PERIOD    60    // seconds (how frequently the sensor reads)
#define HUMIDIFIER_ON_DURATION 5 // seconds (on humidifier if < HUMIDITY_LOWER_THRESH)
#define FAN_ON_DURATION 5        // seconds (on fan if > HUMIDITY_UPPER_THRESH or > CO2_UPPER_THRESH)

// Actuator GPIOs
#define FAN_PIN         GPIO_NUM_18 // ESP32 GPIO that fan is connected to
#define HUMIDIFIER_PIN  GPIO_NUM_21 // ESP32 GPIO that the humidifier is connected to

// extern because these variables have been defined in app_main.c
extern esp_rmaker_device_t *temp_sensor_device; 
extern esp_rmaker_device_t *humidity_sensor_device;
extern esp_rmaker_device_t *co2_sensor_device;
extern esp_rmaker_device_t *fan_device;
extern esp_rmaker_device_t *humidifier_device;

void app_driver_init(void);
int app_driver_set_state(const char *device_name, bool state);
