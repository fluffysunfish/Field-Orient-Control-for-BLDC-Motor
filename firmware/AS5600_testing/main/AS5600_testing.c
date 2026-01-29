/**
 * @file AS5600_testing.c
 * @brief AS5600 Angle Sensor Testing Application
 *
 * This application demonstrates how to:
 * - Initialize I2C bus
 * - Initialize AS5600 magnetic rotary position sensor
 * - Read angle values continuously
 * - Display results in degrees and raw values
 * - Check magnet status
 *
 * Hardware Setup:
 * - AS5600 SDA -> ESP32 GPIO 21 (or change SDA_GPIO below)
 * - AS5600 SCL -> ESP32 GPIO 22 (or change SCL_GPIO below)
 * - AS5600 VCC -> 3.3V or 5V
 * - AS5600 GND -> GND
 * - Place magnet above AS5600 sensor (centered on axis)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "as5600.h"

static const char *TAG = "AS5600_TEST";

// I2C Configuration
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define SDA_GPIO                    21
#define SCL_GPIO                    22

// Update rate
#define UPDATE_RATE_MS              500

/**
 * @brief Initialize I2C master interface
 */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

/**
 * @brief Check and display magnet status
 */
static void check_magnet_status(as5600_handle_t *as5600)
{
    bool magnet_detected = false;
    bool magnet_too_strong = false;
    bool magnet_too_weak = false;
    uint16_t magnitude = 0;
    uint8_t agc = 0;

    // Check magnet detection
    if (as5600_is_magnet_detected(as5600, &magnet_detected) == ESP_OK) {
        if (magnet_detected) {
            ESP_LOGI(TAG, "✓ Magnet detected");
        } else {
            ESP_LOGW(TAG, "✗ No magnet detected!");
        }
    }

    // Check magnet strength
    if (as5600_is_magnet_too_strong(as5600, &magnet_too_strong) == ESP_OK && magnet_too_strong) {
        ESP_LOGW(TAG, "! Magnet too strong - move it farther away");
    }

    if (as5600_is_magnet_too_weak(as5600, &magnet_too_weak) == ESP_OK && magnet_too_weak) {
        ESP_LOGW(TAG, "! Magnet too weak - move it closer");
    }

    // Display magnitude and AGC
    if (as5600_get_magnitude(as5600, &magnitude) == ESP_OK) {
        ESP_LOGI(TAG, "Magnitude: %d", magnitude);
    }

    if (as5600_get_gain(as5600, &agc) == ESP_OK) {
        ESP_LOGI(TAG, "AGC: %d", agc);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "AS5600 Angle Sensor Testing");
    ESP_LOGI(TAG, "==================================\n");

    // Initialize I2C
    ESP_LOGI(TAG, "Initializing I2C...");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "✓ I2C initialized (SDA: GPIO%d, SCL: GPIO%d)\n", SDA_GPIO, SCL_GPIO);

    // Initialize AS5600
    ESP_LOGI(TAG, "Initializing AS5600 sensor...");
    as5600_config_t as5600_config = {
        .i2c_port = I2C_MASTER_NUM,
        .dev_addr = AS5600_I2C_ADDRESS,
    };

    as5600_handle_t as5600;
    ESP_ERROR_CHECK(as5600_init(&as5600_config, &as5600));
    ESP_LOGI(TAG, "✓ AS5600 initialized (I2C Address: 0x%02X)\n", AS5600_I2C_ADDRESS);

    // Initial magnet status check
    ESP_LOGI(TAG, "Checking magnet status...");
    check_magnet_status(&as5600);
    ESP_LOGI(TAG, "");

    // Wait a bit before starting continuous readings
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Starting continuous angle readings...\n");

    // Main loop - read angle continuously
    uint32_t reading_count = 0;
    while (1) {
        float degrees;
        uint16_t raw_angle;
        uint16_t scaled_angle;
        esp_err_t ret;

        reading_count++;

        // Get angle in degrees (0-360)
        ret = as5600_get_degrees(&as5600, &degrees);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read degrees: %s", esp_err_to_name(ret));
        }

        // Get raw angle value (0-4095, 12-bit)
        ret = as5600_get_raw_angle(&as5600, &raw_angle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read raw angle: %s", esp_err_to_name(ret));
        }

        // Get scaled angle value (0-4095, with zero position applied)
        ret = as5600_get_angle(&as5600, &scaled_angle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read scaled angle: %s", esp_err_to_name(ret));
        }

        // Display readings
        ESP_LOGI(TAG, "[%lu] Angle: %.2f° | Raw: %4d | Scaled: %4d",
                 reading_count, degrees, raw_angle, scaled_angle);

        // Periodically check magnet status (every 10 readings)
        if (reading_count % 10 == 0) {
            ESP_LOGI(TAG, "--- Status Check ---");
            check_magnet_status(&as5600);
            ESP_LOGI(TAG, "");
        }

        vTaskDelay(pdMS_TO_TICKS(UPDATE_RATE_MS));
    }
}
