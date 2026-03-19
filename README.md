#include <Arduino.h>
#include <NimBLEDevice.h>
#include <stdio.h> 
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h" 
#include "driver/gpio.h" 
#include "esp_log.h" 
#include "esp_err.h" 
#include "sdkconfig.h" 
#include <stdint.h> 
#include <inttypes.h> 
#include <string.h> 
#include <ctype.h> 
#include <limits.h> 
#include "nvs_flash.h" 
#include "driver/i2s.h" 

static const char *TAG = "inmp441_ctrl"; 

static bool light_state = false; 
static bool fan_state = false; 

#ifndef CONFIG_LIGHT_GPIO 
#define CONFIG_LIGHT_GPIO 2 
#endif 
#ifndef CONFIG_FAN_GPIO 
#define CONFIG_FAN_GPIO 4 
#endif 
#ifndef CONFIG_LIGHT_RELAY_ACTIVE_LOW 
#define CONFIG_LIGHT_RELAY_ACTIVE_LOW 0 
#endif 
#ifndef CONFIG_FAN_RELAY_ACTIVE_LOW 
#define CONFIG_FAN_RELAY_ACTIVE_LOW 0 
#endif 
#ifndef CONFIG_INMP441_BCLK 
#define CONFIG_INMP441_BCLK 26 
#endif 
#ifndef CONFIG_INMP441_LRCLK 
#define CONFIG_INMP441_LRCLK 25 
#endif 
#ifndef CONFIG_INMP441_DIN 
#define CONFIG_INMP441_DIN 35 
#endif 

static void set_outputs(void) 
{ 
    int light_level = light_state ? (CONFIG_LIGHT_RELAY_ACTIVE_LOW ? 0 : 1) 
                                   : (CONFIG_LIGHT_RELAY_ACTIVE_LOW ? 1 : 0); 
    int fan_level = fan_state ? (CONFIG_FAN_RELAY_ACTIVE_LOW ? 0 : 1) 
                               : (CONFIG_FAN_RELAY_ACTIVE_LOW ? 1 : 0); 
    gpio_set_level((gpio_num_t)CONFIG_LIGHT_GPIO, light_level); 
    gpio_set_level((gpio_num_t)CONFIG_FAN_GPIO, fan_level); 
} 

static void outputs_init(void) 
{ 
    gpio_reset_pin((gpio_num_t)CONFIG_LIGHT_GPIO); 
    gpio_set_direction((gpio_num_t)CONFIG_LIGHT_GPIO, GPIO_MODE_OUTPUT); 
    gpio_reset_pin((gpio_num_t)CONFIG_FAN_GPIO); 
    gpio_set_direction((gpio_num_t)CONFIG_FAN_GPIO, GPIO_MODE_OUTPUT); 
    set_outputs(); 
} 

static void i2s_init(void) 
{ 
    i2s_config_t i2s_config = { 
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX), 
        .sample_rate = 16000, 
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, 
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, 
        .communication_format = I2S_COMM_FORMAT_STAND_I2S, 
        .intr_alloc_flags = 0, 
        .dma_buf_count = 8, 
        .dma_buf_len = 1024, 
        .use_apll = false, 
        .tx_desc_auto_clear = false, 
        .fixed_mclk = 0, 
    }; 
    i2s_pin_config_t pin_config = { 
        .mck_io_num = I2S_PIN_NO_CHANGE, 
        .bck_io_num = CONFIG_INMP441_BCLK, 
        .ws_io_num = CONFIG_INMP441_LRCLK, 
        .data_out_num = I2S_PIN_NO_CHANGE, 
        .data_in_num = CONFIG_INMP441_DIN, 
    }; 
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL); 
    i2s_set_pin(I2S_NUM_0, &pin_config); 
} 

static void mic_task(void *arg) 
{ 
    int32_t buf[512]; 
    TickType_t last = xTaskGetTickCount(); 
    while (1) { 
        size_t br = 0; 
        i2s_read(I2S_NUM_0, (void *)buf, sizeof(buf), &br, portMAX_DELAY); 
        if (br == 0) continue; 
        int n = br / sizeof(int32_t); 
        int32_t peak = 0; 
        for (int i = 0; i < n; i++) { 
            int32_t s = buf[i]; 
            int32_t a = s < 0 ? -s : s; 
            if (a > peak) peak = a; 
        } 
        TickType_t now = xTaskGetTickCount(); 
        if (now - last > pdMS_TO_TICKS(1000)) { 
            ESP_LOGI(TAG, "Mic peak=%" PRId32, peak); 
            last = now; 
        } 
    } 
} 

static void normalize_ascii(char *dst, const char *src) 
{ 
    int j = 0; 
    for (int i = 0; src[i]; i++) { 
        unsigned char c = (unsigned char)src[i]; 
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue; 
        if (c < 128) { 
            char x = (char)c; 
            if (x >= 'A' && x <= 'Z') x = (char)(x - 'A' + 'a'); 
            dst[j++] = x; 
        } else { 
            if (c == 0xC4) { 
                unsigned char c2 = (unsigned char)src[i+1]; 
                if (c2 == 0x91 || c2 == 0x90) { // đ or Đ 
                    dst[j++] = 'd'; 
                    i++; 
                } 
            } 
        } 
    } 
    dst[j] = 0; 
} 

static void apply_text_command(const char *cmd) 
{ 
    char ascii[64]; 
    normalize_ascii(ascii, cmd); 
    ESP_LOGI(TAG, "Lệnh nhận được: %s", ascii);
    if (strstr(ascii, "batden")) { 
        light_state = true; 
        set_outputs(); 
    } else if (strstr(ascii, "tatden")) { 
        light_state = false; 
        set_outputs(); 
    } else if (strstr(ascii, "batquat")) { 
        fan_state = true; 
        set_outputs(); 
    } else if (strstr(ascii, "tatquat")) { 
        fan_state = false; 
        set_outputs(); 
    } 
} 

// Class xử lý sự kiện Bluetooth (Arduino Style)
class MyCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            apply_text_command(value.c_str());
        }
    }
};

void ble_init() {
    NimBLEDevice::init("ESP32-VoiceVI");
    NimBLEServer *pServer = NimBLEDevice::createServer();
    NimBLEService *pService = pServer->createService("FFF0");
    NimBLECharacteristic *pCharacteristic = pService->createCharacteristic(
        "FFF1",
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pCharacteristic->setCallbacks(new MyCallbacks());
    pService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID("FFF0");
    pAdvertising->start();
}

void setup() 
{ 
    Serial.begin(115200);
    ESP_LOGI(TAG, "He thong dang khoi dong (Khong coi)...");
    
    outputs_init(); 
    i2s_init(); 
    
    xTaskCreate(mic_task, "mic_task", 4096, NULL, 4, NULL); 
    ble_init(); 
} 

void loop() 
{ 
    vTaskDelay(pdMS_TO_TICKS(1000));
}
