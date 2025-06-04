#include "Max31855Thermocouple.h" // Use the new header name
#include <Arduino.h>              // For ESP_LOGI, millis etc.
#include "esp_log.h"              // For ESP32 logging

const char* Max31855Thermocouple::TAG_V2_STUB = "Max31855Stub";

Max31855Thermocouple::Max31855Thermocouple(
    int csPin, int misoPin, int sckPin,
    const temp_sensor_update_callback_t &update_cb,
    const temp_sensor_error_callback_t &error_cb)
    : m_update_callback(update_cb),
      m_error_callback(error_cb) {
    // csPin, misoPin, sckPin are passed but not used in this stub version
    ESP_LOGW(TAG_V2_STUB, "Constructor: MAX31855 is STUBBED OUT. CS Pin %d (not used).", csPin);
    // Do not create MAX31855 library instance:
    // m_max31855_lib_instance = nullptr;
}

Max31855Thermocouple::~Max31855Thermocouple() {
    ESP_LOGW(TAG_V2_STUB, "Destructor: MAX31855 STUBBED OUT.");
    // No task to delete
    // No library instance to delete
}

void Max31855Thermocouple::setup() {
    ESP_LOGW(TAG_V2_STUB, "Setup: MAX31855 STUBBED OUT. No hardware initialization, no task created.");
    // Do absolutely nothing related to hardware or tasks.
    // The main system will call this, but it will be a no-op.
}

float Max31855Thermocouple::read() {
    // Always return a fixed token temperature
    // ESP_LOGV(TAG_V2_STUB, "Read: Returning STUBBED temperature 20.0 C");
    if (m_update_callback) { // Optionally call update callback with the stubbed value
        m_update_callback(20.0f);
    }
    return 20.0f;
}

bool Max31855Thermocouple::hasError() {
    // Always return no error
    // ESP_LOGV(TAG_V2_STUB, "HasError: Returning STUBBED value false");
    return false;
}

// staticTaskRunner and taskFunction are not needed as no task is created.
// If they were still in the .h, provide empty definitions or remove them.
// void Max31855Thermocouple::staticTaskRunner(void *param) { /* no-op */ }
// void Max31855Thermocouple::taskFunction() { /* no-op */ }

