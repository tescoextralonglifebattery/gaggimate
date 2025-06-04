#include "SimplePump.h"
#include <Arduino.h> // For millis(), pinMode(), digitalWrite()
#include <cmath>     // For fabs()
#include "esp_log.h" // For ESP_LOGI, ESP_LOGD, ESP_LOGW, ESP_LOGE (ensure configured)

// --- DIAGNOSTIC CONTROL ---
// Set to 1 to use simplified ON/OFF logic in SimplePump::loop()
// Set to 0 to use the original PWM logic
#define PUMP_DIAGNOSTIC_SIMPLE_ON_OFF 1

// Define a logging tag for this file (if not already in .h)
static const char *PUMP_LOG_TAG = "SimplePump";

// Ensure pdMS_TO_TICKS is available if not globally defined
#ifndef pdMS_TO_TICKS
#define pdMS_TO_TICKS(xTimeInMs) ((TickType_t)(((uint64_t)(xTimeInMs) * (uint64_t)configTICK_RATE_HZ) / (uint64_t)1000U))
#endif

SimplePump::SimplePump(int pin, uint8_t pumpOn, float windowSize)
    : _pin(pin),
      _pumpOn(pumpOn),
      _setpoint(0.0f), // Default to off
      relayStatus(false),
      _windowSizeMs(static_cast<unsigned long>(windowSize + 0.5f)),
      windowStartTime(0),
      taskHandle(nullptr) { // Type TaskHandle_t as per user's existing code
    if (_windowSizeMs == 0) {
        _windowSizeMs = 1000; // Default to 1 second
        ESP_LOGW(PUMP_LOG_TAG, "Window size was 0, defaulted to 1000ms for pin %d", _pin);
    }
    ESP_LOGI(PUMP_LOG_TAG, "Constructor for pin %d. PWM Window: %lu ms. Diagnostic Simple ON/OFF: %d", _pin, _windowSizeMs, PUMP_DIAGNOSTIC_SIMPLE_ON_OFF);
}

SimplePump::~SimplePump() {
    ESP_LOGI(PUMP_LOG_TAG, "Destructor called for pin %d. Deleting task.", _pin);
    if (taskHandle != nullptr) {
        vTaskDelete(taskHandle);
        taskHandle = nullptr;
    }
    // Ensure pump is off if it was on
    if (relayStatus) {
        digitalWrite(_pin, !_pumpOn);
        ESP_LOGI(PUMP_LOG_TAG, "Pin %d turned OFF in destructor.", _pin);
    }
}

void SimplePump::setup() {
    ESP_LOGI(PUMP_LOG_TAG, "Setup for pin %d.", _pin);
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, !_pumpOn); // Ensure pump is initially OFF
    relayStatus = false;
    ESP_LOGI(PUMP_LOG_TAG, "Pin %d set to OUTPUT. Initial state: OFF (relayStatus=false).", _pin);


    // Increased stack size and priority for diagnostics
    UBaseType_t stackSize = configMINIMAL_STACK_SIZE * 16; // Increased stack
    UBaseType_t priority = 3; // Increased priority

    ESP_LOGI(PUMP_LOG_TAG, "Creating SimplePumpLoop task for pin %d. Stack: %u words, Priority: %u", _pin, (unsigned int)stackSize, (unsigned int)priority);

    BaseType_t taskCreated = xTaskCreate(
        loopTask,
        "SimplePumpLoop",
        stackSize,
        this,
        priority,
        &taskHandle
    );

    if (taskCreated != pdPASS) {
        ESP_LOGE(PUMP_LOG_TAG, "Failed to create SimplePump task for pin %d. Error code: %d", _pin, (int)taskCreated);
        taskHandle = nullptr; // Ensure handle is null on failure
    } else {
        ESP_LOGI(PUMP_LOG_TAG, "SimplePump task created successfully for pin %d.", _pin);
    }
}

void SimplePump::loop() {
#if PUMP_DIAGNOSTIC_SIMPLE_ON_OFF == 1
    // --- DIAGNOSTIC: Simplified ON/OFF Logic ---
    bool shouldBeOn = (_setpoint > 50.0f); // Example: Turn on if setpoint is > 50%

    if (shouldBeOn) {
        if (!relayStatus) {
            // --- DIAGNOSTIC: UNCOMMENTED digitalWrite ---
            digitalWrite(_pin, _pumpOn); 
            ESP_LOGI(PUMP_LOG_TAG, "Pin %d DIAGNOSTIC: Pump ON (digitalWrite executed). Setpoint: %.2f", _pin, _setpoint);
            relayStatus = true; // Update software state
        }
    } else {
        if (relayStatus) {
            // --- DIAGNOSTIC: UNCOMMENTED digitalWrite ---
            digitalWrite(_pin, !_pumpOn);
            ESP_LOGI(PUMP_LOG_TAG, "Pin %d DIAGNOSTIC: Pump OFF (digitalWrite executed). Setpoint: %.2f", _pin, _setpoint);
            relayStatus = false; // Update software state
        }
    }

#else
    // --- ORIGINAL PWM Logic ---
    // (Original PWM logic remains here for when PUMP_DIAGNOSTIC_SIMPLE_ON_OFF is 0)
    if (fabs(_setpoint) < 0.01f) {
        if (relayStatus) {
            digitalWrite(_pin, !_pumpOn);
            relayStatus = false;
            ESP_LOGD(PUMP_LOG_TAG, "Pin %d PWM: Pump OFF (setpoint is zero).", _pin);
        }
        return;
    }

    unsigned long msNow = millis();
    unsigned long elapsedInWindow = msNow - windowStartTime;

    if (elapsedInWindow >= _windowSizeMs) {
        windowStartTime = msNow;
        elapsedInWindow = 0;
    }

    unsigned long outputOnTimeMs = static_cast<unsigned long>((_setpoint / 100.0f) * _windowSizeMs + 0.5f);

    if (outputOnTimeMs > _windowSizeMs) {
        outputOnTimeMs = _windowSizeMs;
    }
    
    if (elapsedInWindow < outputOnTimeMs) {
        if (!relayStatus) {
            digitalWrite(_pin, _pumpOn);
            relayStatus = true;
        }
    } else {
        if (relayStatus) {
            digitalWrite(_pin, !_pumpOn);
            relayStatus = false;
        }
    }
#endif
}

void SimplePump::setPower(float setpoint) {
    float oldSetpoint = _setpoint;
    if (setpoint < 0.0f) {
        _setpoint = 0.0f;
    } else if (setpoint > 100.0f) {
        _setpoint = 100.0f;
    } else {
        _setpoint = setpoint;
    }

    if (fabs(oldSetpoint - _setpoint) > 0.01f) { 
        ESP_LOGI(PUMP_LOG_TAG, "SetPower for pin %d: New setpoint: %.2f (was: %.2f)", _pin, _setpoint, oldSetpoint);
    }

#if PUMP_DIAGNOSTIC_SIMPLE_ON_OFF == 1
    // In simple ON/OFF diagnostic mode, the loop() handles the state based on _setpoint.
    // However, if setpoint becomes 0, we might want to turn it off immediately here too.
    if (fabs(_setpoint) < 0.01f) {
        if (relayStatus) { 
            // --- DIAGNOSTIC: UNCOMMENTED digitalWrite ---
            digitalWrite(_pin, !_pumpOn);
            ESP_LOGI(PUMP_LOG_TAG, "Pin %d DIAGNOSTIC SetPower: Pump OFF immediately (digitalWrite executed). Setpoint: %.2f", _pin, _setpoint);
            relayStatus = false; 
        }
    }
#else
    // Original logic for immediate turn-off if setpoint is zero in PWM mode
    if (fabs(_setpoint) < 0.01f) {
        if (relayStatus) {
            digitalWrite(_pin, !_pumpOn);
            relayStatus = false;
            ESP_LOGD(PUMP_LOG_TAG, "Pin %d PWM: Pump turned OFF immediately by SetPower (setpoint is zero).", _pin);
        }
    }
#endif
}

void SimplePump::loopTask(void *arg) {
    SimplePump *pump = static_cast<SimplePump *>(arg);
    if (!pump) {
        ESP_LOGE(PUMP_LOG_TAG, "loopTask received null argument! Aborting task.");
        vTaskDelete(NULL); 
        return;
    }

    ESP_LOGI(PUMP_LOG_TAG, "loopTask started for pump on pin %d.", pump->_pin);
    unsigned long lastLogTime = 0;
    UBaseType_t HWM_prev = 0;
    unsigned long loopCounter = 0;

    while (true) {
        pump->loop();
        loopCounter++;

        unsigned long currentTime = millis();
        if (currentTime - lastLogTime > 10000) { 
            UBaseType_t HWM = uxTaskGetStackHighWaterMark(NULL); 
            if (HWM != HWM_prev) { 
                 ESP_LOGI(PUMP_LOG_TAG, "Pump Task (Pin %d) Stack HWM: %u words free. Loop count: %lu", pump->_pin, (unsigned int)HWM, loopCounter);
                 HWM_prev = HWM;
            } else {
                 ESP_LOGD(PUMP_LOG_TAG, "Pump Task (Pin %d) tick. Loop count: %lu. HWM unchanged: %u", pump->_pin, loopCounter, (unsigned int)HWM);
            }
            lastLogTime = currentTime;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

