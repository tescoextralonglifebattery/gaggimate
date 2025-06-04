#include "Heater.h"
#include <Arduino.h>   // For pinMode, digitalWrite, millis, micros
#include "esp_log.h"   // For ESP32 logging (ESP_LOGI, ESP_LOGE, etc.)
#include <cmath>       // For fabs

// Initialize static const member for logging tag
const char* Heater::HEATER_CLASS_TAG = "Heater"; // Renamed to avoid macro collision

// Ensure pdMS_TO_TICKS is available.
#ifndef pdMS_TO_TICKS
#define pdMS_TO_TICKS(xTimeInMs) ((TickType_t)(((uint64_t)(xTimeInMs) * (uint64_t)configTICK_RATE_HZ) / (uint64_t)1000U))
#endif

Heater::Heater(TemperatureSensor *sensor, uint8_t heaterPin,
               const heater_error_callback_t &error_callback,
               const pid_result_callback_t &pid_callback)
    : m_sensor(sensor),
      m_heaterPin(heaterPin),
      m_taskHandle(nullptr),
      m_pid(nullptr),
      m_autotuner(nullptr),
      m_error_callback(error_callback),
      m_pid_result_callback(pid_callback),
      m_currentTemperature(0.0f),
      m_setpoint(0.0f),
      m_pidOutput(0.0f),
      m_Kp(38.25f), // Default Kp
      m_Ki(0.01f),  // Default Ki
      m_Kd(39.15f), // Default Kd
      m_isAutotuning(false),
      m_lastAutotuneStepTimeUs(0),
      m_autotuneIntervalUs(HEATER_PWM_WINDOW_MS * 1000UL), // Default, will be set by autotuner setup
      m_relayState(false),
      m_pwmWindowStartTimeMs(0),
      m_plotCount(1) {
    ESP_LOGI(HEATER_CLASS_TAG, "Constructor: Heater object created for pin %d.", m_heaterPin);
    // Allocate PID and Autotuner objects
    m_pid = new QuickPID(&m_currentTemperature, &m_pidOutput, &m_setpoint);
    m_autotuner = new PIDAutotuner();

    if (!m_pid || !m_autotuner) {
        ESP_LOGE(HEATER_CLASS_TAG, "Failed to allocate PID or Autotuner objects!");
        // Consider calling m_error_callback here or throwing an exception if appropriate
    }
}

Heater::~Heater() {
    ESP_LOGI(HEATER_CLASS_TAG, "Destructor: Cleaning up Heater for pin %d.", m_heaterPin);
    // Stop and delete the task first
    if (m_taskHandle != nullptr) {
        ESP_LOGI(HEATER_CLASS_TAG, "Deleting heater task...");
        vTaskDelete(m_taskHandle);
        m_taskHandle = nullptr;
    }

    // Delete dynamically allocated objects
    delete m_pid;
    m_pid = nullptr;
    delete m_autotuner;
    m_autotuner = nullptr;
    ESP_LOGI(HEATER_CLASS_TAG, "PID and Autotuner objects deleted.");
}

void Heater::setup() {
    ESP_LOGI(HEATER_CLASS_TAG, "Setup: Initializing heater on pin %d.", m_heaterPin);
    pinMode(m_heaterPin, OUTPUT);
    digitalWrite(m_heaterPin, LOW); // Ensure heater is off initially
    m_relayState = false;

    if (!m_pid || !m_autotuner) {
        ESP_LOGE(HEATER_CLASS_TAG, "PID or Autotuner not allocated. Cannot setup.");
        if(m_error_callback) m_error_callback();
        return;
    }

    initializePid();

    // Create the FreeRTOS task
    BaseType_t taskCreated = xTaskCreate(
        Heater::staticTaskFunction,     // Static wrapper function
        "HeaterCtrlTask",               // Task name for debugging
        HEATER_TASK_STACK_SIZE_WORDS,   // Stack size
        this,                           // Parameter to pass (this instance)
        HEATER_TASK_PRIORITY,           // Task priority
        &m_taskHandle                   // Task handle
    );

    if (taskCreated != pdPASS) {
        ESP_LOGE(HEATER_CLASS_TAG, "Failed to create heater control task! Error code: %d", (int)taskCreated);
        m_taskHandle = nullptr; // Ensure handle is null if creation failed
        if(m_error_callback) m_error_callback();
    } else {
        ESP_LOGI(HEATER_CLASS_TAG, "Heater control task created successfully.");
    }
}

void Heater::staticTaskFunction(void *param) {
    Heater *heaterInstance = static_cast<Heater *>(param);
    if (heaterInstance) {
        heaterInstance->taskFunction();
    } else {
        ESP_LOGE(HEATER_CLASS_TAG, "staticTaskFunction received null parameter!");
    }
    // Should not reach here if taskFunction has an infinite loop.
    // If it can exit, vTaskDelete(NULL) should be called by taskFunction before returning.
}

void Heater::taskFunction() {
    ESP_LOGI(HEATER_CLASS_TAG, "Heater task started on core %d.", xPortGetCoreID());
    m_pwmWindowStartTimeMs = millis(); // Initialize PWM window start time

    while (true) {
        // Read temperature (critical to do this every loop)
        if (m_sensor) {
            m_currentTemperature = m_sensor->read();
            // TODO: Add check for sensor error (e.g., if m_sensor->hasError())
        } else {
            ESP_LOGE(HEATER_CLASS_TAG, "Sensor is null in task loop!");
            m_pidOutput = 0; // Safety
            // Potentially call m_error_callback or break loop
        }

        if (m_isAutotuning) {
            runAutotuneControl();
        } else {
            runPidControl();
        }
        
        // Apply the calculated output via PWM
        applySoftwarePwm(m_pidOutput);

        vTaskDelay(pdMS_TO_TICKS(HEATER_TASK_LOOP_DELAY_MS));
    }
}

void Heater::initializePid() {
    ESP_LOGD(HEATER_CLASS_TAG, "Initializing PID controller.");
    m_pid->SetOutputLimits(0, HEATER_PWM_OUTPUT_SPAN);
    // Sample time: How often PID::Compute() expects new data.
    // This should align with how frequently the temperature is read and Compute() is called.
    // If task loop is 10ms, a sample time of 100ms to 1000ms is common.
    m_pid->SetSampleTimeUs(HEATER_PWM_WINDOW_MS * 1000UL); // e.g., 1 second sample time
    m_pid->SetMode(QuickPID::Control::automatic);
    m_pid->SetProportionalMode(QuickPID::pMode::pOnError);
    m_pid->SetDerivativeMode(QuickPID::dMode::dOnMeas);
    m_pid->SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
    m_pid->SetTunings(m_Kp, m_Ki, m_Kd);
    m_pid->Initialize(); // Crucial: Initialize PID after settings
    ESP_LOGI(HEATER_CLASS_TAG, "PID initialized. Kp:%.2f, Ki:%.2f, Kd:%.2f, SampleTime:%lums", m_Kp, m_Ki, m_Kd, HEATER_PWM_WINDOW_MS);
}

void Heater::initializeAutotuner(int targetTemp, int cycles) {
    ESP_LOGI(HEATER_CLASS_TAG, "Initializing Autotuner. Target: %dC, Cycles: %d", targetTemp, cycles);
    m_pid->Initialize(); // Reset PID state
    m_pid->SetMode(QuickPID::Control::manual); // Autotuner controls output directly

    m_autotuneIntervalUs = HEATER_PWM_WINDOW_MS * 1000UL; // Default interval, can be tuned

    m_autotuner->setTargetInputValue(targetTemp);
    m_autotuner->setLoopInterval(m_autotuneIntervalUs); // How often tunePID should be called
    m_autotuner->setOutputRange(0, HEATER_PWM_OUTPUT_SPAN);
    m_autotuner->setZNMode(PIDAutotuner::ZNModeLessOvershoot);
    m_autotuner->setTuningCycles(cycles);
    // m_autotuner->startTuningLoop(micros()); // Some libraries might need an explicit start
    ESP_LOGI(HEATER_CLASS_TAG, "Autotuner initialized. Loop Interval: %lu us", m_autotuneIntervalUs);
}

void Heater::runPidControl() {
    if (m_setpoint <= 0.0f) { // Using 0.0f or less as "off"
        // Cast QuickPID::Control enum to uint8_t for comparison with GetMode()
        if (m_pid->GetMode() != static_cast<uint8_t>(QuickPID::Control::manual) || m_pidOutput != 0.0f) {
            m_pid->SetMode(QuickPID::Control::manual);
            m_pidOutput = 0.0f; // Force output off
            ESP_LOGD(HEATER_CLASS_TAG, "PID Control: Setpoint OFF. Mode: Manual, Output: 0. Temp: %.2f", m_currentTemperature);
        }
    } else {
        // Cast QuickPID::Control enum to uint8_t for comparison with GetMode()
        if (m_pid->GetMode() != static_cast<uint8_t>(QuickPID::Control::automatic)) {
            m_pid->SetMode(QuickPID::Control::automatic);
            // m_pid->Initialize(); // Consider if needed when switching to auto without setpoint change
            ESP_LOGD(HEATER_CLASS_TAG, "PID Control: Setpoint ON. Mode: Automatic. Temp: %.2f, Setpoint: %.2f", m_currentTemperature, m_setpoint);
        }
        // m_pidOutput is updated by reference when m_pid->Compute() is called
        if (m_pid->Compute()) {
            ESP_LOGV(HEATER_CLASS_TAG, "PID Computed. Output: %.2f", m_pidOutput); // Changed to Verbose
            // logPlotData(m_pidOutput); // Plot every Nth call if needed
        }
    }
}

void Heater::runAutotuneControl() {
    if (!m_isAutotuning) return; // Should be guarded by caller, but good check

    unsigned long nowUs = micros();
    if (m_lastAutotuneStepTimeUs == 0 || (nowUs - m_lastAutotuneStepTimeUs) >= m_autotuneIntervalUs) {
        if (m_lastAutotuneStepTimeUs == 0) { // First step
            ESP_LOGI(HEATER_CLASS_TAG, "Autotune: Starting first step. Temp: %.2f", m_currentTemperature);
        }
        m_lastAutotuneStepTimeUs = nowUs;

        // m_pidOutput is updated by reference by tunePID
        m_pidOutput = m_autotuner->tunePID(m_currentTemperature, nowUs);
        ESP_LOGD(HEATER_CLASS_TAG, "Autotune Step. Temp: %.2f, Output: %.2f", m_currentTemperature, m_pidOutput);
        logPlotData(m_pidOutput); // Plot every autotune step

        if (m_autotuner->isFinished()) {
            ESP_LOGI(HEATER_CLASS_TAG, "Autotune: Process finished!");
            m_pidOutput = 0.0f; // Turn off heater output
            m_isAutotuning = false;

            float tunedKp = m_autotuner->getKp();
            float tunedKi = m_autotuner->getKi();
            float tunedKd = m_autotuner->getKd();

            ESP_LOGI(HEATER_CLASS_TAG, "Autotune Results: Kp: %.2f, Ki: %.2f, Kd: %.2f", tunedKp, tunedKi, tunedKd);

            if (m_pid_result_callback) {
                m_pid_result_callback(tunedKp, tunedKi, tunedKd);
            }
            setTunings(tunedKp, tunedKi, tunedKd); // Apply new tunings
            m_pid->SetMode(QuickPID::Control::automatic); // Switch PID back to automatic
            // Setpoint remains as it was before autotune, or user can set a new one.
        }
    }
}

void Heater::applySoftwarePwm(float pwmOutputValue) {
    unsigned long msNow = millis();

    if (msNow - m_pwmWindowStartTimeMs >= HEATER_PWM_WINDOW_MS) {
        m_pwmWindowStartTimeMs = msNow;
    }

    // Ensure pwmOutputValue is within the 0 to HEATER_PWM_OUTPUT_SPAN range
    float clampedOutput = pwmOutputValue;
    if (clampedOutput < 0.0f) clampedOutput = 0.0f;
    if (clampedOutput > HEATER_PWM_OUTPUT_SPAN) clampedOutput = HEATER_PWM_OUTPUT_SPAN;
    
    // Calculate on-time based on the output span (e.g., if span is 1000, output 500 means 50% of window)
    unsigned long onTimeMs = static_cast<unsigned long>((clampedOutput / HEATER_PWM_OUTPUT_SPAN) * HEATER_PWM_WINDOW_MS);
    unsigned long elapsedInWindowMs = msNow - m_pwmWindowStartTimeMs;

    bool shouldBeOn = (onTimeMs > 0 && elapsedInWindowMs < onTimeMs);

    if (shouldBeOn) {
        if (!m_relayState) {
            digitalWrite(m_heaterPin, HIGH);
            m_relayState = true;
            ESP_LOGV(HEATER_CLASS_TAG, "PWM: Heater ON. Output: %.2f, OnTime: %lu ms", clampedOutput, onTimeMs);
        }
    } else {
        if (m_relayState) {
            digitalWrite(m_heaterPin, LOW);
            m_relayState = false;
            ESP_LOGV(HEATER_CLASS_TAG, "PWM: Heater OFF. Output: %.2f", clampedOutput);
        }
    }
}

void Heater::logPlotData(float currentPwmOutput) {
    // Plot every Nth call to reduce log spam. Adjust '1' as needed.
    if (m_plotCount >= 1) { 
        m_plotCount = 1;
        char logBuffer[128]; // Static buffer for snprintf
        snprintf(logBuffer, sizeof(logBuffer), "Setpoint: %.2f, Temp: %.2f, PIDOut: %.2f",
                 m_setpoint, m_currentTemperature, currentPwmOutput);
        ESP_LOGI("HeaterPlot", "%s", logBuffer); // Use a distinct tag for plot data
    } else {
        m_plotCount++;
    }
}

void Heater::setSetpoint(float setpoint) {
    if (fabs(m_setpoint - setpoint) > 0.01f) { // Compare floats with a small epsilon
        ESP_LOGI(HEATER_CLASS_TAG, "New setpoint: %.2f C (was: %.2f C)", setpoint, m_setpoint);
        m_setpoint = setpoint;

        if (m_setpoint <= 0.0f) {
            // If new setpoint is off, switch PID to manual and force output to 0
            m_pid->SetMode(QuickPID::Control::manual);
            m_pidOutput = 0.0f; // Ensure output reflects this
            ESP_LOGI(HEATER_CLASS_TAG, "Setpoint OFF. PID: Manual, Output: 0.");
        } else if (!m_isAutotuning) {
            // If setpoint is on and not autotuning, ensure PID is auto and initialized
            m_pid->SetMode(QuickPID::Control::automatic);
            m_pid->Initialize(); // Re-initialize PID for new setpoint or mode change
            ESP_LOGI(HEATER_CLASS_TAG, "Setpoint ACTIVE. PID: Automatic, Initialized.");
        }
        // If autotuning, the autotuner manages the setpoint and PID mode internally.
    }
}

void Heater::setTunings(float Kp, float Ki, float Kd) {
    if (fabs(m_Kp - Kp) > 0.001f || fabs(m_Ki - Ki) > 0.001f || fabs(m_Kd - Kd) > 0.001f) {
        m_Kp = Kp;
        m_Ki = Ki;
        m_Kd = Kd;
        m_pid->SetTunings(m_Kp, m_Ki, m_Kd);
        m_pid->Initialize(); // IMPORTANT: Re-initialize PID after changing tunings
        ESP_LOGI(HEATER_CLASS_TAG, "PID tunings updated: Kp: %.2f, Ki: %.2f, Kd: %.2f", m_Kp, m_Ki, m_Kd);
    }
}

void Heater::startAutotune(int targetTemp, int cycles) {
    if (m_isAutotuning) {
        ESP_LOGW(HEATER_CLASS_TAG, "Autotune command received while already tuning. Ignoring.");
        return;
    }
    ESP_LOGI(HEATER_CLASS_TAG, "Starting autotune. Target: %dC, Cycles: %d", targetTemp, cycles);
    initializeAutotuner(targetTemp, cycles); // Setup the autotuner
    m_isAutotuning = true;
    m_lastAutotuneStepTimeUs = 0; // Reset for the non-blocking loop
    m_pidOutput = 0; // Start with heater off or let autotuner decide first output
}

void Heater::stopAutotune() {
    if (m_isAutotuning) {
        ESP_LOGI(HEATER_CLASS_TAG, "Autotune stopped manually.");
        m_isAutotuning = false;
        m_pidOutput = 0.0f; // Turn off heater
        applySoftwarePwm(m_pidOutput); // Ensure it's applied
        m_pid->SetMode(QuickPID::Control::automatic); // Revert to PID control
        m_pid->Initialize(); // Reset PID
    }
}

float Heater::getCurrentTemperature() const {
    return m_currentTemperature;
}

float Heater::getCurrentSetpoint() const {
    return m_setpoint;
}

bool Heater::isAutotuning() const {
    return m_isAutotuning;
}

