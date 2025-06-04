#ifndef HEATER_H
#define HEATER_H

#include "peripherals/TemperatureSensor.h" // Assuming this is your base class for sensors
#include <QuickPID.h>
#include <PIDAutotuner.h>
#include <functional>          // For std::function
#include "freertos/FreeRTOS.h" // For FreeRTOS types and functions
#include "freertos/task.h"     // For xTaskHandle, vTaskDelete, etc.

// --- Constants ---
constexpr float HEATER_PWM_OUTPUT_SPAN = 1000.0f; // Defines the PWM window/range (e.g., 0-1000 for 0-100.0%)
constexpr uint32_t HEATER_PWM_WINDOW_MS = 1000;   // Software PWM window period in milliseconds (e.g., 1 second)
constexpr uint32_t HEATER_TASK_STACK_SIZE_WORDS = configMINIMAL_STACK_SIZE * 8; // Stack size for the heater task
constexpr UBaseType_t HEATER_TASK_PRIORITY = 5;   // Priority for the heater task (adjust as needed)
constexpr uint32_t HEATER_TASK_LOOP_DELAY_MS = 10; // Delay in the heater task loop

// --- Type Aliases for Callbacks ---
using heater_error_callback_t = std::function<void()>; // Callback for critical errors
using pid_result_callback_t = std::function<void(float Kp, float Ki, float Kd)>; // Callback for autotune results

class Heater {
public:
    /**
     * @brief Constructor for the Heater class.
     * @param sensor Pointer to the temperature sensor object.
     * @param heaterPin GPIO pin connected to the heater relay/driver.
     * @param error_callback Callback function for critical errors.
     * @param pid_callback Callback function to report PID autotune results.
     */
    Heater(TemperatureSensor *sensor, uint8_t heaterPin,
           const heater_error_callback_t &error_callback,
           const pid_result_callback_t &pid_callback);

    /**
     * @brief Destructor. Cleans up allocated resources and stops the task.
     */
    ~Heater();

    /**
     * @brief Initializes the heater, sets up PID, and starts the control task.
     * Must be called once in the main setup.
     */
    void setup();

    /**
     * @brief Sets the target temperature for the PID controller.
     * @param setpoint The desired temperature in Celsius.
     */
    void setSetpoint(float setpoint);

    /**
     * @brief Sets new PID tuning parameters.
     * @param Kp Proportional gain.
     * @param Ki Integral gain.
     * @param Kd Derivative gain.
     */
    void setTunings(float Kp, float Ki, float Kd);

    /**
     * @brief Starts the PID autotuning process.
     * @param targetTemp The temperature at which to perform autotuning.
     * @param cycles The number of tuning cycles to perform.
     */
    void startAutotune(int targetTemp, int cycles);

    /**
     * @brief Stops the PID autotuning process if it's running.
     */
    void stopAutotune();

    /**
     * @brief Gets the current measured temperature.
     * @return Temperature in Celsius.
     */
    float getCurrentTemperature() const;

    /**
     * @brief Gets the current target setpoint.
     * @return Setpoint in Celsius.
     */
    float getCurrentSetpoint() const;

    /**
     * @brief Checks if the autotuning process is currently active.
     * @return True if autotuning, false otherwise.
     */
    bool isAutotuning() const;

private:
    // --- Private Methods ---
    void taskFunction(); // Main function executed by the FreeRTOS task
    void initializePid();
    void initializeAutotuner(int targetTemp, int cycles);
    void runPidControl();
    void runAutotuneControl();
    void applySoftwarePwm(float pwmOutputValue);
    void logPlotData(float currentPwmOutput); // Helper for logging/plotting

    // --- Static Task Runner ---
    // FreeRTOS tasks must be static or global functions. This wrapper calls the instance method.
    static void staticTaskFunction(void *param);

    // --- Member Variables ---
    TemperatureSensor *m_sensor;      // Pointer to the temperature sensor
    uint8_t m_heaterPin;              // GPIO pin for the heater control
    xTaskHandle m_taskHandle;         // Handle for the FreeRTOS task

    QuickPID *m_pid;                  // Pointer to the QuickPID object
    PIDAutotuner *m_autotuner;        // Pointer to the PIDAutotuner object

    heater_error_callback_t m_error_callback; // Callback for error conditions
    pid_result_callback_t m_pid_result_callback; // Callback for PID tuning results

    // PID and Control State
    float m_currentTemperature;       // Last read temperature
    float m_setpoint;                 // Target temperature
    float m_pidOutput;                // Output from PID controller (0 to HEATER_PWM_OUTPUT_SPAN)
    float m_Kp;                       // Proportional gain
    float m_Ki;                       // Integral gain
    float m_Kd;                       // Derivative gain

    // Autotuning State
    bool m_isAutotuning;              // True if autotuning is active
    unsigned long m_lastAutotuneStepTimeUs; // Timestamp of the last autotune step (in microseconds)
    unsigned long m_autotuneIntervalUs;   // Interval between autotune steps (in microseconds)

    // Software PWM State
    bool m_relayState;                // Current state of the heater relay (true if ON)
    unsigned long m_pwmWindowStartTimeMs; // Start time of the current PWM window (in milliseconds)

    // Plotting/Logging Helper
    uint8_t m_plotCount;              // Counter for periodic plotting

    // Logging Tag
    static const char* HEATER_CLASS_TAG; // Renamed to avoid macro collision
};

#endif // HEATER_H
