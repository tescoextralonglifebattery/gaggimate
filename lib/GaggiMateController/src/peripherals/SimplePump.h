#ifndef SIMPLEPUMP_H
#define SIMPLEPUMP_H

#include "Pump.h" // Assuming Pump.h defines the base class 'Pump'
#include <Arduino.h>
#include <freertos/FreeRTOS.h> // Required for xTaskHandle, vTaskDelete, etc.
#include <freertos/task.h>     // Required for xTaskHandle, vTaskDelete, etc.

class SimplePump : public Pump {
 public:
  /**
   * @brief Constructor for SimplePump.
   * @param pin The GPIO pin connected to the pump relay/driver.
   * @param pumpOn The logic level to turn the pump ON (HIGH or LOW).
   * @param windowSize The PWM window size in milliseconds (e.g., 5000.0f for 5 seconds).
   */
  SimplePump(int pin, uint8_t pumpOn, float windowSize = 5000.0f);

  /**
   * @brief Destructor for SimplePump. Cleans up the created FreeRTOS task.
   */
  ~SimplePump();

  /**
   * @brief Sets up the pump pin and starts the control task.
   * Should be called once in the main setup() function.
   */
  void setup() override;

  /**
   * @brief Main control loop for the pump's PWM logic.
   * This method is called periodically by an internal FreeRTOS task.
   * Users typically do not call this directly after setup.
   */
  void loop() override;

  /**
   * @brief Sets the desired power level for the pump.
   * @param setpoint Power level from 0.0 (off) to 100.0 (full power).
   * Values outside this range will be clamped.
   */
  void setPower(float setpoint) override;

 private:
  int _pin;              // GPIO pin for the pump
  uint8_t _pumpOn;       // Logic level to turn pump ON (HIGH or LOW)
  float _setpoint;       // Desired power level (0-100), actual internal value
  bool relayStatus;      // Current status of the relay (true if ON, false if OFF)
  unsigned long _windowSizeMs; // PWM window size in milliseconds
  unsigned long windowStartTime; // Start time of the current PWM window

  TaskHandle_t taskHandle; // Handle for the FreeRTOS task

  // Helper function for the FreeRTOS task.
  static void loopTask(void *arg);

  // For logging, if ESP-IDF logging is used
  // static const char *LOG_TAG; // Uncomment if using ESP_LOGX
};

#endif // SIMPLEPUMP_H
