#ifndef MAX31855THERMOCOUPLE_STUB_H
#define MAX31855THERMOCOUPLE_STUB_H

#include "peripherals/TemperatureSensor.h" // Your base class
#include <functional>                     // For std::function
// No FreeRTOS includes needed if task is removed
// No <MAX31855.h> needed if library is not used

// --- Type Aliases for Callbacks (kept for interface compatibility) ---
using temp_sensor_update_callback_t = std::function<void(float temperature)>;
using temp_sensor_error_callback_t = std::function<void()>;

class Max31855Thermocouple : public TemperatureSensor {
public:
    Max31855Thermocouple(int csPin, int misoPin, int sckPin,
                         const temp_sensor_update_callback_t &update_cb,
                         const temp_sensor_error_callback_t &error_cb);
    ~Max31855Thermocouple();

    void setup(); // Removed 'override' as it was causing a build error
    float read() override;
    bool hasError() override;

private:
    // No task function needed for stub
    // No MAX31855 library instance needed

    // Callbacks are kept for API compatibility but might not be used actively by the stub
    temp_sensor_update_callback_t m_update_callback;
    temp_sensor_error_callback_t m_error_callback;

    // Logging
    static const char* TAG_V2_STUB;
};

#endif // MAX31855THERMOCOUPLE_STUB_H
