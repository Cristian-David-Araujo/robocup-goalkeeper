#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "bno055.h"      // IMU driver
#include "as5600.h"      // AS5600 encoder driver
#include "types.h"         // Shared Pose, Velocity, RawSensorData structs
#include "robot_config.h"  // Constants if needed

// Shared data and mutex (defined elsewhere, e.g., in freertos_tasks.c or global_data.c)
extern RawSensorData sensor_data;
extern SemaphoreHandle_t xSensorDataMutex;

// Task handle (optional, if you want to control it externally)
TaskHandle_t xTaskReadSensorsHandle = NULL;

// Sampling period (1ms = 1000Hz, adjust as needed)
#define SENSOR_TASK_PERIOD_MS 1

void vTaskReadSensors(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 1. Read IMU
        IMU_Data imu_raw;
        imu_read(&imu_raw);  // You implement this in imu.c

        // 2. Read encoders
        EncoderData encoders;
        encoder_read_all(&encoders);  // Also implemented in encoder.c

        // 3. Save data safely
        if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_data.imu = imu_raw;
            sensor_data.encoders = encoders;
            xSemaphoreGive(xSensorDataMutex);
        }

        // 4. Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}
