/**
 * @file gpio_utils.h
 * @brief GPIO pin assignments for the RoboCup Goalkeeper firmware
 *
 * This header defines symbolic names for GPIO pins assigned to hardware peripherals:
 * - AS5600 magnetic encoders (3 units) - Analog input and I2C interfaces
 * - SKYWALKER brushless motors (3 units) - PWM control signals
 * - BNO055 IMU sensor - I2C interface
 *
 * Pin assignments follow the ESP32-S3 hardware layout. Update these definitions
 * if the hardware configuration changes.
 * 
 * All constants follow SCREAMING_SNAKE_CASE naming convention.
 *
 * @note Verify pin assignments match your PCB/hardware before flashing
 */

#ifndef GPIO_UTILS_H
#define GPIO_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// ENCODER AS5600 GPIO PIN ASSIGNMENTS
// =============================================================================

/** @defgroup encoder_as5600_gpio AS5600 Magnetic Encoder Pins
 *  @brief Analog input and I2C pins for three AS5600 encoders
 *  @{
 */
#define GPIO_ENCODER_0_IN_ANALOG 4         ///< Analog input pin for Encoder 0
#define GPIO_ENCODER_0_I2C_SDA 4           ///< I2C SDA pin for Encoder 0
#define GPIO_ENCODER_0_I2C_SCL 5           ///< I2C SCL pin for Encoder 0
#define GPIO_ENCODER_0_I2C_MASTER_NUM 1    ///< I2C master port number for Encoder 0

#define GPIO_ENCODER_1_IN_ANALOG 5         ///< Analog input pin for Encoder 1
#define GPIO_ENCODER_2_IN_ANALOG 6         ///< Analog input pin for Encoder 2
/** @} */

// =============================================================================
// MOTOR SKYWALKER GPIO PIN ASSIGNMENTS
// =============================================================================

/** @defgroup motor_skywalker_gpio SKYWALKER Brushless Motor Pins
 *  @brief PWM output pins for speed and direction control of three motors
 *  @{
 */
#define GPIO_MOTOR_0_SIGNAL_OUT_PWM 7      ///< PWM speed signal for Motor 0
#define GPIO_MOTOR_0_REVERSE_OUT_PWM 8     ///< PWM reverse signal for Motor 0

#define GPIO_MOTOR_1_SIGNAL_OUT_PWM 15     ///< PWM speed signal for Motor 1
#define GPIO_MOTOR_1_REVERSE_OUT_PWM 3     ///< PWM reverse signal for Motor 1

#define GPIO_MOTOR_2_SIGNAL_OUT_PWM 16     ///< PWM speed signal for Motor 2
#define GPIO_MOTOR_2_REVERSE_OUT_PWM 46    ///< PWM reverse signal for Motor 2
/** @} */

// =============================================================================
// IMU BNO055 GPIO PIN ASSIGNMENTS
// =============================================================================

/** @defgroup imu_bno055_gpio BNO055 IMU Sensor Pins
 *  @brief I2C pins for IMU communication
 *  @{
 */
#define GPIO_IMU_I2C_SDA 17                ///< I2C SDA pin for BNO055 IMU
#define GPIO_IMU_I2C_SCL 18                ///< I2C SCL pin for BNO055 IMU
/** @} */

#ifdef __cplusplus
}
#endif

#endif // GPIO_UTILS_H