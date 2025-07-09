/**
 * @file gpio_utils.h
 * @brief Macro definitions for GPIO pins used in the RoboCup Goalkeeper firmware project.
 *
 * This header provides symbolic names for the GPIO pins assigned to various hardware components,
 * including encoders (AS5600), motors (SKYWALKER), and the IMU (BNO055).
 *
 * - Encoder AS5600: Defines analog input pins for three encoders.
 * - Motor SKYWALKER: Defines PWM output pins for signal and reverse control of three motors.
 * - IMU BNO055: Defines UART TX and RX pins for IMU communication.
 *
 * @note Update these definitions if the hardware pinout changes.
 */
#ifndef GPIO_UTILS_H
#define GPIO_UTILS_H

/** @defgroup EncoderAS5600_GPIO Encoder AS5600 GPIO Pins
 *  @brief Analog input pins for the three AS5600 encoders.
 *  @{
 */
#define GPIO_ENCODER_0_IN_ANALOG 4   /**< Analog input pin for Encoder 0 M6*/ 
#define GPIO_ENCODER_0_I2C_SDA 4   /**< I2C SDA pin for Encoder 0 */
#define GPIO_ENCODER_0_I2C_SCL 5   /**< I2C SCL pin for Encoder 0 */
#define GPIO_ENCODER_0_I2C_MASTER_NUM 1   /**< I2C Master number for Encoder 0 */
#define GPIO_ENCODER_1_IN_ANALOG 5   /**< Analog input pin for Encoder 1 */
#define GPIO_ENCODER_2_IN_ANALOG 6   /**< Analog input pin for Encoder 2 */
/** @} */

/** @defgroup MotorSKYWALKER_GPIO Motor SKYWALKER GPIO Pins
 *  @brief PWM output pins for signal and reverse control of three motors.
 *  @{
 */
#define GPIO_MOTOR_0_SIGNAL_OUT_PWM 7    /**< PWM signal output pin for Motor 0 */
#define GPIO_MOTOR_0_REVERSE_OUT_PWM 8   /**< PWM reverse output pin for Motor 0 */
#define GPIO_MOTOR_1_SIGNAL_OUT_PWM 15   /**< PWM signal output pin for Motor 1 */
#define GPIO_MOTOR_1_REVERSE_OUT_PWM 3   /**< PWM reverse output pin for Motor 1 */
#define GPIO_MOTOR_2_SIGNAL_OUT_PWM 16   /**< PWM signal output pin for Motor 2 */
#define GPIO_MOTOR_2_REVERSE_OUT_PWM 46  /**< PWM reverse output pin for Motor 2 */
/** @} */

/** @defgroup IMUBNO055_GPIO IMU BNO055 GPIO Pins
 *  @brief UART TX and RX pins for IMU communication.
 *  @{
 */
#define GPIO_IMU_I2C_SDA 17   /**< UART TX pin for IMU BNO055 */
#define GPIO_IMU_I2C_SCL 18   /**< UART RX pin for IMU BNO055 */
/** @} */


#endif // GPIO_UTILS_H