# BNO055 and AS5600 Refactoring Guide

This document provides a complete mapping of old names to new names after the refactoring to follow snake_case conventions and FreeRTOS/ESP-IDF coding standards.

## BNO055 Module Refactoring

### Header File: `include/bno055.h`

#### Type Definitions
| Old Name | New Name |
|----------|----------|
| `BNO055_t` | `bno055_t` |
| `BNO055_OperationMode` | `bno055_operation_mode_t` |
| `BNO055_PowerMode` | `bno055_power_mode_t` |
| `BNO055_UnitSettings_t` | `bno055_unit_settings_t` |
| `BNO055_CalibProfile_t` | `bno055_calib_profile_t` |

#### Enum Values (Operation Modes)
| Old Name | New Name |
|----------|----------|
| `CONFIGMODE` | `BNO055_MODE_CONFIG` |
| `ACCONLY` | `BNO055_MODE_ACCONLY` |
| `MAGONLY` | `BNO055_MODE_MAGONLY` |
| `GYROONLY` | `BNO055_MODE_GYROONLY` |
| `ACCMAG` | `BNO055_MODE_ACCMAG` |
| `ACCGYRO` | `BNO055_MODE_ACCGYRO` |
| `MAGGYRO` | `BNO055_MODE_MAGGYRO` |
| `AMG` | `BNO055_MODE_AMG` |
| `IMU` | `BNO055_MODE_IMU` |
| `COMPASS` | `BNO055_MODE_COMPASS` |
| `M4G` | `BNO055_MODE_M4G` |
| `NDOF_FMC_OFF` | `BNO055_MODE_NDOF_FMC_OFF` |
| `NDOF` | `BNO055_MODE_NDOF` |
| `INIT` | `BNO055_MODE_INIT` |

#### Enum Values (Power Modes)
| Old Name | New Name |
|----------|----------|
| `NORMAL` | `BNO055_POWER_NORMAL` |
| `LOWPOWER` | `BNO055_POWER_LOWPOWER` |
| `SUSPEND` | `BNO055_POWER_SUSPEND` |

#### Functions
| Old Name | New Name |
|----------|----------|
| `BNO055_Init()` | `bno055_init()` |
| `BNO055_Reset()` | `bno055_reset()` |
| `BNO055_GetCalibrationStatus()` | `bno055_get_calibration_status()` |
| `BNO055_GetInfo()` | `bno055_get_info()` |
| `BNO055_SetOperationMode()` | `bno055_set_operation_mode()` |
| `BNO055_GetEulerAngles()` | `bno055_get_euler_angles()` |
| `BNO055_GetAcceleration()` | `bno055_get_acceleration()` |
| `BNO055_GetGyro()` | `bno055_get_gyro()` |
| `BNO055_GetMagnetometer()` | `bno055_get_magnetometer()` |
| `BNO055_ReadAll()` | `bno055_read_all()` |
| `BNO055_ReadAll_Lineal()` | `bno055_read_all_linear()` |
| `BNO055_SetUnit()` | `bno055_set_unit()` |
| `BNO055_SetPowerMode()` | `bno055_set_power_mode()` |
| `BNO055_GetCalibrationProfile()` | `bno055_get_calibration_profile()` |

#### Internal Functions (now static in .c file)
| Old Name | New Name |
|----------|----------|
| `BN055_Write()` | `bno055_write()` (static) |
| `BNO055_Read()` | `bno055_read()` (static) |
| `BNO055_ConvertData_Accel()` | `bno055_convert_accel_data()` (static) |
| `BNO055_ConvertData_Gyro()` | `bno055_convert_gyro_data()` (static) |
| `BNO055_ConvertData_Euler()` | `bno055_convert_euler_data()` (static) |
| `BNO055_ConvertData_Mag()` | `bno055_convert_mag_data()` (static) |

#### Constants
| Old Name | New Name |
|----------|----------|
| `I2C_MASTER_FREQ_HZ` | `BNO055_I2C_FREQ_HZ` |

---

## AS5600 Module Refactoring

### Header File: `include/as5600.h`

#### Type Definitions
| Old Name | New Name |
|----------|----------|
| `AS5600_t` | `as5600_t` |
| `AS5600_config_t` | `as5600_config_t` (unchanged) |
| `AS5600_reg_t` | `as5600_reg_t` (unchanged) |

#### Functions - Core
| Old Name | New Name |
|----------|----------|
| `AS5600_Init()` | `as5600_init()` |
| `AS5600_Deinit()` | `as5600_deinit()` |
| `AS5600_ADC_GetAngle()` | `as5600_adc_get_angle()` |
| `AS5600_BurnAngleCommand()` | `as5600_burn_angle_command()` |
| `AS5600_BurnSettingCommand()` | `as5600_burn_setting_command()` |
| `AS5600_RegStrToAddr()` | `as5600_reg_str_to_addr()` |

#### Functions - GPIO/ADC
| Old Name | New Name |
|----------|----------|
| `AS5600_InitADC()` | `as5600_init_adc()` |
| `AS5600_InitADC_2()` | `as5600_init_adc_shared()` |
| `AS5600_DeinitADC()` | `as5600_deinit_adc()` |
| `AS5600_InitGPIO()` | `as5600_init_gpio()` |
| `AS5600_DeinitGPIO()` | `as5600_deinit_gpio()` |
| `AS5600_SetGPIO()` | `as5600_set_gpio()` |

#### Functions - I2C
| Old Name | New Name |
|----------|----------|
| `AS5600_ReadReg()` | `as5600_read_reg()` |
| `AS5600_WriteReg()` | `as5600_write_reg()` |
| `AS5600_IsValidReadReg()` | `as5600_is_valid_read_reg()` |
| `AS5600_IsValidWriteReg()` | `as5600_is_valid_write_reg()` |

#### Functions - Configuration
| Old Name | New Name |
|----------|----------|
| `AS5600_SetStartPosition()` | `as5600_set_start_position()` |
| `AS5600_GetStartPosition()` | `as5600_get_start_position()` |
| `AS5600_SetStopPosition()` | `as5600_set_stop_position()` |
| `AS5600_GetStopPosition()` | `as5600_get_stop_position()` |
| `AS5600_SetMaxAngle()` | `as5600_set_max_angle()` |
| `AS5600_GetMaxAngle()` | `as5600_get_max_angle()` |
| `AS5600_SetConf()` | `as5600_set_conf()` |
| `AS5600_GetConf()` | `as5600_get_conf()` |

#### Functions - Output/Status
| Old Name | New Name |
|----------|----------|
| `AS5600_GetRawAngle()` | `as5600_get_raw_angle()` |
| `AS5600_GetAngle()` | `as5600_get_angle()` |
| `AS5600_GetStatus()` | `as5600_get_status()` |
| `AS5600_GetAgc()` | `as5600_get_agc()` |
| `AS5600_GetMagnitude()` | `as5600_get_magnitude()` |

#### Constants
| Old Name | New Name |
|----------|----------|
| `VCC_3V3_MV` | `AS5600_VCC_3V3_MV` |
| `VCC_3V3_MIN_RR_MV` | `AS5600_VCC_3V3_MIN_RR_MV` |
| `VCC_3V3_MAX_RR_MV` | `AS5600_VCC_3V3_MAX_RR_MV` |
| `I2C_MASTER_FREQ_HZ` | `AS5600_I2C_FREQ_HZ` |
| `MAP()` | `AS5600_MAP()` |
| `LIMIT()` | `AS5600_LIMIT()` |

---

## Global Variables

In `main/main.c` and other files:
| Old Name | New Name |
|----------|----------|
| `AS5600_t g_as5600[3]` | `as5600_t g_as5600[3]` |
| `BNO055_t g_bno055` | `bno055_t g_bno055` |

---

## Files That Need Updates

### Main Files
1. `main/main.c` - Global variable declarations
2. `main/init.c` - Initialization calls

### Task Files
3. `tasks/task_read_sensors.c` - Sensor reading calls

---

## Notes

- All functions now use ESP_LOG macros (ESP_LOGI, ESP_LOGE, ESP_LOGW) instead of printf()
- Error handling has been improved with better NULL pointer checks
- Static functions are now properly declared as static in .c files
- Documentation follows Doxygen style with consistent parameter annotations
- All code follows FreeRTOS/ESP-IDF naming conventions

---

## Next Steps

1. Update the definition files (bno055_defs.h, as5600_defs.h) if needed
2. Update all references in main.c, init.c, and task files
3. Rebuild and test the firmware
