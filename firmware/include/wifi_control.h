/**
 * @file wifi_control.h
 * @brief WiFi remote control interface for robot velocity commands
 * 
 * This module provides structures and functions for receiving velocity
 * commands via WiFi network. Supports JSON and binary command formats
 * with built-in safety features including timeout and velocity limiting.
 * 
 * Features:
 * - UDP-based low-latency command reception
 * - JSON and binary protocol support
 * - Command validation and velocity clamping
 * - Automatic timeout and safety stop
 * - Connection monitoring
 * 
 * @author Cristian David Araujo A.
 * @date December 2024
 */

#ifndef WIFI_CONTROL_H
#define WIFI_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "types_utils.h"
#include "config_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// TYPE DEFINITIONS
// =============================================================================

/**
 * @brief WiFi command packet format (binary)
 * 
 * Compact binary format for velocity commands sent over network.
 * Total size: 18 bytes (optimized for UDP transmission)
 */
typedef struct __attribute__((packed)) {
    float vx;               ///< Linear velocity X (m/s)
    float vy;               ///< Linear velocity Y (m/s)
    float wz;               ///< Angular velocity Z (rad/s)
    uint32_t timestamp;     ///< Command timestamp (ms since boot)
    uint16_t checksum;      ///< Simple checksum for error detection
} wifi_cmd_packet_t;

/**
 * @brief WiFi control state
 * 
 * Tracks the current state of WiFi control system including
 * connection status, last command time, and safety flags.
 */
typedef struct {
    bool connected;             ///< WiFi connection status
    bool receiving_commands;    ///< Currently receiving valid commands
    uint32_t last_cmd_time_ms;  ///< Timestamp of last valid command
    uint32_t packets_received;  ///< Total packets received
    uint32_t packets_invalid;   ///< Invalid/malformed packets
    velocity_t last_cmd;        ///< Last valid velocity command
} wifi_control_state_t;

/**
 * @brief WiFi command validation result
 */
typedef enum {
    WIFI_CMD_OK = 0,            ///< Command is valid
    WIFI_CMD_INVALID_FORMAT,    ///< Malformed packet
    WIFI_CMD_OUT_OF_RANGE,      ///< Velocity exceeds limits
    WIFI_CMD_CHECKSUM_ERROR,    ///< Checksum mismatch
    WIFI_CMD_TIMEOUT            ///< Command too old
} wifi_cmd_status_t;

// =============================================================================
// PUBLIC API FUNCTIONS
// =============================================================================

/**
 * @brief Initialize WiFi control system
 * 
 * Connects to configured WiFi network and sets up UDP socket for
 * receiving velocity commands.
 * 
 * @param[out] state Pointer to state structure to initialize
 * @return true if initialization successful, false otherwise
 */
bool wifi_control_init(wifi_control_state_t *state);

/**
 * @brief Parse and validate WiFi command packet
 * 
 * Parses binary command packet, validates checksum, checks velocity
 * limits, and updates state.
 * 
 * @param[in] packet Pointer to received packet data
 * @param[in] length Packet length in bytes
 * @param[out] cmd Pointer to velocity command structure to fill
 * @param[in,out] state Pointer to control state
 * @return wifi_cmd_status_t validation result
 */
wifi_cmd_status_t wifi_control_parse_packet(const uint8_t *packet, 
                                             size_t length,
                                             velocity_t *cmd,
                                             wifi_control_state_t *state);

/**
 * @brief Parse JSON command string
 * 
 * Parses JSON-formatted velocity command:
 * {"vx":0.5,"vy":0.3,"wz":0.1,"timestamp":123456}
 * 
 * @param[in] json_str JSON string to parse
 * @param[out] cmd Pointer to velocity command structure to fill
 * @return wifi_cmd_status_t validation result
 */
wifi_cmd_status_t wifi_control_parse_json(const char *json_str, velocity_t *cmd);

/**
 * @brief Check if command has timed out
 * 
 * Compares current time against last command time to detect timeout.
 * If timeout detected, returns zero velocity command.
 * 
 * @param[in,out] state Pointer to control state
 * @param[out] cmd Pointer to velocity command (zeroed if timeout)
 * @return true if command is still valid, false if timed out
 */
bool wifi_control_check_timeout(wifi_control_state_t *state, velocity_t *cmd);

/**
 * @brief Clamp velocity command to safe limits
 * 
 * Ensures velocity command does not exceed configured maximum values.
 * Modifies command in-place.
 * 
 * @param[in,out] cmd Pointer to velocity command to clamp
 */
void wifi_control_clamp_velocity(velocity_t *cmd);

/**
 * @brief Calculate simple checksum for command packet
 * 
 * Computes 16-bit checksum over velocity values for error detection.
 * 
 * @param[in] cmd Pointer to velocity command
 * @return uint16_t calculated checksum
 */
uint16_t wifi_control_calc_checksum(const velocity_t *cmd);

/**
 * @brief Get WiFi control statistics
 * 
 * Returns current control state including packet counts and connection status.
 * 
 * @param[in] state Pointer to control state
 * @param[out] connected WiFi connection status
 * @param[out] packets_rx Total packets received
 * @param[out] packets_invalid Invalid packets count
 */
void wifi_control_get_stats(const wifi_control_state_t *state,
                            bool *connected,
                            uint32_t *packets_rx,
                            uint32_t *packets_invalid);

#ifdef __cplusplus
}
#endif

#endif // WIFI_CONTROL_H
