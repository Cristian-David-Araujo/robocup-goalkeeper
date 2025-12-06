/**
 * @file wifi_control.c
 * @brief WiFi control utility functions implementation
 * 
 * Provides helper functions for WiFi command parsing, validation,
 * and checksum calculation.
 * 
 * @author Cristian David Araujo A.
 * @date December 2024
 */

#include "wifi_control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "WIFI_CTRL";

// =============================================================================
// PUBLIC API IMPLEMENTATION
// =============================================================================

bool wifi_control_init(wifi_control_state_t *state)
{
    if (!state) {
        return false;
    }

    memset(state, 0, sizeof(wifi_control_state_t));
    state->connected = false;
    state->receiving_commands = false;
    state->last_cmd_time_ms = 0;
    state->packets_received = 0;
    state->packets_invalid = 0;

    ESP_LOGI(TAG, "WiFi control initialized");
    return true;
}

wifi_cmd_status_t wifi_control_parse_packet(const uint8_t *packet, 
                                             size_t length,
                                             velocity_t *cmd,
                                             wifi_control_state_t *state)
{
    if (!packet || !cmd || !state) {
        return WIFI_CMD_INVALID_FORMAT;
    }

    // Check packet size
    if (length != sizeof(wifi_cmd_packet_t)) {
        ESP_LOGW(TAG, "Invalid packet size: %d (expected %d)", 
                 length, sizeof(wifi_cmd_packet_t));
        return WIFI_CMD_INVALID_FORMAT;
    }

    // Cast to packet structure
    const wifi_cmd_packet_t *pkt = (const wifi_cmd_packet_t *)packet;

    // Verify checksum
    velocity_t temp_cmd = {
        .vx = pkt->vx,
        .vy = pkt->vy,
        .wz = pkt->wz
    };
    
    uint16_t calc_checksum = wifi_control_calc_checksum(&temp_cmd);
    if (calc_checksum != pkt->checksum) {
        ESP_LOGW(TAG, "Checksum mismatch: got 0x%04X, expected 0x%04X",
                 pkt->checksum, calc_checksum);
        state->packets_invalid++;
        return WIFI_CMD_CHECKSUM_ERROR;
    }

    // Extract velocities
    cmd->vx = pkt->vx;
    cmd->vy = pkt->vy;
    cmd->wz = pkt->wz;

    // Check velocity limits
    if (fabsf(cmd->vx) > WIFI_MAX_LINEAR_VEL ||
        fabsf(cmd->vy) > WIFI_MAX_LINEAR_VEL ||
        fabsf(cmd->wz) > WIFI_MAX_ANGULAR_VEL) {
        ESP_LOGW(TAG, "Velocity out of range, clamping");
        wifi_control_clamp_velocity(cmd);
    }

    state->packets_received++;
    return WIFI_CMD_OK;
}

wifi_cmd_status_t wifi_control_parse_json(const char *json_str, velocity_t *cmd)
{
    if (!json_str || !cmd) {
        return WIFI_CMD_INVALID_FORMAT;
    }

    // Simple JSON parser
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    
    // Try parsing with various format variations
    int parsed = sscanf(json_str, "{\"vx\":%f,\"vy\":%f,\"wz\":%f", &vx, &vy, &wz);
    if (parsed != 3) {
        parsed = sscanf(json_str, "{ \"vx\" : %f , \"vy\" : %f , \"wz\" : %f", &vx, &vy, &wz);
    }
    if (parsed != 3) {
        parsed = sscanf(json_str, "{\"vx\": %f, \"vy\": %f, \"wz\": %f", &vx, &vy, &wz);
    }

    if (parsed != 3) {
        ESP_LOGW(TAG, "Failed to parse JSON: %s", json_str);
        return WIFI_CMD_INVALID_FORMAT;
    }

    cmd->vx = vx;
    cmd->vy = vy;
    cmd->wz = wz;

    // Validate ranges
    if (fabsf(vx) > WIFI_MAX_LINEAR_VEL * 2.0f ||
        fabsf(vy) > WIFI_MAX_LINEAR_VEL * 2.0f ||
        fabsf(wz) > WIFI_MAX_ANGULAR_VEL * 2.0f) {
        ESP_LOGW(TAG, "Velocity significantly out of range");
        return WIFI_CMD_OUT_OF_RANGE;
    }

    return WIFI_CMD_OK;
}

bool wifi_control_check_timeout(wifi_control_state_t *state, velocity_t *cmd)
{
    if (!state || !cmd) {
        return false;
    }

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = now_ms - state->last_cmd_time_ms;

    if (elapsed > WIFI_CMD_TIMEOUT_MS) {
        // Timeout detected - zero the command
        cmd->vx = 0.0f;
        cmd->vy = 0.0f;
        cmd->wz = 0.0f;
        state->receiving_commands = false;
        return false;
    }

    return true;
}

void wifi_control_clamp_velocity(velocity_t *cmd)
{
    if (!cmd) {
        return;
    }

    // Clamp linear velocities
    if (cmd->vx > WIFI_MAX_LINEAR_VEL) {
        cmd->vx = WIFI_MAX_LINEAR_VEL;
    } else if (cmd->vx < -WIFI_MAX_LINEAR_VEL) {
        cmd->vx = -WIFI_MAX_LINEAR_VEL;
    }

    if (cmd->vy > WIFI_MAX_LINEAR_VEL) {
        cmd->vy = WIFI_MAX_LINEAR_VEL;
    } else if (cmd->vy < -WIFI_MAX_LINEAR_VEL) {
        cmd->vy = -WIFI_MAX_LINEAR_VEL;
    }

    // Clamp angular velocity
    if (cmd->wz > WIFI_MAX_ANGULAR_VEL) {
        cmd->wz = WIFI_MAX_ANGULAR_VEL;
    } else if (cmd->wz < -WIFI_MAX_ANGULAR_VEL) {
        cmd->wz = -WIFI_MAX_ANGULAR_VEL;
    }
}

uint16_t wifi_control_calc_checksum(const velocity_t *cmd)
{
    if (!cmd) {
        return 0;
    }

    // Simple checksum: XOR of all bytes
    const uint8_t *data = (const uint8_t *)cmd;
    uint16_t checksum = 0;

    for (size_t i = 0; i < sizeof(float) * 3; i++) {
        checksum ^= data[i];
        checksum = (checksum << 1) | (checksum >> 15); // Rotate left
    }

    return checksum;
}

void wifi_control_get_stats(const wifi_control_state_t *state,
                            bool *connected,
                            uint32_t *packets_rx,
                            uint32_t *packets_invalid)
{
    if (!state) {
        return;
    }

    if (connected) {
        *connected = state->connected;
    }
    if (packets_rx) {
        *packets_rx = state->packets_received;
    }
    if (packets_invalid) {
        *packets_invalid = state->packets_invalid;
    }
}
