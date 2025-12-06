/**
 * @file task_wifi_comm.c
 * @brief WiFi communication task for remote robot control
 * 
 * This task manages WiFi connection and receives velocity commands from
 * an external controller via UDP socket. Commands are validated, clamped
 * to safety limits, and forwarded to the velocity control system.
 * 
 * Features:
 * - UDP socket for low-latency command reception
 * - JSON command parsing
 * - Command validation and velocity clamping
 * - Automatic timeout and safety stop
 * - Connection status monitoring
 * 
 * Thread-safety: Sends commands via g_desired_velocity_queue
 * 
 * @author Cristian David Araujo A.
 * @date December 2024
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "wifi_control.h"
#include "types_utils.h"
#include "config_utils.h"

#include <string.h>
#include <math.h>

// =============================================================================
// LOGGING
// =============================================================================

static const char *TAG = "WIFI_COMM";

// =============================================================================
// EXTERNAL SHARED DATA
// =============================================================================

extern QueueHandle_t g_desired_velocity_queue;

// =============================================================================
// INTERNAL STATE
// =============================================================================

static wifi_control_state_t g_wifi_state = {0};
static int g_udp_socket = -1;

// =============================================================================
// INTERNAL HELPER FUNCTIONS
// =============================================================================

/**
 * @brief WiFi event handler
 * 
 * Handles WiFi connection and disconnection events.
 * 
 * @param arg User argument (unused)
 * @param event_base Event base
 * @param event_id Event ID
 * @param event_data Event data
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi started, connecting...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        g_wifi_state.connected = false;
        g_wifi_state.receiving_commands = false;
        esp_wifi_connect();
        ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "WiFi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        g_wifi_state.connected = true;
    }
}

/**
 * @brief Initialize WiFi in station mode
 * 
 * Configures and starts WiFi, connects to configured network.
 * 
 * @return true if successful, false otherwise
 */
static bool wifi_init_sta(void)
{
    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // WiFi configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, 
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, 
                                               &wifi_event_handler, NULL));

    // Configure WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_CONTROL_SSID,
            .password = WIFI_CONTROL_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization complete");
    return true;
}

/**
 * @brief Create UDP socket for receiving commands
 * 
 * @return Socket file descriptor, or -1 on error
 */
static int create_udp_socket(void)
{
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(WIFI_CONTROL_PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return -1;
    }

    // Set socket timeout
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "UDP socket listening on port %d", WIFI_CONTROL_PORT);
    return sock;
}

/**
 * @brief Parse JSON velocity command
 * 
 * Expected format: {"vx":0.5,"vy":0.3,"wz":0.1}
 * 
 * @param json_str JSON string
 * @param cmd Output velocity command
 * @return wifi_cmd_status_t Parse result
 */
static wifi_cmd_status_t parse_json_command(const char *json_str, velocity_t *cmd)
{
    // Simple JSON parser (for production, use cJSON library)
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    
    if (sscanf(json_str, "{\"vx\":%f,\"vy\":%f,\"wz\":%f", &vx, &vy, &wz) != 3) {
        // Try alternative format with spaces
        if (sscanf(json_str, "{ \"vx\" : %f , \"vy\" : %f , \"wz\" : %f", &vx, &vy, &wz) != 3) {
            return WIFI_CMD_INVALID_FORMAT;
        }
    }

    cmd->vx = vx;
    cmd->vy = vy;
    cmd->wz = wz;

    return WIFI_CMD_OK;
}

/**
 * @brief Clamp velocity to configured limits
 * 
 * @param cmd Velocity command to clamp (modified in-place)
 */
static void clamp_velocity(velocity_t *cmd)
{
    // Clamp linear velocities
    if (cmd->vx > WIFI_MAX_LINEAR_VEL) cmd->vx = WIFI_MAX_LINEAR_VEL;
    if (cmd->vx < -WIFI_MAX_LINEAR_VEL) cmd->vx = -WIFI_MAX_LINEAR_VEL;
    
    if (cmd->vy > WIFI_MAX_LINEAR_VEL) cmd->vy = WIFI_MAX_LINEAR_VEL;
    if (cmd->vy < -WIFI_MAX_LINEAR_VEL) cmd->vy = -WIFI_MAX_LINEAR_VEL;

    // Clamp angular velocity
    if (cmd->wz > WIFI_MAX_ANGULAR_VEL) cmd->wz = WIFI_MAX_ANGULAR_VEL;
    if (cmd->wz < -WIFI_MAX_ANGULAR_VEL) cmd->wz = -WIFI_MAX_ANGULAR_VEL;
}

/**
 * @brief Check for command timeout and send zero velocity if needed
 * 
 * @return true if command is still valid, false if timed out
 */
static bool check_command_timeout(void)
{
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = now_ms - g_wifi_state.last_cmd_time_ms;

    if (elapsed > WIFI_CMD_TIMEOUT_MS && g_wifi_state.receiving_commands) {
        ESP_LOGW(TAG, "Command timeout (%lu ms), sending stop", elapsed);
        
        // Send zero velocity command
        velocity_t stop_cmd = {0};
        if (g_desired_velocity_queue) {
            xQueueSend(g_desired_velocity_queue, &stop_cmd, 0);
        }
        
        g_wifi_state.receiving_commands = false;
        return false;
    }

    return true;
}

// =============================================================================
// TASK IMPLEMENTATION
// =============================================================================

/**
 * @brief WiFi communication task
 * 
 * Main task that:
 * 1. Initializes WiFi connection
 * 2. Creates UDP socket
 * 3. Receives velocity commands
 * 4. Validates and forwards commands
 * 5. Monitors timeout
 * 
 * @param pvParameters Unused task parameter
 */
void task_wifi_comm(void *pvParameters)
{
    ESP_LOGI(TAG, "WiFi communication task started");

    // Initialize WiFi
    if (!wifi_init_sta()) {
        ESP_LOGE(TAG, "WiFi initialization failed");
        vTaskDelete(NULL);
        return;
    }

    // Wait for WiFi connection
    while (!g_wifi_state.connected) {
        ESP_LOGI(TAG, "Waiting for WiFi connection...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Create UDP socket
    g_udp_socket = create_udp_socket();
    if (g_udp_socket < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        vTaskDelete(NULL);
        return;
    }

    // Main receive loop
    char rx_buffer[WIFI_CONTROL_BUFFER_SIZE];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    ESP_LOGI(TAG, "Ready to receive commands");

    while (1) {
        // Receive data
        int len = recvfrom(g_udp_socket, rx_buffer, sizeof(rx_buffer) - 1, 0, 
                          (struct sockaddr *)&source_addr, &socklen);

        if (len < 0) {
            // Timeout or error
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Normal timeout, check command timeout
                check_command_timeout();
                continue;
            }
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            continue;
        }

        // Null-terminate received data
        rx_buffer[len] = 0;

        // Parse command
        velocity_t cmd = {0};
        wifi_cmd_status_t status = parse_json_command(rx_buffer, &cmd);

        if (status != WIFI_CMD_OK) {
            ESP_LOGW(TAG, "Invalid command format");
            g_wifi_state.packets_invalid++;
            continue;
        }

        // Clamp velocity to safety limits
        clamp_velocity(&cmd);

        // Update state
        g_wifi_state.packets_received++;
        g_wifi_state.last_cmd_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        g_wifi_state.last_cmd = cmd;
        g_wifi_state.receiving_commands = true;

        // Forward command to velocity control task
        if (g_desired_velocity_queue) {
            if (xQueueSend(g_desired_velocity_queue, &cmd, pdMS_TO_TICKS(5)) != pdTRUE) {
                ESP_LOGW(TAG, "Failed to send command to queue (full)");
            }
        }

        // Log periodically (every 50 packets)
        if (g_wifi_state.packets_received % 50 == 0) {
            ESP_LOGI(TAG, "Packets RX: %lu, Invalid: %lu, Last cmd: vx=%.2f vy=%.2f wz=%.2f",
                     g_wifi_state.packets_received, g_wifi_state.packets_invalid,
                     cmd.vx, cmd.vy, cmd.wz);
        }
    }

    // Cleanup (never reached in normal operation)
    close(g_udp_socket);
    vTaskDelete(NULL);
}
