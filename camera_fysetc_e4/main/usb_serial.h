/**
 * @file usb_serial.h
 * @brief USB serial command parser for Raspberry Pi communication
 * 
 * Parses commands received over USB serial:
 * - VEL <pan> <tilt> <zoom> - Set velocities
 * - GOTO <n> - Move to preset n
 * - SAVE <n> - Save current position as preset n
 * - HOME - Start homing sequence
 * - POS - Query current positions
 * - STATUS - Query system status
 */

#ifndef USB_SERIAL_H
#define USB_SERIAL_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Command types
 */
typedef enum {
    CMD_NONE,
    CMD_VEL,        // VEL <pan> <tilt> <zoom>
    CMD_JOYSTICK,   // j,yaw,pitch,zoom (values -32768 to 32768)
    CMD_GOTO,       // GOTO <n>
    CMD_SAVE,       // SAVE <n>
    CMD_HOME,       // HOME
    CMD_POS,        // POS
    CMD_STATUS,     // STATUS
    CMD_STOP,       // STOP
    CMD_PRECISION,  // PRECISION <0|1>
    CMD_LIMITS,     // LIMITS <axis> <min> <max>
    CMD_UNKNOWN
} cmd_type_t;

/**
 * @brief Parsed command structure
 */
typedef struct {
    cmd_type_t type;
    float velocities[3];  // For VEL command
    uint8_t preset_index; // For GOTO/SAVE commands
    bool precision_enable; // For PRECISION command
    uint8_t limits_axis;   // For LIMITS command
    float limits_min;
    float limits_max;
} parsed_cmd_t;

/**
 * @brief Initialize USB serial
 */
void usb_serial_init(void);

/**
 * @brief Process incoming serial data
 * @param cmd Output parsed command structure
 * @return true if a complete command was parsed
 */
bool usb_serial_parse_command(parsed_cmd_t* cmd);

/**
 * @brief Send response string
 */
void usb_serial_send(const char* format, ...);

/**
 * @brief Send position response
 */
void usb_serial_send_position(float pan, float tilt, float zoom);

/**
 * @brief Send status response
 */
void usb_serial_send_status(const char* status);

/**
 * @brief Serial message entry for logging
 */
typedef struct {
    int64_t timestamp_ms;  // Timestamp in milliseconds
    char message[128];     // Message text
    bool is_command;       // true if incoming command, false if outgoing response
} serial_message_t;

/**
 * @brief Get recent serial messages
 * @param messages Output array of messages
 * @param max_messages Maximum number of messages to return
 * @return Number of messages returned
 */
int usb_serial_get_messages(serial_message_t* messages, int max_messages);

#endif // USB_SERIAL_H

