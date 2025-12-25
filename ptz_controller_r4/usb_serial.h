/**
 * @file usb_serial.h
 * @brief USB serial command parser for Raspberry Pi communication
 * 
 * Command protocol:
 * - VEL <pan> <tilt> <zoom> - Set velocities (normalized -1 to 1)
 * - GOTO <n> - Move to preset n
 * - SAVE <n> - Save current position as preset n
 * - HOME - Start homing sequence
 * - STOP - Emergency stop
 * - PRECISION <0|1> - Enable/disable precision mode
 * - POS - Get current positions
 * - STATUS - Get system status
 */

#ifndef USB_SERIAL_H
#define USB_SERIAL_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize USB serial communication
 */
void usb_serial_init(void);

/**
 * @brief Process incoming serial commands (call from main loop)
 * @return true if a command was processed
 */
bool usb_serial_process(void);

/**
 * @brief Send status message
 */
void usb_serial_send_status(const char* message);

/**
 * @brief Send error message
 */
void usb_serial_send_error(const char* message);

#endif // USB_SERIAL_H

