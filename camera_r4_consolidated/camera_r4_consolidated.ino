/*
 * Camera Motion Rig - Consolidated Controller for Arduino Uno R4 Minima
 * 
 * This sketch consolidates pan, tilt, and zoom control from two separate
 * Arduino Uno R3 boards into a single Arduino Uno R4 Minima board.
 * 
 * Key Optimizations:
 * - All stepper control is non-blocking using micros()/millis() timers
 * - No delay() or delayMicroseconds() calls
 * - Zero zoom operation runs in background without blocking
 * - All three axes can move simultaneously
 * - Improved serial command responsiveness
 * 
 * Hardware: CNC Shield with TMC2209 stepper drivers
 * - Pitch (Tilt): X-axis (Step: 2, Dir: 5)
 * - Yaw (Pan): Y-axis (Step: 3, Dir: 6)
 * - Zoom: Z-axis (Step: 4, Dir: 7)
 */

#include "SafeStringReader.h"
#include <EEPROM.h>

// Pin definitions for CNC Shield with TMC2209
// Pitch (Tilt) - X-axis
const int StepX = 2;
const int DirX = 5;

// Yaw (Pan) - Y-axis
const int StepY = 3;
const int DirY = 6;

// Zoom - Z-axis
const int StepZ = 4;
const int DirZ = 7;

// Endstops (active LOW with INPUT_PULLUP)
const int EndstopX = 9;  // Pitch (Tilt) endstop
const int EndstopY = 10; // Yaw (Pan) endstop

const int EndstopDefaultPos = 0;

// EEPROM addresses for preset storage
// Each preset stores: pitch (2 bytes), yaw (2 bytes), zoom (2 bytes) = 6 bytes each
// 4 presets = 24 bytes total
// Magic number (2 bytes) + version (1 byte) = 3 bytes header
// Limits: 12 bytes (6 limits × 2 bytes each)
// Offsets: 6 bytes (3 offsets × 2 bytes each)
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_VERSION_ADDR = 2;
const int EEPROM_PRESET_START = 3;
const int EEPROM_LIMITS_START = 27;  // After presets (3 + 24)
const int EEPROM_OFFSETS_START = 39;  // After limits (27 + 12)
const int EEPROM_MAGIC = 0xCAFE;  // Magic number to detect valid data
const int EEPROM_VERSION = 2;  // Incremented for new data structure
const int PRESET_SIZE = 6;  // 2 bytes per axis (pitch, yaw, zoom)
const int NUM_PRESETS = 4;

// Serial communication
createSafeStringReader(sfReader, 16, " ");

// Stepper motor direction enums
enum StepperDirection {
  DIR_STOP = 0,
  DIR_FORWARD = 1,
  DIR_REVERSE = 2
};

enum ZoomDirection {
  ZOOM_OUT = 1,
  ZOOM_IN = 2,
  ZOOM_STOP = 0
};

// Global state variables
int BlockUserInput = 0;
int SetpointStarted = 0;
bool EmergencyStop = false;  // Emergency stop flag

// Soft position limits (stored in EEPROM, configurable)
int PitchMinLimit = -10000;   // Minimum pitch position
int PitchMaxLimit = 10000;     // Maximum pitch position
int YawMinLimit = -20000;      // Minimum yaw position
int YawMaxLimit = 20000;       // Maximum yaw position
int ZoomMinLimit = 0;          // Minimum zoom position
int ZoomMaxLimit = 2000;       // Maximum zoom position

// Position offsets (calibration)
int PitchOffset = 0;
int YawOffset = 0;
int ZoomOffset = 0;

// Acceleration ramping
const int ACCELERATION_STEP = 50;  // Speed change per update (microseconds)
const int ACCELERATION_INTERVAL = 1000;  // Update interval (microseconds)
unsigned long PitchAccelTimer = 0;
unsigned long YawAccelTimer = 0;
int PitchTargetSpeed = 2000;
int YawTargetSpeed = 1800 * 4;
int ZoomTargetSpeed = 2000;

// Speed interpolation for setpoint motion
const int SETPOINT_FAST_DISTANCE = 500;  // Distance threshold for fast speed
const int SETPOINT_SLOW_DISTANCE = 50;   // Distance threshold for slow speed
const float SETPOINT_FAST_MULTIPLIER = 1.0;  // Fast speed multiplier
const float SETPOINT_SLOW_MULTIPLIER = 0.3;  // Slow speed multiplier (30% for precision)

// Movement interpolation mode
bool SmoothInterpolationEnabled = true;  // Enable smooth S-curve interpolation

// Pitch (Tilt) stepper state
unsigned long PitchStepTimer = 0;
int iStepperPitchSpeed = 2000;  // microseconds per step (half period)
int iStepperPitchMove = DIR_STOP;
int iStepperPitchPos = EndstopDefaultPos;
int StoredPitchSpeed = 2000 * 1.5;
int StoredPitchPos = 0;
int StoredPitchPosB = 0;
int StoredPitchPosC = 0;
int StoredPitchPosD = 0;
int TargetPitchPos = 0;

// Joystick control for proportional movement
int JoystickPitchValue = 0;  // -32768 to 32768 (normalized from Python)
int JoystickYawValue = 0;    // -32768 to 32768
int JoystickZoomValue = 0;   // -32768 to 32768 (negative = zoom out, positive = zoom in)
const int JOYSTICK_MAX = 32768;
const int JOYSTICK_DEADZONE = JOYSTICK_MAX * 0.06;  // 6% deadzone

// Maximum speeds (microseconds per step - lower = faster)
const int PITCH_MAX_SPEED = 2000;      // Fastest pitch speed
const int PITCH_MIN_SPEED = 10000;     // Slowest pitch speed (for fine control)
const int YAW_MAX_SPEED = 1800 * 4;    // Fastest yaw speed
const int YAW_MIN_SPEED = 1800 * 4 * 3; // Slowest yaw speed
const int ZOOM_MAX_SPEED = 2000;       // Fastest zoom speed (milliseconds)
const int ZOOM_MIN_SPEED = 5000;       // Slowest zoom speed

// Yaw (Pan) stepper state
unsigned long YawStepTimer = 0;
int iStepperYawSpeed = 1800 * 4;  // microseconds per step (half period)
int iStepperYawMove = DIR_STOP;
int iStepperYawPos = EndstopDefaultPos;
int StoredYawSpeed = 2000 * 1;
int StoredYawPos = 0;
int StoredYawPosB = 0;
int StoredYawPosC = 0;
int StoredYawPosD = 0;
int TargetYawPos = 0;

// Zoom stepper state
unsigned long ZoomStepTimer = 0;
int iStepperZoomSpeed = 2000 * 0.65;  // milliseconds per step
ZoomDirection iStepperZoomMove = ZOOM_STOP;
int iStepperZoomPos = 0;
int StoredZoomSpeed = 2000 * 1;
int StoredZoomPos = 0;
int StoredZoomPosB = 0;
int StoredZoomPosC = 0;
int StoredZoomPosD = 0;
int TargetZoomPos = 0;
int StoredZoomAStop = 0;
int StoredZoomBStop = 1490;

// Zeroing state for zoom (non-blocking)
bool ZeroZoomInProgress = false;
int ZeroZoomStepsRemaining = 0;

// Zeroing state for pitch and yaw endstops (non-blocking)
bool ZeroPitchInProgress = false;
bool ZeroYawInProgress = false;
bool ZeroPitchComplete = false;
bool ZeroYawComplete = false;
bool ZeroPitchTimeout = false;
bool ZeroYawTimeout = false;
int ZeroPitchBackoffSteps = 0;
int ZeroYawBackoffSteps = 0;
int ZeroPitchStepsTaken = 0;
int ZeroYawStepsTaken = 0;
const int ENDSTOP_BACKOFF_STEPS = 100;  // Steps to back off after hitting endstop
const int ENDSTOP_ZERO_SPEED = 2000;    // Speed for zeroing (microseconds per step)

// Maximum steps for zeroing (timeout protection)
// Adjust these based on your stepper motor configuration:
// 
// Calculation: steps = (degrees / 360) * (steps_per_rev * microstepping)
// 
// Example with typical NEMA 17 stepper:
//   - Base: 200 steps/revolution (1.8 degrees per step)
//   - With 16x microstepping: 200 * 16 = 3200 steps/revolution
//   - 90 degrees = (90/360) * 3200 = 800 steps
//   - 180 degrees = (180/360) * 3200 = 1600 steps
//
// If using different microstepping or gear ratios, recalculate:
//   - 8x microstepping: 200 * 8 = 1600 steps/rev, 90° = 400 steps, 180° = 800 steps
//   - 32x microstepping: 200 * 32 = 6400 steps/rev, 90° = 1600 steps, 180° = 3200 steps
//   - With gear reduction: multiply by gear ratio (e.g., 2:1 gear = double the steps)
//
const int MAX_PITCH_ZERO_STEPS = 800;   // Maximum steps for 90 degrees pitch (down)
const int MAX_YAW_ZERO_STEPS = 1600;    // Maximum steps for 180 degrees yaw (pan)

// Debug output control
const bool ENABLE_DEBUG_OUTPUT = true;  // Set to false to disable Serial.println() calls
const int SERIAL_BUFFER_MIN = 32;       // Minimum free bytes in serial buffer before printing

void setup() {
  // Initialize pins
  pinMode(StepX, OUTPUT);
  pinMode(DirX, OUTPUT);
  pinMode(StepY, OUTPUT);
  pinMode(DirY, OUTPUT);
  pinMode(StepZ, OUTPUT);
  pinMode(DirZ, OUTPUT);
  
  // Endstops (active LOW when pressed, use INPUT_PULLUP)
  pinMode(EndstopX, INPUT_PULLUP);
  pinMode(EndstopY, INPUT_PULLUP);

  // Initialize serial communication
  Serial.begin(115200);
  SafeString::setOutput(Serial);
  sfReader.setTimeout(1000);
  sfReader.flushInput();
  sfReader.connect(Serial);

  // Initialize timers (use micros for pitch/yaw, millis for zoom)
  PitchStepTimer = micros();
  YawStepTimer = micros();
  ZoomStepTimer = millis();
  PitchAccelTimer = micros();
  YawAccelTimer = micros();
  
  // Load preset positions, limits, and offsets from EEPROM
  load_presets_from_eeprom();
  
  // Initialize target speeds for acceleration ramping
  PitchTargetSpeed = iStepperPitchSpeed;
  YawTargetSpeed = iStepperYawSpeed;
  ZoomTargetSpeed = iStepperZoomSpeed;
  
  // Start zeroing operations on bootup (non-blocking)
  start_zero_pitch_pos();
  start_zero_yaw_pos();
  start_zero_zoom_pos();
}

// Non-blocking timer check for pitch stepper (uses microseconds for precision)
bool can_step_pitch(int interval_us) {
  return ((micros() - PitchStepTimer) >= (unsigned long)interval_us);
}

// Non-blocking timer check for yaw stepper (uses microseconds for precision)
bool can_step_yaw(int interval_us) {
  return ((micros() - YawStepTimer) >= (unsigned long)interval_us);
}

// Non-blocking timer check for zoom stepper (uses milliseconds)
bool can_step_zoom(int interval_ms) {
  return ((millis() - ZoomStepTimer) >= (unsigned long)interval_ms);
}

// Safe debug print function - only prints if buffer has space
void debug_print(const char* message) {
  if (ENABLE_DEBUG_OUTPUT && Serial.availableForWrite() >= SERIAL_BUFFER_MIN) {
    Serial.println(message);
  }
}

// Check if position is within soft limits
bool check_pitch_limit(int pos) {
  return (pos >= PitchMinLimit && pos <= PitchMaxLimit);
}

bool check_yaw_limit(int pos) {
  return (pos >= YawMinLimit && pos <= YawMaxLimit);
}

bool check_zoom_limit(int pos) {
  return (pos >= ZoomMinLimit && pos <= ZoomMaxLimit);
}

// Apply position offsets
int apply_pitch_offset(int pos) {
  return pos + PitchOffset;
}

int apply_yaw_offset(int pos) {
  return pos + YawOffset;
}

int apply_zoom_offset(int pos) {
  return pos + ZoomOffset;
}

// Get position with offset applied (for reporting)
int get_pitch_position() {
  return apply_pitch_offset(iStepperPitchPos);
}

int get_yaw_position() {
  return apply_yaw_offset(iStepperYawPos);
}

int get_zoom_position() {
  return apply_zoom_offset(iStepperZoomPos);
}

// Emergency stop - stops all movement immediately
void emergency_stop() {
  EmergencyStop = true;
  iStepperPitchMove = DIR_STOP;
  iStepperYawMove = DIR_STOP;
  iStepperZoomMove = ZOOM_STOP;
  SetpointStarted = 0;
  BlockUserInput = 0;
  debug_print("EMERGENCY STOP ACTIVATED");
}

// Clear emergency stop
void clear_emergency_stop() {
  EmergencyStop = false;
  debug_print("Emergency stop cleared");
}

// Check endstops during normal operation (safety feature)
void check_endstops_during_operation() {
  if (EmergencyStop || ZeroPitchInProgress || ZeroYawInProgress) {
    return;  // Don't check during zeroing
  }
  
  // Check pitch endstop
  if (digitalRead(EndstopX) == LOW) {
    // Endstop triggered - stop pitch movement
    if (iStepperPitchMove == DIR_REVERSE) {  // Only stop if moving towards endstop
      iStepperPitchMove = DIR_STOP;
      debug_print("WARNING: Pitch endstop triggered during operation");
    }
  }
  
  // Check yaw endstop
  if (digitalRead(EndstopY) == LOW) {
    // Endstop triggered - stop yaw movement
    if (iStepperYawMove == DIR_REVERSE) {  // Only stop if moving towards endstop
      iStepperYawMove = DIR_STOP;
      debug_print("WARNING: Yaw endstop triggered during operation");
    }
  }
}

// Acceleration ramping - gradually change speed towards target
void update_acceleration_ramping() {
  if (EmergencyStop) return;
  
  // Pitch acceleration
  if (micros() - PitchAccelTimer >= ACCELERATION_INTERVAL) {
    if (iStepperPitchSpeed < PitchTargetSpeed) {
      iStepperPitchSpeed = min(iStepperPitchSpeed + ACCELERATION_STEP, PitchTargetSpeed);
    } else if (iStepperPitchSpeed > PitchTargetSpeed) {
      iStepperPitchSpeed = max(iStepperPitchSpeed - ACCELERATION_STEP, PitchTargetSpeed);
    }
    PitchAccelTimer = micros();
  }
  
  // Yaw acceleration
  if (micros() - YawAccelTimer >= ACCELERATION_INTERVAL) {
    if (iStepperYawSpeed < YawTargetSpeed) {
      iStepperYawSpeed = min(iStepperYawSpeed + ACCELERATION_STEP, YawTargetSpeed);
    } else if (iStepperYawSpeed > YawTargetSpeed) {
      iStepperYawSpeed = max(iStepperYawSpeed - ACCELERATION_STEP, YawTargetSpeed);
    }
    YawAccelTimer = micros();
  }
}

// Calculate interpolated speed for setpoint motion based on distance
// Uses S-curve interpolation for smooth acceleration/deceleration
int calculate_setpoint_speed(int currentPos, int targetPos, int baseSpeed) {
  if (!SmoothInterpolationEnabled) {
    return baseSpeed;  // Return base speed if interpolation disabled
  }
  
  int distance = abs(targetPos - currentPos);
  
  if (distance == 0) {
    return baseSpeed;
  }
  
  // Distance-based speed interpolation with S-curve smoothing
  float speedMultiplier;
  if (distance > SETPOINT_FAST_DISTANCE) {
    // Far from target - use fast speed
    speedMultiplier = SETPOINT_FAST_MULTIPLIER;
  } else if (distance < SETPOINT_SLOW_DISTANCE) {
    // Close to target - use slow speed for precision
    speedMultiplier = SETPOINT_SLOW_MULTIPLIER;
  } else {
    // In between - interpolate with S-curve for smooth transition
    float ratio = (float)(distance - SETPOINT_SLOW_DISTANCE) / (float)(SETPOINT_FAST_DISTANCE - SETPOINT_SLOW_DISTANCE);
    // Apply S-curve easing: smooth acceleration and deceleration
    float easedRatio;
    if (ratio < 0.5) {
      // Acceleration phase
      easedRatio = 2.0 * ratio * ratio;
    } else {
      // Deceleration phase
      easedRatio = 1.0 - 2.0 * (1.0 - ratio) * (1.0 - ratio);
    }
    speedMultiplier = SETPOINT_SLOW_MULTIPLIER + (SETPOINT_FAST_MULTIPLIER - SETPOINT_SLOW_MULTIPLIER) * easedRatio;
  }
  
  return (int)(baseSpeed * speedMultiplier);
}

// EEPROM Functions
void save_presets_to_eeprom() {
  // Write magic number and version
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  EEPROM.put(EEPROM_VERSION_ADDR, EEPROM_VERSION);
  
  // Save all 4 presets
  int addr = EEPROM_PRESET_START;
  EEPROM.put(addr, StoredPitchPos); addr += 2;
  EEPROM.put(addr, StoredYawPos); addr += 2;
  EEPROM.put(addr, StoredZoomPos); addr += 2;
  
  EEPROM.put(addr, StoredPitchPosB); addr += 2;
  EEPROM.put(addr, StoredYawPosB); addr += 2;
  EEPROM.put(addr, StoredZoomPosB); addr += 2;
  
  EEPROM.put(addr, StoredPitchPosC); addr += 2;
  EEPROM.put(addr, StoredYawPosC); addr += 2;
  EEPROM.put(addr, StoredZoomPosC); addr += 2;
  
  EEPROM.put(addr, StoredPitchPosD); addr += 2;
  EEPROM.put(addr, StoredYawPosD); addr += 2;
  EEPROM.put(addr, StoredZoomPosD);
  
  // Save limits
  addr = EEPROM_LIMITS_START;
  EEPROM.put(addr, PitchMinLimit); addr += 2;
  EEPROM.put(addr, PitchMaxLimit); addr += 2;
  EEPROM.put(addr, YawMinLimit); addr += 2;
  EEPROM.put(addr, YawMaxLimit); addr += 2;
  EEPROM.put(addr, ZoomMinLimit); addr += 2;
  EEPROM.put(addr, ZoomMaxLimit);
  
  // Save offsets
  addr = EEPROM_OFFSETS_START;
  EEPROM.put(addr, PitchOffset); addr += 2;
  EEPROM.put(addr, YawOffset); addr += 2;
  EEPROM.put(addr, ZoomOffset);
}

void load_presets_from_eeprom() {
  // Check if EEPROM has valid data
  int magic = 0;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  int version = 0;
  EEPROM.get(EEPROM_VERSION_ADDR, version);
  
  if (magic == EEPROM_MAGIC && version >= 1) {
    // Load all 4 presets
    int addr = EEPROM_PRESET_START;
    EEPROM.get(addr, StoredPitchPos); addr += 2;
    EEPROM.get(addr, StoredYawPos); addr += 2;
    EEPROM.get(addr, StoredZoomPos); addr += 2;
    
    EEPROM.get(addr, StoredPitchPosB); addr += 2;
    EEPROM.get(addr, StoredYawPosB); addr += 2;
    EEPROM.get(addr, StoredZoomPosB); addr += 2;
    
    EEPROM.get(addr, StoredPitchPosC); addr += 2;
    EEPROM.get(addr, StoredYawPosC); addr += 2;
    EEPROM.get(addr, StoredZoomPosC); addr += 2;
    
    EEPROM.get(addr, StoredPitchPosD); addr += 2;
    EEPROM.get(addr, StoredYawPosD); addr += 2;
    EEPROM.get(addr, StoredZoomPosD);
    
    // Load limits and offsets if version >= 2
    if (version >= 2) {
      addr = EEPROM_LIMITS_START;
      EEPROM.get(addr, PitchMinLimit); addr += 2;
      EEPROM.get(addr, PitchMaxLimit); addr += 2;
      EEPROM.get(addr, YawMinLimit); addr += 2;
      EEPROM.get(addr, YawMaxLimit); addr += 2;
      EEPROM.get(addr, ZoomMinLimit); addr += 2;
      EEPROM.get(addr, ZoomMaxLimit);
      
      addr = EEPROM_OFFSETS_START;
      EEPROM.get(addr, PitchOffset); addr += 2;
      EEPROM.get(addr, YawOffset); addr += 2;
      EEPROM.get(addr, ZoomOffset);
    }
  }
  // If magic number doesn't match, use default values (already initialized)
}

// Start zeroing pitch position using endstop (non-blocking)
void start_zero_pitch_pos() {
  ZeroPitchInProgress = true;
  ZeroPitchComplete = false;
  ZeroPitchTimeout = false;
  ZeroPitchBackoffSteps = 0;
  ZeroPitchStepsTaken = 0;
  // Move towards endstop (reverse direction - typically endstop is at minimum position)
  iStepperPitchMove = DIR_REVERSE;
  iStepperPitchSpeed = ENDSTOP_ZERO_SPEED;
}

// Start zeroing yaw position using endstop (non-blocking)
void start_zero_yaw_pos() {
  ZeroYawInProgress = true;
  ZeroYawComplete = false;
  ZeroYawTimeout = false;
  ZeroYawBackoffSteps = 0;
  ZeroYawStepsTaken = 0;
  // Move towards endstop (reverse direction - typically endstop is at minimum position)
  iStepperYawMove = DIR_REVERSE;
  iStepperYawSpeed = ENDSTOP_ZERO_SPEED;
}

// Start zeroing zoom position (non-blocking)
void start_zero_zoom_pos() {
  ZeroZoomInProgress = true;
  ZeroZoomStepsRemaining = 1140;
  digitalWrite(DirZ, HIGH); // Zoom out direction
  iStepperZoomMove = ZOOM_OUT;
}

// Handle non-blocking zero zoom operation
void handle_zero_zoom() {
  if (ZeroZoomInProgress && can_step_zoom(iStepperZoomSpeed)) {
    // Set direction (zoom out)
    digitalWrite(DirZ, HIGH);
    
    // Generate step pulse
    digitalWrite(StepZ, !digitalRead(StepZ));
    ZoomStepTimer = millis();
    iStepperZoomPos -= 1;
    ZeroZoomStepsRemaining--;
    
    if (ZeroZoomStepsRemaining <= 0) {
      iStepperZoomMove = ZOOM_STOP;
      iStepperZoomPos = 0;
      ZeroZoomInProgress = false;
    }
  }
}

// Handle non-blocking pitch zeroing with endstop
void handle_zero_pitch() {
  if (!ZeroPitchInProgress || ZeroPitchComplete || ZeroPitchTimeout) {
    return;
  }
  
  // Check for timeout (maximum steps reached)
  if (ZeroPitchStepsTaken >= MAX_PITCH_ZERO_STEPS) {
    // Timeout reached - stop zeroing and report error
    iStepperPitchMove = DIR_STOP;
    ZeroPitchInProgress = false;
    ZeroPitchComplete = false;
    ZeroPitchTimeout = true;
      iStepperPitchSpeed = StoredPitchSpeed; // Restore normal speed
      debug_print("ERROR: Pitch zeroing timeout - endstop not reached within 90 degrees");
      return;
  }
  
  int endstopState = digitalRead(EndstopX);
  
  if (endstopState == LOW) {
    // Endstop is pressed (active LOW)
    if (ZeroPitchBackoffSteps < ENDSTOP_BACKOFF_STEPS) {
      // Back off from endstop
      if (can_step_pitch(ENDSTOP_ZERO_SPEED)) {
        digitalWrite(DirX, HIGH); // Move forward (away from endstop)
        digitalWrite(StepX, !digitalRead(StepX));
        PitchStepTimer = micros();
        iStepperPitchPos += 1;
        ZeroPitchBackoffSteps++;
      }
    } else {
      // Backoff complete, zeroing is done
      iStepperPitchPos = EndstopDefaultPos;
      iStepperPitchMove = DIR_STOP;
      ZeroPitchInProgress = false;
      ZeroPitchComplete = true;
      iStepperPitchSpeed = StoredPitchSpeed; // Restore normal speed
      debug_print("Pitch zeroing complete");
    }
  } else {
    // Endstop not pressed, continue moving towards it
    if (can_step_pitch(ENDSTOP_ZERO_SPEED)) {
      digitalWrite(DirX, LOW); // Move reverse (towards endstop)
      digitalWrite(StepX, !digitalRead(StepX));
      PitchStepTimer = micros();
      iStepperPitchPos -= 1;
      ZeroPitchStepsTaken++; // Increment step counter
    }
  }
}

// Handle pitch (tilt) stepper - non-blocking
void handle_pitch_stepper() {
  // Don't allow normal movement during zeroing (unless timeout occurred)
  if (ZeroPitchInProgress && !ZeroPitchComplete && !ZeroPitchTimeout) {
    return;
  }
  
  if (EmergencyStop) {
    iStepperPitchMove = DIR_STOP;
    return;
  }
  
  if (BlockUserInput > 0 && SetpointStarted == 0) {
    return;
  }

  if (iStepperPitchMove != DIR_STOP && can_step_pitch(iStepperPitchSpeed)) {
    // Check soft limits before moving
    int nextPos = iStepperPitchPos;
    if (iStepperPitchMove == DIR_FORWARD) {
      nextPos += 1;
      if (!check_pitch_limit(nextPos)) {
        iStepperPitchMove = DIR_STOP;
        debug_print("Pitch max limit reached");
        return;
      }
    } else if (iStepperPitchMove == DIR_REVERSE) {
      nextPos -= 1;
      if (!check_pitch_limit(nextPos)) {
        iStepperPitchMove = DIR_STOP;
        debug_print("Pitch min limit reached");
        return;
      }
    }
    
    // Set direction
    if (iStepperPitchMove == DIR_FORWARD) {
      digitalWrite(DirX, HIGH);
    } else if (iStepperPitchMove == DIR_REVERSE) {
      digitalWrite(DirX, LOW);
    }
    
    // Generate step pulse (toggle)
    digitalWrite(StepX, !digitalRead(StepX));
    PitchStepTimer = micros();
    
    // Update position
    if (iStepperPitchMove == DIR_FORWARD) {
      iStepperPitchPos += 1;
    } else if (iStepperPitchMove == DIR_REVERSE) {
      iStepperPitchPos -= 1;
    }
  }
}

// Handle non-blocking yaw zeroing with endstop
void handle_zero_yaw() {
  if (!ZeroYawInProgress || ZeroYawComplete || ZeroYawTimeout) {
    return;
  }
  
  // Check for timeout (maximum steps reached)
  if (ZeroYawStepsTaken >= MAX_YAW_ZERO_STEPS) {
    // Timeout reached - stop zeroing and report error
    iStepperYawMove = DIR_STOP;
    ZeroYawInProgress = false;
    ZeroYawComplete = false;
    ZeroYawTimeout = true;
      iStepperYawSpeed = StoredYawSpeed; // Restore normal speed
      debug_print("ERROR: Yaw zeroing timeout - endstop not reached within 180 degrees");
      return;
  }
  
  int endstopState = digitalRead(EndstopY);
  
  if (endstopState == LOW) {
    // Endstop is pressed (active LOW)
    if (ZeroYawBackoffSteps < ENDSTOP_BACKOFF_STEPS) {
      // Back off from endstop
      if (can_step_yaw(ENDSTOP_ZERO_SPEED)) {
        digitalWrite(DirY, HIGH); // Move forward (away from endstop)
        digitalWrite(StepY, !digitalRead(StepY));
        YawStepTimer = micros();
        iStepperYawPos += 1;
        ZeroYawBackoffSteps++;
      }
    } else {
      // Backoff complete, zeroing is done
      iStepperYawPos = EndstopDefaultPos;
      iStepperYawMove = DIR_STOP;
      ZeroYawInProgress = false;
      ZeroYawComplete = true;
      iStepperYawSpeed = StoredYawSpeed; // Restore normal speed
      debug_print("Yaw zeroing complete");
    }
  } else {
    // Endstop not pressed, continue moving towards it
    if (can_step_yaw(ENDSTOP_ZERO_SPEED)) {
      digitalWrite(DirY, LOW); // Move reverse (towards endstop)
      digitalWrite(StepY, !digitalRead(StepY));
      YawStepTimer = micros();
      iStepperYawPos -= 1;
      ZeroYawStepsTaken++; // Increment step counter
    }
  }
}

// Handle yaw (pan) stepper - non-blocking
void handle_yaw_stepper() {
  // Don't allow normal movement during zeroing (unless timeout occurred)
  if (ZeroYawInProgress && !ZeroYawComplete && !ZeroYawTimeout) {
    return;
  }
  
  if (EmergencyStop) {
    iStepperYawMove = DIR_STOP;
    return;
  }
  
  if (BlockUserInput > 0 && SetpointStarted == 0) {
    return;
  }

  if (iStepperYawMove != DIR_STOP && can_step_yaw(iStepperYawSpeed)) {
    // Check soft limits before moving
    int nextPos = iStepperYawPos;
    if (iStepperYawMove == DIR_FORWARD) {
      nextPos += 1;
      if (!check_yaw_limit(nextPos)) {
        iStepperYawMove = DIR_STOP;
        debug_print("Yaw max limit reached");
        return;
      }
    } else if (iStepperYawMove == DIR_REVERSE) {
      nextPos -= 1;
      if (!check_yaw_limit(nextPos)) {
        iStepperYawMove = DIR_STOP;
        debug_print("Yaw min limit reached");
        return;
      }
    }
    
    // Set direction
    if (iStepperYawMove == DIR_FORWARD) {
      digitalWrite(DirY, HIGH);
    } else if (iStepperYawMove == DIR_REVERSE) {
      digitalWrite(DirY, LOW);
    }
    
    // Generate step pulse (toggle)
    digitalWrite(StepY, !digitalRead(StepY));
    YawStepTimer = micros();
    
    // Update position
    if (iStepperYawMove == DIR_FORWARD) {
      iStepperYawPos += 1;
    } else if (iStepperYawMove == DIR_REVERSE) {
      iStepperYawPos -= 1;
    }
  }
}

// Handle zoom stepper - non-blocking
void handle_zoom_stepper() {
  // Don't handle zoom if zeroing is in progress
  if (ZeroZoomInProgress) {
    return;
  }
  
  if (EmergencyStop) {
    iStepperZoomMove = ZOOM_STOP;
    return;
  }
  
  if (iStepperZoomMove != ZOOM_STOP && can_step_zoom(iStepperZoomSpeed)) {
    // Check soft limits before moving
    int nextPos = iStepperZoomPos;
    if (iStepperZoomMove == ZOOM_IN) {
      nextPos += 1;
      if (!check_zoom_limit(nextPos)) {
        iStepperZoomMove = ZOOM_STOP;
        debug_print("Zoom max limit reached");
        return;
      }
    } else if (iStepperZoomMove == ZOOM_OUT) {
      nextPos -= 1;
      if (!check_zoom_limit(nextPos)) {
        iStepperZoomMove = ZOOM_STOP;
        debug_print("Zoom min limit reached");
        return;
      }
    }
    
    // Set direction
    if (iStepperZoomMove == ZOOM_IN) {
      digitalWrite(DirZ, LOW);
    } else if (iStepperZoomMove == ZOOM_OUT) {
      digitalWrite(DirZ, HIGH);
    }
    
    // Generate step pulse (toggle)
    digitalWrite(StepZ, !digitalRead(StepZ));
    ZoomStepTimer = millis();
    
    // Update position
    if (iStepperZoomMove == ZOOM_IN) {
      iStepperZoomPos += 1;
    } else if (iStepperZoomMove == ZOOM_OUT) {
      iStepperZoomPos -= 1;
    }
  }
}

// Handle setpoint motion for all axes
void handle_setpoint_motion() {
  // Don't allow setpoint motion during zeroing operations (unless timeout occurred)
  // Timeout allows manual control even if zeroing failed
  if (((ZeroPitchInProgress && !ZeroPitchComplete && !ZeroPitchTimeout) || 
       (ZeroYawInProgress && !ZeroYawComplete && !ZeroYawTimeout) || 
       ZeroZoomInProgress)) {
    return;
  }
  
  if (SetpointStarted > 0) {
    BlockUserInput = 1;

    // Determine target positions based on setpoint (apply offsets)
    switch (SetpointStarted) {
      case 1:
        TargetPitchPos = StoredPitchPos - PitchOffset;  // Remove offset to get raw position
        TargetYawPos = StoredYawPos - YawOffset;
        TargetZoomPos = StoredZoomPos - ZoomOffset;
        break;
      case 2:
        TargetPitchPos = StoredPitchPosB - PitchOffset;
        TargetYawPos = StoredYawPosB - YawOffset;
        TargetZoomPos = StoredZoomPosB - ZoomOffset;
        break;
      case 3:
        TargetPitchPos = StoredPitchPosC - PitchOffset;
        TargetYawPos = StoredYawPosC - YawOffset;
        TargetZoomPos = StoredZoomPosC - ZoomOffset;
        break;
      case 4:
        TargetPitchPos = StoredPitchPosD - PitchOffset;
        TargetYawPos = StoredYawPosD - YawOffset;
        TargetZoomPos = StoredZoomPosD - ZoomOffset;
        break;
    }

    // Pitch movement logic with speed interpolation
    if (iStepperPitchPos < TargetPitchPos) {
      iStepperPitchMove = DIR_FORWARD;
      // Apply speed interpolation based on distance to target
      PitchTargetSpeed = calculate_setpoint_speed(iStepperPitchPos, TargetPitchPos, StoredPitchSpeed);
    } else if (iStepperPitchPos > TargetPitchPos) {
      iStepperPitchMove = DIR_REVERSE;
      // Apply speed interpolation based on distance to target
      PitchTargetSpeed = calculate_setpoint_speed(iStepperPitchPos, TargetPitchPos, StoredPitchSpeed);
    } else {
      iStepperPitchMove = DIR_STOP;
      PitchTargetSpeed = StoredPitchSpeed;
      iStepperPitchSpeed = StoredPitchSpeed;
    }

    // Yaw movement logic with speed interpolation
    if (iStepperYawPos < TargetYawPos) {
      iStepperYawMove = DIR_FORWARD;
      // Apply speed interpolation based on distance to target
      YawTargetSpeed = calculate_setpoint_speed(iStepperYawPos, TargetYawPos, StoredYawSpeed);
    } else if (iStepperYawPos > TargetYawPos) {
      iStepperYawMove = DIR_REVERSE;
      // Apply speed interpolation based on distance to target
      YawTargetSpeed = calculate_setpoint_speed(iStepperYawPos, TargetYawPos, StoredYawSpeed);
    } else {
      iStepperYawMove = DIR_STOP;
      YawTargetSpeed = StoredYawSpeed;
      iStepperYawSpeed = StoredYawSpeed;
    }

    // Zoom movement logic with speed interpolation
    if (iStepperZoomPos < TargetZoomPos) {
      iStepperZoomMove = ZOOM_IN;
      // Apply speed interpolation based on distance to target
      int distance = abs(TargetZoomPos - iStepperZoomPos);
      if (distance > SETPOINT_FAST_DISTANCE) {
        iStepperZoomSpeed = (int)(StoredZoomSpeed * SETPOINT_FAST_MULTIPLIER);
      } else if (distance < SETPOINT_SLOW_DISTANCE) {
        iStepperZoomSpeed = (int)(StoredZoomSpeed * SETPOINT_SLOW_MULTIPLIER);
      } else {
        float ratio = (float)(distance - SETPOINT_SLOW_DISTANCE) / (float)(SETPOINT_FAST_DISTANCE - SETPOINT_SLOW_DISTANCE);
        float speedMultiplier = SETPOINT_SLOW_MULTIPLIER + (SETPOINT_FAST_MULTIPLIER - SETPOINT_SLOW_MULTIPLIER) * ratio;
        iStepperZoomSpeed = (int)(StoredZoomSpeed * speedMultiplier);
      }
    } else if (iStepperZoomPos > TargetZoomPos) {
      iStepperZoomMove = ZOOM_OUT;
      // Apply speed interpolation based on distance to target
      int distance = abs(TargetZoomPos - iStepperZoomPos);
      if (distance > SETPOINT_FAST_DISTANCE) {
        iStepperZoomSpeed = (int)(StoredZoomSpeed * SETPOINT_FAST_MULTIPLIER);
      } else if (distance < SETPOINT_SLOW_DISTANCE) {
        iStepperZoomSpeed = (int)(StoredZoomSpeed * SETPOINT_SLOW_MULTIPLIER);
      } else {
        float ratio = (float)(distance - SETPOINT_SLOW_DISTANCE) / (float)(SETPOINT_FAST_DISTANCE - SETPOINT_SLOW_DISTANCE);
        float speedMultiplier = SETPOINT_SLOW_MULTIPLIER + (SETPOINT_FAST_MULTIPLIER - SETPOINT_SLOW_MULTIPLIER) * ratio;
        iStepperZoomSpeed = (int)(StoredZoomSpeed * speedMultiplier);
      }
    } else {
      iStepperZoomMove = ZOOM_STOP;
      iStepperZoomSpeed = StoredZoomSpeed;
    }

    // Check if all axes have reached their targets
    if ((iStepperPitchPos == TargetPitchPos) && 
        (iStepperYawPos == TargetYawPos) && 
        (iStepperZoomPos == TargetZoomPos)) {
      SetpointStarted = 0;
      BlockUserInput = 0;
      iStepperPitchMove = DIR_STOP;
      iStepperYawMove = DIR_STOP;
      iStepperZoomMove = ZOOM_STOP;
      iStepperPitchSpeed = StoredPitchSpeed;
      iStepperYawSpeed = StoredYawSpeed;
    }
  }
}

// Update movement based on joystick values (proportional control)
void update_movement_from_joystick() {
  // Don't update if zeroing is in progress
  if ((ZeroPitchInProgress && !ZeroPitchComplete && !ZeroPitchTimeout) ||
      (ZeroYawInProgress && !ZeroYawComplete && !ZeroYawTimeout) ||
      ZeroZoomInProgress) {
    return;
  }
  
  // Don't update during setpoint motion
  if (SetpointStarted > 0) {
    return;
  }
  
  // Handle Pitch (Y-axis joystick)
  if (abs(JoystickPitchValue) < JOYSTICK_DEADZONE) {
    iStepperPitchMove = DIR_STOP;
  } else {
    // Calculate proportional speed
    float pitchRatio = (float)abs(JoystickPitchValue) / (float)JOYSTICK_MAX;
    pitchRatio = constrain(pitchRatio, 0.0, 1.0);
    
    // Map ratio to speed range (inverse: lower microseconds = faster)
    int pitchSpeed = PITCH_MIN_SPEED - (int)((PITCH_MIN_SPEED - PITCH_MAX_SPEED) * pitchRatio);
    PitchTargetSpeed = constrain(pitchSpeed, PITCH_MAX_SPEED, PITCH_MIN_SPEED);
    // Speed will ramp via acceleration function
    
    // Set direction based on sign
    if (JoystickPitchValue > 0) {
      iStepperPitchMove = DIR_FORWARD;
    } else {
      iStepperPitchMove = DIR_REVERSE;
    }
  } else {
    iStepperPitchMove = DIR_STOP;
    PitchTargetSpeed = PITCH_MIN_SPEED;  // Slow to stop when joystick released
  }
  
  // Handle Yaw (X-axis joystick)
  if (abs(JoystickYawValue) < JOYSTICK_DEADZONE) {
    iStepperYawMove = DIR_STOP;
    YawTargetSpeed = YAW_MIN_SPEED;  // Slow to stop
  } else {
    // Calculate proportional speed
    float yawRatio = (float)abs(JoystickYawValue) / (float)JOYSTICK_MAX;
    yawRatio = constrain(yawRatio, 0.0, 1.0);
    
    // Map ratio to speed range (inverse: lower microseconds = faster)
    int yawSpeed = YAW_MIN_SPEED - (int)((YAW_MIN_SPEED - YAW_MAX_SPEED) * yawRatio);
    YawTargetSpeed = constrain(yawSpeed, YAW_MAX_SPEED, YAW_MIN_SPEED);
    // Speed will ramp via acceleration function
    
    // Set direction based on sign
    if (JoystickYawValue > 0) {
      iStepperYawMove = DIR_FORWARD;
    } else {
      iStepperYawMove = DIR_REVERSE;
    }
  }
  
  // Handle Zoom (trigger values)
  if (abs(JoystickZoomValue) < JOYSTICK_DEADZONE) {
    iStepperZoomMove = ZOOM_STOP;
  } else {
    // Calculate proportional speed
    float zoomRatio = (float)abs(JoystickZoomValue) / (float)JOYSTICK_MAX;
    zoomRatio = constrain(zoomRatio, 0.0, 1.0);
    
    // Map ratio to speed range (milliseconds - lower = faster)
    int zoomSpeed = ZOOM_MIN_SPEED - (int)((ZOOM_MIN_SPEED - ZOOM_MAX_SPEED) * zoomRatio);
    iStepperZoomSpeed = constrain(zoomSpeed, ZOOM_MAX_SPEED, ZOOM_MIN_SPEED);
    
    // Set direction based on sign
    if (JoystickZoomValue > 0) {
      iStepperZoomMove = ZOOM_IN;
    } else {
      iStepperZoomMove = ZOOM_OUT;
    }
  }
}

// Handle serial data input
void handle_data_input() {
  if (sfReader.read()) {
    if (sfReader == "info") {
      debug_print("consolidated_module");
    }
    
    // Joystick command: j,yaw,pitch,zoom where values are joystick positions (-32768 to 32768)
    else if (sfReader.startsWith("j,")) {
      // Parse joystick values: j,yaw,pitch,zoom
      // Remove "j," prefix
      sfReader.removeBefore(2);
      
      // Parse comma-separated values manually
      int yawEnd = -1;
      int pitchEnd = -1;
      
      // Find comma positions
      for (int i = 0; i < sfReader.length(); i++) {
        if (sfReader[i] == ',' && yawEnd == -1) {
          yawEnd = i;
        } else if (sfReader[i] == ',' && pitchEnd == -1) {
          pitchEnd = i;
          break;
        }
      }
      
      if (yawEnd > 0 && pitchEnd > yawEnd) {
        // Extract yaw value
        SafeString yawStr = sfReader.substring(0, yawEnd);
        yawStr.toInt(JoystickYawValue);
        
        // Extract pitch value
        SafeString pitchStr = sfReader.substring(yawEnd + 1, pitchEnd);
        pitchStr.toInt(JoystickPitchValue);
        
        // Extract zoom value (remaining after second comma)
        SafeString zoomStr = sfReader.substring(pitchEnd + 1);
        zoomStr.toInt(JoystickZoomValue);
        
        // Constrain values to valid range
        JoystickYawValue = constrain(JoystickYawValue, -JOYSTICK_MAX, JOYSTICK_MAX);
        JoystickPitchValue = constrain(JoystickPitchValue, -JOYSTICK_MAX, JOYSTICK_MAX);
        JoystickZoomValue = constrain(JoystickZoomValue, -JOYSTICK_MAX, JOYSTICK_MAX);
        
        // Update movement based on joystick values
        update_movement_from_joystick();
      }
    }
    
    // Legacy discrete commands (for backward compatibility)
    // Pitch control
    else if (sfReader == "a") {
      JoystickPitchValue = JOYSTICK_MAX; // Full forward
      update_movement_from_joystick();
    }
    else if (sfReader == "b") {
      JoystickPitchValue = -JOYSTICK_MAX; // Full reverse
      update_movement_from_joystick();
    }
    else if (sfReader == "c") {
      JoystickPitchValue = 0;
      update_movement_from_joystick();
    }
    
    // Yaw control
    else if (sfReader == "1") {
      JoystickYawValue = JOYSTICK_MAX; // Full forward
      update_movement_from_joystick();
    }
    else if (sfReader == "2") {
      JoystickYawValue = -JOYSTICK_MAX; // Full reverse
      update_movement_from_joystick();
    }
    else if (sfReader == "3") {
      JoystickYawValue = 0;
      update_movement_from_joystick();
    }
    
    // Zoom control
    else if (sfReader == "4") {
      JoystickZoomValue = -JOYSTICK_MAX; // Zoom out
      update_movement_from_joystick();
    }
    else if (sfReader == "5") {
      JoystickZoomValue = JOYSTICK_MAX; // Zoom in
      update_movement_from_joystick();
    }
    else if (sfReader == "6") {
      JoystickZoomValue = 0;
      update_movement_from_joystick();
    }
    
    // Set/move to target positions (also saves to EEPROM)
    // Store positions with offset applied (so they work correctly when loaded)
    else if (sfReader == "s") {
      StoredPitchSpeed = iStepperPitchSpeed;
      StoredYawSpeed = iStepperYawSpeed;
      StoredPitchPos = get_pitch_position();  // Store with offset
      StoredYawPos = get_yaw_position();
      StoredZoomPos = get_zoom_position();
      save_presets_to_eeprom();
    }
    else if (sfReader == "s2") {
      StoredPitchSpeed = iStepperPitchSpeed;
      StoredYawSpeed = iStepperYawSpeed;
      StoredPitchPosB = get_pitch_position();
      StoredYawPosB = get_yaw_position();
      StoredZoomPosB = get_zoom_position();
      save_presets_to_eeprom();
    }
    else if (sfReader == "s3") {
      StoredPitchSpeed = iStepperPitchSpeed;
      StoredYawSpeed = iStepperYawSpeed;
      StoredPitchPosC = get_pitch_position();
      StoredYawPosC = get_yaw_position();
      StoredZoomPosC = get_zoom_position();
      save_presets_to_eeprom();
    }
    else if (sfReader == "s4") {
      StoredPitchSpeed = iStepperPitchSpeed;
      StoredYawSpeed = iStepperYawSpeed;
      StoredPitchPosD = get_pitch_position();
      StoredYawPosD = get_yaw_position();
      StoredZoomPosD = get_zoom_position();
      save_presets_to_eeprom();
    }
    else if (sfReader == "t") {
      SetpointStarted = 1;
      iStepperPitchSpeed = 2000 * 1.5;
      iStepperYawSpeed = 2000 * 1;
    }
    else if (sfReader == "t2") {
      SetpointStarted = 2;
      iStepperPitchSpeed = 2000 * 1.5;
      iStepperYawSpeed = 2000 * 1;
    }
    else if (sfReader == "t3") {
      SetpointStarted = 3;
      iStepperPitchSpeed = 2000 * 1.5;
      iStepperYawSpeed = 2000 * 1;
    }
    else if (sfReader == "t4") {
      SetpointStarted = 4;
      iStepperPitchSpeed = 2000 * 1.5;
      iStepperYawSpeed = 2000 * 1;
    }
    
    // Speed control (legacy - now handled by joystick, but kept for compatibility)
    else if (sfReader.startsWith("p")) {
      sfReader.removeBefore(1);
      sfReader.toInt(iStepperPitchSpeed);
      // Note: This will be overridden by joystick control
    }
    else if (sfReader.startsWith("y")) {
      sfReader.removeBefore(1);
      sfReader.toInt(iStepperYawSpeed);
      // Note: This will be overridden by joystick control
    }
    else if (sfReader.startsWith("z")) {
      iStepperZoomSpeed = sfReader.substring(1).toInt();
      // Note: This will be overridden by joystick control
    }
    
    // Reset zoom position
    else if (sfReader == "ea") {
      iStepperZoomPos = 0;
      StoredZoomBStop = 1490;
    }
    
    // Update B stop position
    else if (sfReader == "eb") {
      StoredZoomBStop = iStepperZoomPos + 1;
    }
    
    // Flush input
    else if (sfReader == "flush") {
      sfReader.flushInput();
    }
    
    // Emergency stop
    else if (sfReader == "estop") {
      emergency_stop();
    }
    
    // Clear emergency stop
    else if (sfReader == "estop_clear") {
      clear_emergency_stop();
    }
    
    // Position query - returns current positions
    else if (sfReader == "pos") {
      if (Serial.availableForWrite() >= 50) {
        Serial.print("POS:");
        Serial.print(get_pitch_position());
        Serial.print(",");
        Serial.print(get_yaw_position());
        Serial.print(",");
        Serial.print(get_zoom_position());
        Serial.println();
      }
    }
    
    // Configuration commands
    // Set pitch limits: plim,min,max
    else if (sfReader.startsWith("plim,")) {
      sfReader.removeBefore(5);
      int comma = sfReader.indexOf(',');
      if (comma > 0) {
        SafeString minStr = sfReader.substring(0, comma);
        SafeString maxStr = sfReader.substring(comma + 1);
        minStr.toInt(PitchMinLimit);
        maxStr.toInt(PitchMaxLimit);
        save_presets_to_eeprom();
        debug_print("Pitch limits updated");
      }
    }
    
    // Set yaw limits: ylim,min,max
    else if (sfReader.startsWith("ylim,")) {
      sfReader.removeBefore(5);
      int comma = sfReader.indexOf(',');
      if (comma > 0) {
        SafeString minStr = sfReader.substring(0, comma);
        SafeString maxStr = sfReader.substring(comma + 1);
        minStr.toInt(YawMinLimit);
        maxStr.toInt(YawMaxLimit);
        save_presets_to_eeprom();
        debug_print("Yaw limits updated");
      }
    }
    
    // Set zoom limits: zlim,min,max
    else if (sfReader.startsWith("zlim,")) {
      sfReader.removeBefore(5);
      int comma = sfReader.indexOf(',');
      if (comma > 0) {
        SafeString minStr = sfReader.substring(0, comma);
        SafeString maxStr = sfReader.substring(comma + 1);
        minStr.toInt(ZoomMinLimit);
        maxStr.toInt(ZoomMaxLimit);
        save_presets_to_eeprom();
        debug_print("Zoom limits updated");
      }
    }
    
    // Set position offsets: poff,value (pitch offset)
    else if (sfReader.startsWith("poff,")) {
      sfReader.removeBefore(5);
      sfReader.toInt(PitchOffset);
      save_presets_to_eeprom();
      debug_print("Pitch offset updated");
    }
    
    // Set yaw offset: yoff,value
    else if (sfReader.startsWith("yoff,")) {
      sfReader.removeBefore(5);
      sfReader.toInt(YawOffset);
      save_presets_to_eeprom();
      debug_print("Yaw offset updated");
    }
    
    // Set zoom offset: zoff,value
    else if (sfReader.startsWith("zoff,")) {
      sfReader.removeBefore(5);
      sfReader.toInt(ZoomOffset);
      save_presets_to_eeprom();
      debug_print("Zoom offset updated");
    }
    
    // Query limits: limits
    else if (sfReader == "limits") {
      if (Serial.availableForWrite() >= 100) {
        Serial.print("LIMITS:P,");
        Serial.print(PitchMinLimit);
        Serial.print(",");
        Serial.print(PitchMaxLimit);
        Serial.print(" Y,");
        Serial.print(YawMinLimit);
        Serial.print(",");
        Serial.print(YawMaxLimit);
        Serial.print(" Z,");
        Serial.print(ZoomMinLimit);
        Serial.print(",");
        Serial.print(ZoomMaxLimit);
        Serial.println();
      }
    }
    
    // Query offsets: offsets
    else if (sfReader == "offsets") {
      if (Serial.availableForWrite() >= 50) {
        Serial.print("OFFSETS:P,");
        Serial.print(PitchOffset);
        Serial.print(" Y,");
        Serial.print(YawOffset);
        Serial.print(" Z,");
        Serial.print(ZoomOffset);
        Serial.println();
      }
    }
    
    // Set acceleration step: accel,value
    else if (sfReader.startsWith("accel,")) {
      // Note: This would require making ACCELERATION_STEP non-const
      // For now, just acknowledge
      debug_print("Acceleration step is fixed in code");
    }
    
    // Toggle smooth interpolation: smooth,0 or smooth,1
    else if (sfReader.startsWith("smooth,")) {
      sfReader.removeBefore(7);
      int value = 0;
      sfReader.toInt(value);
      SmoothInterpolationEnabled = (value != 0);
      if (SmoothInterpolationEnabled) {
        debug_print("Smooth interpolation enabled");
      } else {
        debug_print("Smooth interpolation disabled");
      }
    }
    
    // Status query: status
    else if (sfReader == "status") {
      if (Serial.availableForWrite() >= 150) {
        Serial.print("STATUS:");
        Serial.print("P,");
        Serial.print(get_pitch_position());
        Serial.print(",M,");
        Serial.print(iStepperPitchMove);
        Serial.print(",S,");
        Serial.print(iStepperPitchSpeed);
        Serial.print(" Y,");
        Serial.print(get_yaw_position());
        Serial.print(",M,");
        Serial.print(iStepperYawMove);
        Serial.print(",S,");
        Serial.print(iStepperYawSpeed);
        Serial.print(" Z,");
        Serial.print(get_zoom_position());
        Serial.print(",M,");
        Serial.print(iStepperZoomMove);
        Serial.print(",S,");
        Serial.print(iStepperZoomSpeed);
        Serial.print(" ESTOP,");
        Serial.print(EmergencyStop ? 1 : 0);
        Serial.print(" SETPOINT,");
        Serial.print(SetpointStarted);
        Serial.println();
      }
    }
  }
}

// Main loop - all operations are non-blocking
void loop() {
  // Handle zeroing operations if in progress
  if (ZeroPitchInProgress && !ZeroPitchComplete) {
    handle_zero_pitch();
  }
  
  if (ZeroYawInProgress && !ZeroYawComplete) {
    handle_zero_yaw();
  }
  
  if (ZeroZoomInProgress) {
    handle_zero_zoom();
  }
  
  // Check endstops during normal operation (safety)
  if (!EmergencyStop) {
    check_endstops_during_operation();
  }
  
  // Update acceleration ramping
  update_acceleration_ramping();
  
  // Handle serial input
  handle_data_input();
  
  // Handle all stepper motors (non-blocking)
  handle_pitch_stepper();
  handle_yaw_stepper();
  handle_zoom_stepper();
  
  // Handle setpoint motion
  handle_setpoint_motion();
}

