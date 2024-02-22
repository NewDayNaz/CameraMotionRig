#include "SafeStringReader.h"

// Pin definitions
const int StepZ =  4;
const int DirZ =  7;

// Stepper motor control
createSafeStringReader(sfReader,  16, " "); // Reader for up to  20 chars, tokens terminated by space or timeout
unsigned long StepTimer;

// Zoom control
enum ZoomDirection {
  ZOOM_OUT =  1,
  ZOOM_IN =  2,
  ZOOM_STOP =  0
};

enum Setpoint {
  SETPOINT_A =  1,
  SETPOINT_B =  2,
  SETPOINT_C =  3,
  SETPOINT_D =  4
};

int BlockUserInput =  0;
int SetpointStarted =  0;

int StoredZoomAStop =  0;
int StoredZoomBStop =  1490;

int StoredZoomSpeed =  2000 *  1;
int TargetZoomPos =  0;
int StoredZoomPos =  0;
int StoredZoomPosB =  0;
int StoredZoomPosC =  0;
int StoredZoomPosD =  0;

// Stepper motor state
int iStepperSpeedRamp =  0;
int iStepperZoomSpeed =  2000 *  0.65;
ZoomDirection iStepperZoomMove = ZOOM_STOP;
int iStepperZoomPos =  0;

void setup() {
  // Initialize pins
  pinMode(StepZ, OUTPUT);
  pinMode(DirZ, OUTPUT);

  // Initialize serial communication
  Serial.begin(115200);
  SafeString::setOutput(Serial);
  sfReader.setTimeout(1000);
  sfReader.flushInput();
  sfReader.connect(Serial);

  // Initialize stepper motor
  StepTimer = millis();
  zero_zoom_pos();
}

bool can_we_step_zoom(int interval) {
  return ((millis() - StepTimer) >= interval);
}

void step_zoom_stepper() {
  digitalWrite(StepZ, !digitalRead(StepZ));
  StepTimer = millis();
}

void handle_zoom_stepper() {
  if (iStepperZoomMove != ZOOM_STOP && can_we_step_zoom(iStepperZoomSpeed)) {
    step_zoom_stepper();
    iStepperZoomPos += (iStepperZoomMove == ZOOM_IN) ?  1 : -1;
  }
}

void zero_zoom_pos() {
  digitalWrite(DirZ, HIGH); // Zoom out
  for (int i =  0; i <=  1140; i++) {
    if (can_we_step_zoom(iStepperZoomSpeed)) {
      step_zoom_stepper();
    } else {
      i--;
    }
  }
  iStepperZoomPos =  0;
}

void handle_setpoint_motion() {
  if (SetpointStarted >  0) {
    BlockUserInput =  1;

    // Determine target position based on setpoint
    switch (SetpointStarted) {
      case SETPOINT_A:
        TargetZoomPos = StoredZoomPos;
        break;
      case SETPOINT_B:
        TargetZoomPos = StoredZoomPosB;
        break;
      case SETPOINT_C:
        TargetZoomPos = StoredZoomPosC;
        break;
      case SETPOINT_D:
        TargetZoomPos = StoredZoomPosD;
        break;
    }

    // Determine zoom direction
    iStepperZoomMove = (iStepperZoomPos < TargetZoomPos) ? ZOOM_IN : ZOOM_OUT;

    // Stop moving when target position is reached
    if (iStepperZoomPos == TargetZoomPos) {
      iStepperZoomMove = ZOOM_STOP;
      SetpointStarted =  0;
      BlockUserInput =  0;
    }
  }
}

void handle_data_input() {
  if (sfReader.read()) {
    if (sfReader == "info") {
      Serial.println("zoom_module");
    }
    // Zoom control
    else if (sfReader == "4") {
      iStepperZoomMove = ZOOM_OUT;
    }
    else if (sfReader == "5") {
      iStepperZoomMove = ZOOM_IN;
    }
    else if (sfReader == "6") {
      iStepperZoomMove = ZOOM_STOP;
    }
    // Set/move to target
    else if (sfReader.startsWith("s")) {
      int setpoint = sfReader.substring(1).toInt();
      switch (setpoint) {
        case  1:
          StoredZoomPos = iStepperZoomPos;
          break;
        case  2:
          StoredZoomPosB = iStepperZoomPos;
          break;
        case  3:
          StoredZoomPosC = iStepperZoomPos;
          break;
        case  4:
          StoredZoomPosD = iStepperZoomPos;
          break;
      }
    }
    else if (sfReader.startsWith("t")) {
      SetpointStarted = sfReader.substring(1).toInt();
    }
    // Speed control
    else if (sfReader.startsWith("z")) {
      iStepperZoomSpeed = sfReader.substring(1).toInt();
    }
    // Reset zoom position
    else if (sfReader == "ea") {
      iStepperZoomPos =  0;
      StoredZoomBStop =  1490;
    }
    // Update B stop position
    else if (sfReader == "eb") {
      StoredZoomBStop = iStepperZoomPos +  1;
    }
    // Flush input
    else if (sfReader == "flush") {
      sfReader.flushInput();
    }
  }
}

void loop() {
  handle_data_input();
  handle_stepper_control();
}