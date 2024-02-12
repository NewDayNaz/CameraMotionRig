#include "SafeStringReader.h"

const int StepZ = 4;
const int DirZ = 7;

createSafeStringReader(sfReader, 16, " "); // a reader for upto 20 chars to read tokens terminated by space or timeout

unsigned long StepTimer;

// HOW TO SWITCH TO TIMERS??
//  Have a function dedicated to just pulses and the timers are constant for high/low
//  Looped logic just decides if we need to move and what direction

int BlockUserInput = 0;
int SetpointStarted = 0;

int StoredZoomAStop = 0;
int StoredZoomBStop = 1490;

int StoredZoomSpeed = 2000 * 1;
int TargetZoomPos = 0;
int StoredZoomPos = 0;
int StoredZoomPosB = 0;
int StoredZoomPosC = 0;
int StoredZoomPosD = 0;

void setup()
{
  pinMode(StepZ, OUTPUT);
  pinMode(DirZ, OUTPUT);

  StepTimer = millis();

  zero_zoom_pos();

  Serial.begin(115200); // begin transmission
  SafeString::setOutput(Serial);
  sfReader.setTimeout(1000); // set 1 sec timeout
  sfReader.flushInput(); // empty Serial RX buffer and then skip until either find delimiter or timeout
  sfReader.connect(Serial); // read from Serial
}

bool can_we_step_zoom(int interval) {
  return ((millis() - StepTimer) >= interval)
}

void step_zoom_stepper() {
  if (digitalRead (StepZ) == HIGH) {
    digitalWrite(StepZ, LOW);
  } else {
    digitalWrite(StepZ, HIGH);
  }

  StepTimer = millis();
}

int iStepperSpeedRamp = 0;

int iStepperZoomSpeed = 2000 * 0.65;
int iStepperZoomMove = 0;
int iStepperZoomPos = 0;
void handle_zoom_stepper() {
  if (iStepperZoomMove == 1)
  {
    digitalWrite(DirZ, LOW); // set direction, HIGH for clockwise, LOW for anticlockwise
  }
  else if (iStepperZoomMove == 2)
  {
    digitalWrite(DirZ, HIGH); // set direction, HIGH for clockwise, LOW for anticlockwise
  }

  if (iStepperZoomMove != 0) {
    bool allowedToMove = true;
    if (iStepperZoomMove == 1 && iStepperZoomPos >= StoredZoomBStop) { // limit for B stop
     allowedToMove = false;
    } 
    
    if (iStepperZoomMove == 2 && iStepperZoomPos <= 0) { // zoomed out endstop is enforced
     allowedToMove = false; 
    }

    if (allowedToMove) {
      digitalWrite(StepZ, HIGH);
      delayMicroseconds(iStepperZoomSpeed);
      digitalWrite(StepZ, LOW);
      delayMicroseconds(iStepperZoomSpeed);
      if (iStepperZoomMove == 1) {
        iStepperZoomPos += 1;
      } else if (iStepperZoomMove == 2) {
        iStepperZoomPos -= 1;
      }
    }
  }
}

void zero_zoom_pos()
{
  digitalWrite(DirZ, HIGH); // zoom out

  iStepperZoomPos = 0;

  while (iStepperZoomPos < 1139) {
    if (can_we_step_zoom(2000)) {
      step_zoom_stepper();
      iStepperZoomPos += 1;
    }
  }
  
  iStepperZoomPos = 0;
}

void handle_setpoint_motion() 
{
  if (SetpointStarted > 0) {
    BlockUserInput = 1;

    if (SetpointStarted == 1) {
      TargetZoomPos = StoredZoomPos;
    } else if (SetpointStarted == 2) {
      TargetZoomPos = StoredZoomPosB;
    } else if (SetpointStarted == 3) {
      TargetZoomPos = StoredZoomPosC;
    } else if (SetpointStarted == 4) {
      TargetZoomPos = StoredZoomPosD;
    }

    if (iStepperZoomPos == TargetZoomPos) {
      iStepperZoomMove = 0;
      iStepperZoomSpeed = StoredZoomSpeed;
    }

    if (iStepperZoomPos == TargetZoomPos) {
      SetpointStarted = 0;
      BlockUserInput = 0;
      iStepperZoomMove = 0;
      iStepperZoomSpeed = StoredZoomSpeed;
    }

    // iStepperZoomSpeed = iStepperZoomSpeed / 2;

    if (iStepperZoomPos > TargetZoomPos) {
      iStepperZoomMove = 2;
    } else if (iStepperZoomPos < TargetZoomPos) {
      iStepperZoomMove = 1;
    }

    BlockUserInput = 0; // Allow input while moving for now
  }
}

void handle_stepper_control()
{
  handle_zoom_stepper();
  handle_setpoint_motion();
}

void handle_data_input()
{
  // Serial.print(sfReader);

  if (sfReader == "info") {
    Serial.println("zoom_module");
  }

  // Zoom control
  if (sfReader == "4") {
    iStepperZoomMove = 2;
  }
  else if (sfReader == "5") {
    iStepperZoomMove = 1;
  }
  else if (sfReader == "6") {
    iStepperZoomMove = 0;
  }

  // Set/move to target
  if (sfReader == "s") {
    StoredZoomPos = iStepperZoomPos;
  } else if (sfReader == "s2") {
    StoredZoomPosB = iStepperZoomPos;
  } else if (sfReader == "s3") {
    StoredZoomPosC = iStepperZoomPos;
  } else if (sfReader == "s4") {
    StoredZoomPosD = iStepperZoomPos;
  } else if (sfReader == "t") {
    SetpointStarted = 1;
  } else if (sfReader == "t2") {
    SetpointStarted = 2;
  } else if (sfReader == "t3") {
    SetpointStarted = 3;
  } else if (sfReader == "t4") {
    SetpointStarted = 4;
  } else if (sfReader == "ea") {
    iStepperZoomPos = 0;
    StoredZoomBStop = 1490;
  } else if (sfReader == "eb") {
    StoredZoomBStop = iStepperZoomPos + 1;
  }

  // Speed control
  if (sfReader.startsWith("z")) {
    sfReader.removeBefore(1);
    sfReader.toInt(iStepperZoomSpeed);
  }
}

void loop()
{
  if (sfReader.read()) { // got a line or timed out  delimiter is NOT returned
    if (sfReader.hasError()) { // input length exceeded
      // Serial.println(F(" sfReader hasError. Read '\\0' or input overflowed."));
    }
    if (sfReader.getDelimiter() == -1) { // no delimiter so timed out
      // Serial.println(F(" Input timed out without space"));
    }
    // Serial.print(F(" got a line of input '")); Serial.print(sfReader); Serial.println("'");
    handle_data_input();
    if (sfReader == "flush") {
      sfReader.flushInput();
    }
    // no need to clear sfReader as read() does that
  }

  handle_stepper_control();
}
