#include "SafeStringReader.h"

const int StepX = 2;
const int DirX = 5;
const int StepY = 3;
const int DirY = 6;

const int EndstopX = 9;
const int EndstopY = 10;

const int EndstopDefaultPos = 0;

createSafeStringReader(sfReader, 16, " "); // a reader for upto 20 chars to read tokens terminated by space or timeout

// millisDelay pitchDelay;
// millisDelay yawDelay;
// millisDelay zoomDelay;

// HOW TO SWITCH TO TIMERS??
//  Have a function dedicated to just pulses and the timers are constant for high/low
//  Looped logic just decides if we need to move and what direction



int BlockUserInput = 0;
int SetpointStarted = 0;
int StoredPitchSpeed = 2000 * 1.5;
int StoredYawSpeed = 2000 * 1;
int TargetPitchPos = 0;
int TargetYawPos = 0;
int StoredPitchPos = 0;
int StoredYawPos = 0;
int StoredPitchPosB = 0;
int StoredYawPosB = 0;
int StoredPitchPosC = 0;
int StoredYawPosC = 0;
int StoredPitchPosD = 0;
int StoredYawPosD = 0;

void setup()
{
  pinMode(StepX, OUTPUT);
  pinMode(DirX, OUTPUT);
  pinMode(StepY, OUTPUT);
  pinMode(DirY, OUTPUT);

  // pinMode(EndstopX, INPUT_PULLUP);
  // pinMode(EndstopY, INPUT_PULLUP);

  Serial.begin(115200); // begin transmission
  SafeString::setOutput(Serial);
  sfReader.setTimeout(1000); // set 1 sec timeout
  sfReader.flushInput(); // empty Serial RX buffer and then skip until either find delimiter or timeout
  sfReader.connect(Serial); // read from Serial
}

int iStepperSpeedRamp = 0;

int iStepperPitchSpeed = 2000;
int iStepperPitchMove = 0;
int iStepperPitchPos = EndstopDefaultPos; // 10000 is default zero pos
void handle_pitch_stepper() {
  if (BlockUserInput > 0) {
    return;
  }

  if (iStepperPitchMove == 1)
  {
    digitalWrite(DirX, HIGH); // set direction, HIGH for clockwise, LOW for anticlockwise
  }
  else if (iStepperPitchMove == 2)
  {
    digitalWrite(DirX, LOW); // set direction, HIGH for clockwise, LOW for anticlockwise
  }

  if (iStepperPitchMove != 0) {
    digitalWrite(StepX, HIGH);
    delayMicroseconds(iStepperPitchSpeed / 2);
    digitalWrite(StepX, LOW);
    delayMicroseconds(iStepperPitchSpeed / 2);
    if (iStepperPitchMove == 1) {
      iStepperPitchPos += 1;
    } else if (iStepperPitchMove == 2) {
      iStepperPitchPos -= 1;
    }
  }
}

int iStepperYawSpeed = 1800 * 4;
int iStepperYawMove = 0;
int iStepperYawPos = EndstopDefaultPos;
void handle_yaw_stepper() {
  if (BlockUserInput > 0) {
    return;
  }

  if (iStepperYawMove == 1)
  {
    digitalWrite(DirY, HIGH); // set direction, HIGH for clockwise, LOW for anticlockwise
  }
  else if (iStepperYawMove == 2)
  {
    digitalWrite(DirY, LOW); // set direction, HIGH for clockwise, LOW for anticlockwise
  }

  if (iStepperYawMove != 0) {
    digitalWrite(StepY, HIGH);
    delayMicroseconds(iStepperYawSpeed);
    digitalWrite(StepY, LOW);
    delayMicroseconds(iStepperYawSpeed);
    if (iStepperYawMove == 1) {
      iStepperYawPos += 1;
    } else if (iStepperYawMove == 2) {
      iStepperYawPos -= 1;
    }
  }

  
}

void handle_setpoint_motion() 
{
  if (SetpointStarted > 0) {
    BlockUserInput = 1;

    if (SetpointStarted == 1) {
      TargetPitchPos = StoredPitchPos;
      TargetYawPos = StoredYawPos;
    } else if (SetpointStarted == 2) {
      TargetPitchPos = StoredPitchPosB;
      TargetYawPos = StoredYawPosB;
    } else if (SetpointStarted == 3) {
      TargetPitchPos = StoredPitchPosC;
      TargetYawPos = StoredYawPosC;
    } else if (SetpointStarted == 4) {
      TargetPitchPos = StoredPitchPosD;
      TargetYawPos = StoredYawPosD;
    }
    
    if (iStepperPitchPos == TargetPitchPos) {
      iStepperPitchMove = 0;
      iStepperPitchSpeed = StoredPitchSpeed;
    }

    if (iStepperYawPos == TargetYawPos) {
      iStepperYawMove = 0;
      iStepperYawSpeed = StoredYawSpeed;
    }

    if ((iStepperPitchPos == TargetPitchPos) && (iStepperYawPos == TargetYawPos)) {
      SetpointStarted = 0;
      BlockUserInput = 0;
      iStepperPitchMove = 0;
      iStepperYawMove = 0;
      iStepperPitchSpeed = StoredPitchSpeed;
      iStepperYawSpeed = StoredYawSpeed;
    }
    
    // iStepperPitchSpeed = iStepperPitchSpeed / 2;

    if (iStepperPitchPos < TargetPitchPos) {
      iStepperPitchMove = 1;
    } else if (iStepperPitchPos > TargetPitchPos) {
      iStepperPitchMove = 2;
    }

    // iStepperYawSpeed = iStepperYawSpeed / 2;

    if (iStepperYawPos < TargetYawPos) {
      iStepperYawMove = 1;
    } else if (iStepperYawPos > TargetYawPos) {
      iStepperYawMove = 2;
    }

    BlockUserInput = 0; // Allow input while moving for now
  }
}

int ZeroStepperStarted = 0;

// void handle_zero_steppers()
// {
//   if (ZeroStepperStarted) {
//     int limitX = digitalRead(EndstopX);
//     int limitY = digitalRead(EndstopY);

//     if (limitX == LOW) { // while endstop is inactive
//       if (iStepperPitchPos < (EndstopDefaultPos + 1000)) {
//         iStepperPitchMove = 2;
//       } else {
//         iStepperPitchMove = 1;
//       }
//     }

//     if (limitY == LOW) { // while endstop is inactive
//       if (iStepperYawPos < (EndstopDefaultPos + 1000)) {
//         iStepperYawMove = 2;
//       } else {
//         iStepperYawMove = 2;
//       }
//     }

//     if (limitX == HIGH && limitY == HIGH) {
//       ZeroStepperStarted = 0;
//       iStepperYawMove = 0;
//       iStepperPitchMove = 0;
//       iStepperPitchPos = EndstopDefaultPos;
//       iStepperYawPos = EndstopDefaultPos;
//     }
//   }
// }

void handle_stepper_control()
{
  // handle_zero_steppers();
  handle_yaw_stepper();
  handle_pitch_stepper();
  handle_setpoint_motion();
}

void handle_data_input()
{
  // Serial.print(sfReader);

  // Pitch control
  if (sfReader == "a") {
    iStepperPitchMove = 1;
  }
  else if (sfReader == "b") {
    iStepperPitchMove = 2;
  }
  else if (sfReader == "c") {
    iStepperPitchMove = 0;
  }

  // Yaw control
  if (sfReader == "1") {
    iStepperYawMove = 1;
  }
  else if (sfReader == "2") {
    iStepperYawMove = 2;
  }
  else if (sfReader == "3") {
    iStepperYawMove = 0;
  }

  // Set/move to target
  if (sfReader == "s") {
    StoredPitchSpeed = iStepperPitchSpeed;
    StoredYawSpeed = iStepperYawSpeed;
    StoredPitchPos = iStepperPitchPos;
    StoredYawPos = iStepperYawPos;
  } else if (sfReader == "s2") {
    StoredPitchSpeed = iStepperPitchSpeed;
    StoredYawSpeed = iStepperYawSpeed;
    StoredPitchPosB = iStepperPitchPos;
    StoredYawPosB = iStepperYawPos;
  } else if (sfReader == "s3") {
    StoredPitchSpeed = iStepperPitchSpeed;
    StoredYawSpeed = iStepperYawSpeed;
    StoredPitchPosC = iStepperPitchPos;
    StoredYawPosC = iStepperYawPos;
  } else if (sfReader == "s4") {
    StoredPitchSpeed = iStepperPitchSpeed;
    StoredYawSpeed = iStepperYawSpeed;
    StoredPitchPosD = iStepperPitchPos;
    StoredYawPosD = iStepperYawPos;
  } else if (sfReader == "t") {
    SetpointStarted = 1;
    iStepperPitchSpeed = 2000 * 1.5;
    iStepperYawSpeed = 2000 * 1;
  } else if (sfReader == "t2") {
    SetpointStarted = 2;
    iStepperPitchSpeed = 2000 * 1.5;
    iStepperYawSpeed = 2000 * 1;
  } else if (sfReader == "t3") {
    SetpointStarted = 3;
    iStepperPitchSpeed = 2000 * 1.5;
    iStepperYawSpeed = 2000 * 1;
  } else if (sfReader == "t4") {
    SetpointStarted = 4;
    iStepperPitchSpeed = 2000 * 1.5;
    iStepperYawSpeed = 2000 * 1;
  }

  // Speed control
  if (sfReader.startsWith("p")) {
    sfReader.removeBefore(1);
    sfReader.toInt(iStepperPitchSpeed);
  } 
  else if (sfReader.startsWith("y")) {
    sfReader.removeBefore(1);
    sfReader.toInt(iStepperYawSpeed);
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
