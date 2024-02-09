// #include <FastAccelStepper.h>

// FastAccelStepperEngine engine = FastAccelStepperEngine();
// FastAccelStepper *stepperPitch = NULL;
// FastAccelStepper *stepperYaw = NULL;

String data = "";

const int StepX = 2;
const int DirX = 5;
const int StepY = 3;
const int DirY = 6;
const int StepZ = 4;
const int DirZ = 7;

void setup()
{
  pinMode(StepX, OUTPUT);
  pinMode(DirX, OUTPUT);
  pinMode(StepY, OUTPUT);
  pinMode(DirY, OUTPUT);
  pinMode(StepZ, OUTPUT);
  pinMode(DirZ, OUTPUT);

  // engine.init();
  // stepperPitch = engine.stepperConnectToPin(2);
  // if (stepperPitch)
  // {
  //   stepperPitch->setDirectionPin(5);
  //   stepperPitch->setSpeedInHz(500);    // 500 steps/s
  //   stepperPitch->setAcceleration(100); // 100 steps/s²
  //   // stepperPitch->move(1000);
  // }

  // stepperYaw = engine.stepperConnectToPin(3);
  // if (stepperYaw)
  // {
  //   stepperYaw->setDirectionPin(6);
  //   stepperYaw->setSpeedInHz(500);    // 500 steps/s
  //   stepperYaw->setAcceleration(100); // 100 steps/s²
  //   // stepperYaw->move(1000);
  // }

  // digitalWrite(5, HIGH);

  Serial.begin(115200); // begin transmission
}

int iStepperSpeedRamp = 0;

int iStepperPitchSpeed = 9000;
int iStepperPitchMove = 0;
void handle_pitch_stepper() {
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
    delayMicroseconds(iStepperPitchSpeed);
    digitalWrite(StepX, LOW);
    delayMicroseconds(iStepperPitchSpeed);
  }
}

int iStepperYawSpeed = 1800 * 4;
int iStepperYawMove = 0;
void handle_yaw_stepper() {
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
  }

  
}

int iStepperZoomSpeed = 2000;
int iStepperZoomMove = 0;
int iStepperZoomPos = 0;
void handle_zoom_stepper() {
  if (iStepperZoomMove == 1)
  {
    digitalWrite(DirZ, HIGH); // set direction, HIGH for clockwise, LOW for anticlockwise
  }
  else if (iStepperZoomMove == 2)
  {
    digitalWrite(DirZ, LOW); // set direction, HIGH for clockwise, LOW for anticlockwise
  }

  if (iStepperZoomMove != 0) {
    bool allowedToMove = true;
    if (iStepperZoomMove == 1 && iStepperZoomPos > 220) {
      allowedToMove = false;
    } else if (iStepperZoomMove == 2 && iStepperZoomPos < -150) {
      allowedToMove = false;
    }

    // Serial.print(allowedToMove);
    // Serial.print("\n");

    if (allowedToMove) {
      int zoomSpeed = iStepperZoomSpeed;
      if (abs(iStepperZoomPos) < 160) {
        zoomSpeed = 50;
      }

      digitalWrite(StepZ, HIGH);
      delayMicroseconds(zoomSpeed);
      digitalWrite(StepZ, LOW);
      delayMicroseconds(zoomSpeed);
      if (iStepperZoomMove == 1) {
        iStepperZoomPos += 1;
      } else {
        iStepperZoomPos -= 1;
      }
    }
  } else if (iStepperZoomPos > 1 || iStepperZoomPos < -1) {
    if (iStepperZoomPos < 0) {
      digitalWrite(DirZ, HIGH);
      
      digitalWrite(StepZ, HIGH);
      delayMicroseconds(iStepperZoomSpeed);
      digitalWrite(StepZ, LOW);
      delayMicroseconds(iStepperZoomSpeed);
      iStepperZoomPos += 1;
    } else {
      digitalWrite(DirZ, LOW);
      
      digitalWrite(StepZ, HIGH);
      delayMicroseconds(iStepperZoomSpeed);
      digitalWrite(StepZ, LOW);
      delayMicroseconds(iStepperZoomSpeed);
      iStepperZoomPos -= 1;
    }
  }
}

void handle_stepper_control()
{
  handle_yaw_stepper();
  handle_pitch_stepper();
  handle_zoom_stepper();
}

void serialFlush(void){
    while(Serial.available() > 0){
        char c = Serial.read();
    }
} 

char buf[120];
int readline(int readch, char *buffer, int len) {
    static int pos = 0;
    int rpos;

    if (readch > 0) {
        switch(readch) {
            case '\r': // Ignore CR
                break;
            case '\n': // Return on new-line
                rpos = pos;
                pos = 0; // Reset position index ready for next time
                return rpos;
            default:
                if (pos < len-1) {
                    buffer[pos++] = readch;
                    buffer[pos] = 0;
                }
        }
    }
    return 0;
}

void handleSerialData() {
  if (readline(Serial.read(), buf, 120) > 0) {
    // if (strcmp("n", buf) == 0) {
    if (strncmp("j", buf, 1) == 0) {
        // Serial.print("j=");

        const char delim[2] = ",";
        char *token;
        token = strtok(buf, delim);

        int idx = 0;
        while( token != NULL ) {
          switch(idx) {
              case 1:
                iStepperPitchMove = atoi(token);
                  break;
              case 2:
                iStepperYawMove = atoi(token);
                  break;
              case 3:
                iStepperZoomMove = atoi(token);
                  break;
              case 4:
                iStepperPitchSpeed = atoi(token);
                  break;
              case 5:
                iStepperYawSpeed = atoi(token);
                break;
              case 6: 
                iStepperZoomSpeed = atoi(token);
                break;
          }

          // Serial.print(token);
          // Serial.print("|");
          token = strtok(NULL, delim);
          idx++;
        }

        // Serial.print("||");

        // Serial.print(iStepperPitchMove);
        // Serial.print("|");
        // Serial.print(iStepperYawMove);
        // Serial.print("|");
        // Serial.print(iStepperZoomMove);
        // Serial.print("|");
        // Serial.print(iStepperPitchSpeed);
        // Serial.print("|");
        // Serial.print(iStepperYawSpeed);
        // Serial.print("|");
        // Serial.print(iStepperZoomSpeed);

        // Serial.print("\n");
    }
  }
}

void loop()
{
  // handleSerialData();

  String val;
  while (Serial.available() > 0)
  {
    // val = val + (char)Serial.read(); // read data byte by byte and store it
    char incByte = Serial.read();

    if (incByte == '0')
    {
      data = "";
      serialFlush();
    }
    else if (incByte == 'a')
    {
      iStepperPitchMove = 1;
      data = "";
    }
    else if (incByte == 'b')
    {
      iStepperPitchMove = 2;
      data = "";
    }
    else if (incByte == 'c')
    {
      iStepperPitchMove = 0;
      data = "";
    }
    else if (incByte == '1')
    {
      iStepperYawMove = 1;
      data = "";
    }
    else if (incByte == '2')
    {
      // while (Serial.available() > 0 && incByte != ';')
      // {
      //   incByte = Serial.read();
      //   if (incByte != ';') {
      //     data += incByte;
      //   }
      // }
      // iStepperYawSpeed = data.toInt();
      iStepperYawMove = 2;
      data = "";
    }
    else if (incByte == '3')
    {
      iStepperYawMove = 0;
      data = "";
    }
    else if (incByte == '4')
    {
      iStepperZoomMove = 1;
      data = "";
    }
    else if (incByte == '5')
    {
      iStepperZoomMove = 2;
      data = "";
    }
    else if (incByte == '6')
    {
      iStepperZoomMove = 0;
      data = "";
    }
    else if (incByte == 'p')
    {
      delay(2); // wait to make sure all the serial data has arrived
      while (Serial.available() > 0)
      {
        incByte = Serial.read();
        data += incByte;
      }
      // iStepperPitchSpeed = data.toInt();
      data = "";
    }
    else if (incByte == 'y')
    {
      delay(2); // wait to make sure all the serial data has arrived
      while (Serial.available() > 0)
      {
        incByte = Serial.read();
        data += incByte;
      }
      iStepperYawSpeed = data.toInt();
      data = "";
    }
    else if (incByte == 'z')
    {
      delay(2); // wait to make sure all the serial data has arrived
      while (Serial.available() > 0)
      {
        incByte = Serial.read();
        data += incByte;
      }
      iStepperZoomSpeed = data.toInt();
      data = "";
    }
    else
    {
      data += incByte;
    }
  }

  handle_stepper_control();

  // digitalWrite(2,HIGH);
  // delayMicroseconds(300);
  // digitalWrite(2,LOW);
  // delayMicroseconds(300);

  // if (val.startsWith("y1")) {
  //   iStepperYawMove = 1;
  //   bStepperYawDirty = true;
  // } else if (val.startsWith("y-1")) {
  //   iStepperYawMove = -1;
  //   bStepperYawDirty = true;
  // } else if (val.startsWith("y0")) {
  //   iStepperYawMove = 0;
  //   bStepperYawDirty = true;
  // }

  // TODO: figure out protocol
  //    rpc call name, args??
  // TODO: add motor control loop
  // Serial.print(val); // send the received data back to raspberry pi
}
