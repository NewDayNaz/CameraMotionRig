#include <AccelStepper.h>
// Look into AccelStepper

const int StepX = 2;
const int DirX = 5;
const int StepY = 3;
const int DirY = 6;
const int StepZ = 4;
const int DirZ = 7;

AccelStepper stepperPitch(AccelStepper::FULL2WIRE, StepX, DirX);
AccelStepper stepperYaw(AccelStepper::FULL2WIRE, StepY, DirY);
AccelStepper stepperZoom(AccelStepper::FULL2WIRE, StepZ, DirZ);


double xVal = 0;
double yVal = 0;
double zVal = 0;

double lbVal = 0;
double rbVal = 0;

void setup()
{
    Serial.begin(115200);

    stepperPitch.setMaxSpeed(200);
    stepperPitch.setAcceleration(3000);
    stepperYaw.setMaxSpeed(200);
    stepperYaw.setAcceleration(3000);
    stepperZoom.setMaxSpeed(600);
    stepperZoom.setAcceleration(30000);
}

bool bHasNullPitch = false;
void handle_pitch_stepper() {

    if (yVal > 0)
    {
        bHasNullPitch = false;
        stepperPitch.setAcceleration(4000);
        stepperPitch.moveTo(stepperPitch.currentPosition() + (200 * yVal));
    }
    else if (yVal < 0)
    {
        bHasNullPitch = false;
        stepperPitch.setAcceleration(4000);
        stepperPitch.moveTo(stepperPitch.currentPosition() - (200 * abs(yVal)));
    } else {
        stepperPitch.setAcceleration(1000);
        if (!bHasNullPitch) {
            bHasNullPitch = true;
            stepperPitch.moveTo(stepperPitch.currentPosition());
        }
    }

    stepperPitch.run();
}

bool bHasNullYaw = false;
void handle_yaw_stepper() {

    if (xVal > 0)
    {
        bHasNullYaw = false;
        stepperYaw.setAcceleration(4000);
        stepperYaw.moveTo(stepperYaw.currentPosition() + (100 * xVal));
    }
    else if (xVal < 0)
    {
        bHasNullYaw = false;
        stepperYaw.setAcceleration(4000);
        stepperYaw.moveTo(stepperYaw.currentPosition() - (100 * abs(xVal)));
    } else {
        stepperYaw.setAcceleration(1000);
        if (!bHasNullYaw) {
            bHasNullYaw = true;
            stepperYaw.moveTo(stepperYaw.currentPosition()); 
        }
    }

    stepperYaw.run();
}

void handle_zoom_stepper() {

    if (zVal < 0)
    {
        // stepperZoom.setAcceleration(8000);
        stepperZoom.moveTo(1400);
    }
    else if (zVal > 0)
    {
        // stepperZoom.setAcceleration(8000);
        stepperZoom.moveTo(-1000);
    } else {
        // stepperZoom.setAcceleration(8000);
        stepperZoom.moveTo(0);
    }

    stepperZoom.run();
}

long savedPitchPos = 0;
long savedYawPos = 0;

void handle_bumpers() {
    if (rbVal > 0) {
        savedPitchPos = stepperPitch.currentPosition();
        savedYawPos = stepperYaw.currentPosition();
    }

    if (lbVal > 0) {

        bHasNullPitch = true;
        bHasNullYaw = true;
        stepperPitch.setAcceleration(1000);
        stepperYaw.setAcceleration(1000);
        stepperPitch.moveTo(savedPitchPos);
        stepperYaw.moveTo(savedYawPos);
    }
}

char buf[80];
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

void loop()
{
    if (readline(Serial.read(), buf, 80) > 0) {
        // if (strcmp("n", buf) == 0) {
            if (strncmp("j", buf, 1) == 0) {
                const char delim[2] = ",";
                char *token;
                token = strtok(buf, delim);

                int idx = 0;
                while( token != NULL ) {
                    switch(idx) {
                        case 1:
                            xVal = atof(token);
                            break;
                        case 2:
                            yVal = atof(token);
                            break;
                        case 3:
                            zVal = atof(token);
                            break;
                        case 4:
                            lbVal = atof(token);
                        case 5:
                            rbVal = atof(token);
                    }

                    // Serial.print(token);
                    // Serial.print("|");
                    token = strtok(NULL, delim);
                    idx++;
                }
            }
    }

    handle_pitch_stepper();
    handle_yaw_stepper();
    handle_zoom_stepper();
    // handle_bumpers();
}
