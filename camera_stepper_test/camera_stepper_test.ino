const int StepX = 2;
const int DirX = 5;
const int StepY = 3;
const int DirY = 6;

void setup()
{
  pinMode(StepX, OUTPUT);
  pinMode(DirX, OUTPUT);
  pinMode(StepY, OUTPUT);
  pinMode(DirY, OUTPUT);
}

void handle_pitch_stepper() {
    int i = 0;
    while (i < 10)
    {
      digitalWrite(DirX, HIGH); // set direction, HIGH for clockwise, LOW for anticlockwise

      digitalWrite(StepX, HIGH);
      delayMicroseconds(2000 * 5);
      digitalWrite(StepX, LOW);
      delayMicroseconds(2000 * 5);
      i = i + 1;
    }
    
    i = 0;
    while (i < 10)
    {
      digitalWrite(DirX, LOW); // set direction, HIGH for clockwise, LOW for anticlockwise

      digitalWrite(StepX, HIGH);
      delayMicroseconds(2000 * 5);
      digitalWrite(StepX, LOW);
      delayMicroseconds(2000 * 5);
      i = i + 1;
    }
}

void handle_yaw_stepper() {
  int i = 0;
  while (i < 10)
  {
    digitalWrite(DirY, HIGH); // set direction, HIGH for clockwise, LOW for anticlockwise

    digitalWrite(StepY, HIGH);
    delayMicroseconds(2000 * 5);
    digitalWrite(StepY, LOW);
    delayMicroseconds(2000 * 5);
    i = i + 1;
  }
  
  i = 0;
  while (i < 10)
  {
    digitalWrite(DirY, LOW); // set direction, HIGH for clockwise, LOW for anticlockwise

    digitalWrite(StepY, HIGH);
    delayMicroseconds(2000 * 5);
    digitalWrite(StepY, LOW);
    delayMicroseconds(2000 * 5);
    i = i + 1;
  }
}

void handle_stepper_control()
{
  handle_yaw_stepper();
  delay(2000);
  handle_pitch_stepper();
  delay(2000);
}

void loop()
{
  handle_stepper_control();
}
