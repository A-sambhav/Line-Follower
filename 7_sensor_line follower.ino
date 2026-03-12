// -------- Motor Driver --------
#define AIN1 11
#define AIN2 12
#define PWMA 5

#define BIN1 9
#define BIN2 8
#define PWMB 6

#define STBY 10

// -------- Sensors --------
#define S6 A6   // leftmost (analog only)
#define S0 A0
#define S1 A1
#define S2 A2
#define S3 A3
#define S4 A4
#define S5 A5   // rightmost

#define START_BUTTON 4

// -------- PID --------
float Kp = 105;
float Kd = 35;

int baseSpeed = 40;
int searchSpeed = 50;

int lastError = 0;

bool running = false;

int threshold = 500;   // threshold for A6 analog sensor

void setup()
{
  pinMode(S0, INPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(START_BUTTON, INPUT_PULLUP);
}

void loop()
{
  if (digitalRead(START_BUTTON) == LOW)
  {
    running = !running;
    delay(300);
  }

  if (!running) return;

  // -------- Sensor Read --------
  int s6 = analogRead(S6) < threshold;   // FIXED for A6
  int s0 = !digitalRead(S0);
  int s1 = !digitalRead(S1);
  int s2 = !digitalRead(S2);
  int s3 = !digitalRead(S3);
  int s4 = !digitalRead(S4);
  int s5 = !digitalRead(S5);

  int sensorSum = s6 + s0 + s1 + s2 + s3 + s4 + s5;

  // -------- LINE LOST --------
  if(sensorSum == 0)
  {
    motor(-searchSpeed, searchSpeed);   // spin left
    return;
  }

  // -------- PID --------
  int error =
      (-3 * s6) +
      (-2 * s0) +
      (-1 * s1) +
      ( 0 * s2) +
      ( 1 * s3) +
      ( 2 * s4) +
      ( 3 * s5);

  int P = error;
  int D = error - lastError;

  int correction = Kp * P + Kd * D;

  lastError = error;

  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  motor(leftSpeed, rightSpeed);
}

// -------- MOTOR CONTROL --------
void motor(int left, int right)
{
  if(left >= 0){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, left);
  }
  else{
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -left);
  }

  if(right >= 0){
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, right);
  }
  else{
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -right);
  }
}
