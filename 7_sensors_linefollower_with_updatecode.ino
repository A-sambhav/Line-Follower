// -------- Motor Driver --------
#define AIN1 11
#define AIN2 12
#define PWMA 5

#define BIN1 9
#define BIN2 8
#define PWMB 6

#define STBY 10

// -------- Sensors --------
#define S6 A6
#define S0 A0
#define S1 A1
#define S2 A2
#define S3 A3
#define S4 A4
#define S5 A5

#define START_BUTTON 4

float Kp = 0.12;
float Kd = 0.8;

int baseSpeed = 60;
int searchSpeed = 80;

int lastError = 0;
bool running = false;

int threshold = 500;

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

  int s6 = analogRead(S6) < threshold;
  int s0 = !digitalRead(S0);
  int s1 = !digitalRead(S1);
  int s2 = !digitalRead(S2);
  int s3 = !digitalRead(S3);
  int s4 = !digitalRead(S4);
  int s5 = !digitalRead(S5);

  int sensors[7] = {s6,s0,s1,s2,s3,s4,s5};
  int weights[7] = {0,1000,2000,3000,4000,5000,6000};

  int sum = 0;
  long weightedSum = 0;

  for(int i=0;i<7;i++)
  {
    sum += sensors[i];
    weightedSum += (long)sensors[i] * weights[i];
  }

  // -------- LINE LOST --------
  if(sum == 0)
  {
    motor(-searchSpeed, searchSpeed);
    return;
  }

  int position = weightedSum / sum;

  int error = position - 3000;

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
