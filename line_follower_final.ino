#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//--------Pin definitions for the TB6612FNG Motor Driver----
#define AIN1 11
#define AIN2 12
#define PWMA 5
#define BIN1 9
#define BIN2 8
#define PWMB 6
#define STBY 10
//------------------------------------------------------------

//--------Enter Line Details here---------
bool isBlackLine = 0;             // 1 for black line, 0 for white line
unsigned int lineThickness = 30;  // Line thickness in mm (10–35)
unsigned int numSensors = 5;      // Number of sensors (5 or 7)
//-----------------------------------------

int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int lfSpeed = 150;                // Max speed for straight lines
int minSpeed = 50;                // Min speed for turns
int currentSpeed = 50;            // Starting speed (slow)
int sensorWeight[5] = {2, 1, 0, -1, -2};
int activeSensors;
float Kp = 0.1;
float Kd = 0.05;
float Ki = 0.0;

int onLine = 1;
int minValues[7], maxValues[7], threshold[7], sensorValue[7], sensorArray[7];

void setup() {
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(9600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(3, INPUT_PULLUP);  // Pushbutton
  pinMode(4, INPUT_PULLUP);  // Pushbutton
  pinMode(13, OUTPUT);        // LED

  pinMode(5, OUTPUT);     
  digitalWrite(5, HIGH);  // Enables motor driver

  lineThickness = constrain(lineThickness, 10, 35);
  if (numSensors == 5) {
    sensorWeight[1] = 4;
    sensorWeight[5] = -4;
  }
}

void loop() {
  while (digitalRead(3)) {}
  delay(1000);
  calibrate();
  while (digitalRead(4)) {}
  delay(1000);

  while (1) {
    readLine();
    if (onLine == 1) {  // PID LINE FOLLOW
      linefollow();
      adjustSpeed();  // New function to adjust speed based on error
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
      currentSpeed = minSpeed;  // Slow down when off the line
      if (error > 0) {
        motor1run(-50);
        motor2run(lfSpeed);
      } else {
        motor1run(lfSpeed);
        motor2run(-50);
      }
    }
  }
}

void linefollow() {
  error = 0;
  activeSensors = 0;

  if (numSensors == 7) {
    for (int i = 0; i < 7; i++) {
      error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
      activeSensors += sensorArray[i];
    }
    error = error / activeSensors;
  }
  if (numSensors == 5) {
    for (int i = 1; i < 6; i++) {
      error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
      activeSensors += sensorArray[i];
    }
    error = error / activeSensors;
  }

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  if (lsp > 255) lsp = 255;
  if (lsp < 0) lsp = 0;
  if (rsp > 255) rsp = 255;
  if (rsp < 0) rsp = 0;

  motor1run(lsp);
  motor2run(rsp);
}

// New function to adjust speed based on error
void adjustSpeed() {
  int absError = abs(error);  // Absolute value of error
  if (absError < 500) {       // Small error = straight line
    if (currentSpeed < lfSpeed) currentSpeed += 1;  // Speed up gradually
  } else {                    // Large error = turn
    if (currentSpeed > minSpeed) currentSpeed -= 30; // Slow down gradually
  }
  currentSpeed = constrain(currentSpeed, minSpeed, lfSpeed); // Keep within bounds
}

void calibrate() {
  for (int i = 0; i < 7; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  for (int i = 0; i < 20000; i++) {
    motor1run(50);
    motor2run(-50);
    for (int i = 0; i < 7; i++) {
      if (analogRead(i) < minValues[i]) minValues[i] = analogRead(i);
      if (analogRead(i) > maxValues[i]) maxValues[i] = analogRead(i);
    }
  }

  for (int i = 0; i < 7; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  motor1run(0);
  motor2run(0);
}

void readLine() {
  onLine = 0;
  if (numSensors == 7) {
    for (int i = 0; i < 7; i++) {
      if (isBlackLine) {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
      } else {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 1000, 0);
      }
      sensorValue[i] = constrain(sensorValue[i], 0, 1000);
      sensorArray[i] = sensorValue[i] > 500;
      if (sensorArray[i]) onLine = 1;
    }
  }
  if (numSensors == 5) {
    for (int i = 1; i < 6; i++) {
      if (isBlackLine) {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
      } else {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 1000, 0);
      }
      sensorValue[i] = constrain(sensorValue[i], 0, 1000);
      sensorArray[i] = sensorValue[i] > 500;
      if (sensorArray[i]) onLine = 1;
    }
  }
}

void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, 0);
  }
}

void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, 0);
  }
}
