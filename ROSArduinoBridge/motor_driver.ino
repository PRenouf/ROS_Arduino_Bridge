

#if defined(PR_H1_01)
void initMotorControllers() {
  pinMode(M1_ENA, OUTPUT);
  pinMode(M1_ENB, OUTPUT);
  pinMode(M2_ENA, OUTPUT);
  pinMode(M2_ENB, OUTPUT);
  pinMode(M3_ENA, OUTPUT);
  pinMode(M3_ENB, OUTPUT);
}

void enableMotorControllers() {  // Enable all H-Bridges
  enableMotorController(MOTOR1);
  enableMotorController(MOTOR2);
  enableMotorController(MOTOR3);
}

void enableMotorController(int i) {  // Enable Each H-Bridge
  if (i == MOTOR1) {
    digitalWrite(M1_ENA, HIGH);
    digitalWrite(M1_ENB, HIGH);
  } else if (i == MOTOR2) {
    digitalWrite(M2_ENA, HIGH);
    digitalWrite(M2_ENB, HIGH);
  } else if (i == MOTOR3) {
    digitalWrite(M3_ENA, HIGH);
    digitalWrite(M3_ENB, HIGH);
  }
}


void disableMotorControllers() {  // Disable all H-Bridges
  enableMotorController(MOTOR1);
  enableMotorController(MOTOR2);
  enableMotorController(MOTOR3);
}

void disableMotorController(int i) {  // Disable Each H-Bridge
  if (i == MOTOR1) {
    digitalWrite(M1_ENA, LOW);
    digitalWrite(M1_ENB, LOW);
  } else if (i == MOTOR2) {
    digitalWrite(M2_ENA, LOW);
    digitalWrite(M2_ENB, LOW);
  } else if (i == MOTOR3) {
    digitalWrite(M3_ENA, LOW);
    digitalWrite(M3_ENB, LOW);
  }
}



void testMotors() {
  setMotorSpeed(MOTOR1, TEST_SPEED);
  delay(TEST_MOTOR_DURATION);
  setMotorSpeed(MOTOR1, 0);
  delay(TEST_MOTOR_DURATION);
  setMotorSpeed(MOTOR1, -1 * TEST_SPEED);
  delay(TEST_MOTOR_DURATION);
  setMotorSpeed(MOTOR1, 0);
  delay(TEST_MOTOR_DURATION);

  setMotorSpeed(MOTOR2, TEST_SPEED);
  delay(TEST_MOTOR_DURATION);
  setMotorSpeed(MOTOR2, 0);
  delay(TEST_MOTOR_DURATION);
  setMotorSpeed(MOTOR2, -1 * TEST_SPEED);
  delay(TEST_MOTOR_DURATION);
  setMotorSpeed(MOTOR2, 0);
  delay(TEST_MOTOR_DURATION);

  setMotorSpeed(MOTOR3, TEST_SPEED);
  delay(TEST_MOTOR_DURATION);
  setMotorSpeed(MOTOR3, 0);
  delay(TEST_MOTOR_DURATION);
  setMotorSpeed(MOTOR3, -1 * TEST_SPEED);
  delay(TEST_MOTOR_DURATION);
  setMotorSpeed(MOTOR3, 0);
  delay(TEST_MOTOR_DURATION);
}


void setMotorSpeed(int i, int spd) {
  unsigned char reverse = 0;

  if (spd < 0) {  // Set Reverse Flag is Applicable
    spd = -spd;
    reverse = 1;
  }

  if (spd > 255)  // Speed Limit is 255
    spd = 255;

  if (i == MOTOR1) {  // Set Motor 1 Direction and Speed
    if (reverse == 0) {
      digitalWrite(M1_INA, LOW);
      digitalWrite(M1_INB, HIGH);
    } else if (reverse == 1) {
      digitalWrite(M1_INB, LOW);
      digitalWrite(M1_INA, HIGH);
    }
    analogWrite(M1_PWM, spd);
  } else if (i == MOTOR2) {  // Set Motor 2 Direction and Speed
    if (reverse == 0) {
      digitalWrite(M2_INA, LOW);
      digitalWrite(M2_INB, HIGH);
    } else if (reverse == 1) {
      digitalWrite(M2_INB, LOW);
      digitalWrite(M2_INA, HIGH);
    }
    analogWrite(M2_PWM, spd);
  } else if (i == MOTOR3) {  // Set Motor 3 Direction and Speed
    if (reverse == 0) {
      digitalWrite(M3_INA, LOW);
      digitalWrite(M3_INB, HIGH);
    } else if (reverse == 1) {
      digitalWrite(M3_INB, LOW);
      digitalWrite(M3_INA, HIGH);
    }
    analogWrite(M3_PWM, spd);
  }
}

void setMotorSpeeds(int motor1Speed, int motor2Speed, int motor3Speed) {
  setMotorSpeed(MOTOR1, motor1Speed);
  setMotorSpeed(MOTOR2, motor2Speed);
  setMotorSpeed(MOTOR3, motor3Speed);
}
#else
#error A motor driver must be selected!
#endif
