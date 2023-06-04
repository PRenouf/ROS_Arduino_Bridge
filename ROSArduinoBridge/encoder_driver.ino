/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */

#if defined(PR_H1_01)
volatile long m1_enc_pos = 0L;
volatile long m2_enc_pos = 0L;
volatile long m3_enc_pos = 0L;

static const int8_t ENC_STATES[] = { 0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0 };
/*  ENCODER LOOK-UP-TABLE:
          Index | Val |   Encoder
            0   |  0  |   00 <- 00  No Change
            1   |  1  |   00 <- 01  Forward
            2   | -1  |   00 <- 10  Backward
            3   |  0  |   00 <- 11  2 Bits Change, erroneous behaviour
            4   | -1  |   01 <- 00  Backward
            5   |  0  |   01 <- 01  No Change
            6   |  0  |   01 <- 10  2 Bits Change, erroneous behaviour
            7   |  1  |   01 <- 11  Forward
            8   |  1  |   10 <- 00  Forward
            9   |  0  |   10 <- 01  2 Bits Change, erroneous behaviour
            10  |  0  |   10 <- 10  No Change
            11  | -1  |   10 <- 11  Backward
            12  |  0  |   11 <- 00  2 Bits Change, erroneous behaviour
            13  | -1  |   11 <- 01  Backward
            14  |  1  |   11 <- 10  Forward
            15  |  0  |   11 <- 11  No Change
*/



void initEncoders() {
  pinMode(M1_ENCDRA, INPUT_PULLUP);
  pinMode(M1_ENCDRB, INPUT_PULLUP);
  pinMode(M2_ENCDRA, INPUT_PULLUP);
  pinMode(M2_ENCDRB, INPUT_PULLUP);
  pinMode(M3_ENCDRA, INPUT_PULLUP);
  pinMode(M3_ENCDRB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M1_ENCDRA), ISR_M1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENCDRB), ISR_M1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENCDRA), ISR_M2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENCDRB), ISR_M2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M3_ENCDRA), ISR_M3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M3_ENCDRB), ISR_M3, CHANGE);
}

void ISR_M1() {
  static uint8_t enc_last = 0;
  enc_last <<= 2;                                                      // Bit shift left by 2. This saves the last values
  enc_last |= (digitalRead(M1_ENCDRA) << 1 | digitalRead(M1_ENCDRB));  // 2 LSBs become new encoder values
  m1_enc_pos += ENC_STATES[(enc_last & 0x0f)];                         // Workout direction from state table and update count
}

void ISR_M2() {
  static uint8_t enc_last = 0;
  enc_last <<= 2;                                                      // Bit shift left by 2. This saves the last values
  enc_last |= (digitalRead(M2_ENCDRA) << 1 | digitalRead(M2_ENCDRB));  // 2 LSBs become new encoder values
  m2_enc_pos += ENC_STATES[(enc_last & 0x0f)];                         // Workout direction from state table and update count
}

void ISR_M3() {
  static uint8_t enc_last = 0;
  enc_last <<= 2;                                                      // Bit shift left by 2. This saves the last values
  enc_last |= (digitalRead(M3_ENCDRA) << 1 | digitalRead(M3_ENCDRB));  // 2 LSBs become new encoder values
  m3_enc_pos += ENC_STATES[(enc_last & 0x0f)];                         // Workout direction from state table and update count
}


/* Wrap the encoder reading function */
long readEncoder(int i) {
  if (i == MOTOR1) return m1_enc_pos;
  else if (i == MOTOR2) return m2_enc_pos;
  else if (i == MOTOR3) return m3_enc_pos;
  else return 0L;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == MOTOR1) {
    m1_enc_pos = 0L;
    return;
  } else if (i == MOTOR2) {
    m2_enc_pos = 0L;
    return;
  } else if (i == MOTOR3) {
    m3_enc_pos = 0L;
    return;
  }
}
#else
#error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(MOTOR1);
  resetEncoder(MOTOR2);
  resetEncoder(MOTOR3);
}
