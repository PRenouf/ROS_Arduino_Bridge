/***************************************************************
   Motor driver function definitions - by Phil Renouf
   *************************************************************/

#ifdef PR_H1_01
  #define M1_ENA 10
  #define M1_ENB 8
  #define M1_INA 11
  #define M1_INB 7
  #define M1_PWM 9
  #define M2_ENA 19
  #define M2_ENB 16
  #define M2_INA 20
  #define M2_INB 15
  #define M2_PWM 18
  #define M3_ENA 3
  #define M3_ENB 5
  #define M3_INA 2
  #define M3_INB 22
  #define M3_PWM 4

  #define TEST_SPEED 50
  #define TEST_MOTOR_DURATION 1000
#endif

void initMotorControllers();
void enableMotorControllers();
void enableMotorController(int i);
void disableMotorControllers();
void disableMotorController(int i);
void testMotors();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int motor1Speed, int motor2Speed, int motor3Speed);
