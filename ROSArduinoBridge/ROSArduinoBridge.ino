/*********************************************************************
 *  ROSArduinoBridge
 
    
 *********************************************************************/

/* Use PR-H1-01 Motor Control Shield */
#define PR_H1_01

/* Serial port baud rate */
#define BAUDRATE 57600

/* Maximum PWM signal */
#define MAX_PWM 255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"


#ifdef PR_H1_01

#include "motor_driver.h"    // Motor driver function definitions
#include "encoder_driver.h"  // Encoder driver function definitions
//#include "diff_controller.h"    // PID parameters and functions


#define PID_RATE 30                        // [Hz]  Run the PID loop at PID_RATE times per second
const int PID_INTERVAL = 1000 / PID_RATE;  // Convert the rate into an interval
unsigned long nextPID = PID_INTERVAL;      // Track the next time we make a PID calculation

#define AUTO_STOP_INTERVAL 2000  // Stop the robot if a command is not recieved in xx milliseconds - SAFETY!
long lastMotorCommand = AUTO_STOP_INTERVAL;

#endif

/* Variable initialization */

static uint8_t moving = 0;

static uint8_t arg = 0;    // Count number of arguments in a command
static uint8_t ind = 0;    // Current index in command char array
static char chr;           // Temp Input character
static char cmd;           // Current command
static char argChar1[16];  // Character arrays to hold command arguments
static char argChar2[16];
static char argChar3[16];
long arg1;                 // The arguments converted to integers
long arg2;
long arg3;


// Clear the Current Command
void resetCommand() {
  cmd = ' ';
  memset(argChar1, 0, sizeof(argChar1));
  memset(argChar2, 0, sizeof(argChar2));
  memset(argChar3, 0, sizeof(argChar3));
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg = 0;
  ind = 0;
}

// Print the current command
void printCommand() {
  Serial.print("Command: <");
  Serial.print(cmd);
  Serial.print("> with Arguments <");
  Serial.print(argChar1);
  Serial.print("> <");
  Serial.print(argChar2);
  Serial.print("> <");
  Serial.print(argChar3);
  Serial.println("> ");
}

// Read in a command
void processCMD() {
  while (Serial.available() > 0) {
    chr = Serial.read();  // Read the next character

    if (chr == 13) {      // Terminate a command with a CR
      printCommand();
      runCommand();
      resetCommand();
    }

    else if (chr == ' ') {  // Use spaces to delimit parts of the command
      if (arg == 0)
        arg = 1;
      else if (arg == 1) {
        arg = 2;
        ind = 0;
      } else if (arg == 2) {
        arg = 3;
        ind = 0;
      }
      continue;
    }

    else {
      if (arg == 0) {
        cmd = chr;  // The first arg is the single-letter command
                    // Subsequent arguments can be more than one character
      } else if (arg == 1) {
        argChar1[ind] = chr;
        ind++;
      } else if (arg == 2) {
        argChar2[ind] = chr;
        ind++;
      } else if (arg == 3) {
        argChar3[ind] = chr;
        ind++;
      }
    }
  }
}


// Run a Command
int runCommand() {
  arg1 = atoi(argChar1);
  arg2 = atoi(argChar2);
  arg3 = atoi(argChar3);

  switch (cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;

    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;

    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;

    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;

    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;

    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;

    case PING:
      //    Serial.println(Ping(arg1));
      break;

#ifdef PR_H1_01
    case READ_ENCODERS:
      Serial.print(readEncoder(MOTOR1));
      Serial.print(" ");
      Serial.print(readEncoder(MOTOR2));
      Serial.print(" ");
      Serial.println(readEncoder(MOTOR3));
      break;

    case RESET_ENCODERS:
      resetEncoders();
      //resetPID();
      Serial.println("OK");
      break;

    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0 && arg3 == 0) {
        setMotorSpeeds(0, 0, 0);
        //resetPID();
        //moving = 0;
      } else {
      }  // moving = 1;
      //leftPID.TargetTicksPerFrame = arg1;
      //rightPID.TargetTicksPerFrame = arg2;
      //rightPID.TargetTicksPerFrame = arg3;
      Serial.println("OK");
      break;

    case MOTOR_RAW_PWM:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      //  resetPID();
      //  moving = 0;  // Sneaky way to temporarily disable the PID
      setMotorSpeeds(arg1, arg2, arg3);
      Serial.println("OK");
      break;

    case UPDATE_PID:
      //      while ((str = strtok_r(p, ":", &p)) != '\0') {
      //        pid_args[i] = atoi(str);
      //        i++;
      //      }
      //Kp = pid_args[0];
      //Kd = pid_args[1];
      //Ki = pid_args[2];
      //Ko = pid_args[3];
      Serial.println("OK");
      break;
#endif

    default:
      Serial.println("Invalid Command");
      break;
  }
  return 0;
}





/* SETUP FUNCTION
    1) Serial
    2) PR-H1-01 Hardware - Motors and Encoders
    3) TBC
*/
void setup() {
  Serial.begin(BAUDRATE);

#ifdef PR_H1_01
  initMotorControllers();
  initEncoders();
  enableMotorControllers();
#endif

}





/* MAIN LOOP
    1)  Read and parse input from the serial port and run any valid commands
    2)  Run a PID calculation at the target
    3)  Check for auto-stop conditions
*/
void loop() {
  processCMD();

  // If we are using base control, run a PID calculation at the appropriate intervals
  //#ifdef USE_BASE
  //  if (millis() > nextPID) {
  //    updatePID();
  //    nextPID += PID_INTERVAL;
  //  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    ;
    setMotorSpeeds(0, 0, 0);
    moving = 0;
  }
}
