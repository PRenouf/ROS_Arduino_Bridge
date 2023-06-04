/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ANALOG_READ    'A'
#define GET_BAUDRATE   'B'
#define PIN_MODE       'C'
#define DIGITAL_READ   'D'
#define READ_ENCODERS  'E'
#define MOTOR_SPEEDS   'M'
#define MOTOR_RAW_PWM  'O'
#define PING           'P'
#define RESET_ENCODERS 'R'
#define SERVO_WRITE    'S'
#define SERVO_READ     'T'
#define UPDATE_PID     'U'
#define DIGITAL_WRITE  'W'
#define ANALOG_WRITE   'X'
#define MOTOR1          0
#define MOTOR2          1
#define MOTOR3          2

#endif


