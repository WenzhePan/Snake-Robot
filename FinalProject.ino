#include <DynamixelShield.h>
#include <stdlib.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

const uint8_t motor1 = 1;
const uint8_t motor2 = 2;
const uint8_t motor3 = 3;
const uint8_t motor4 = 4;
DynamixelShield dxl;

void setup() {
  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);  // Assuming a baud rate of 57600 for the Dynamixel
  dxl.setJointMode(1);  // Setting motors to joint mode
  dxl.setJointMode(2);
  dxl.setJointMode(3);
  dxl.setJointMode(4);

}
// assuming that the amplitude of the sanke moving and the frequency will be as following:
const float beta_0 = 80;
const float f = 0.1;
const float N = 4;
// the angle of each motor will be defined as follow:
float angle1 = 0;
float angle2 = 0;
float angle3 = -30;
float angle4 = -30;

// offset need to be coinsidered
float offset1 = 0;
float offset2 = 0;
float offset3 = 0;
float offset4 = 0;
void loop() {
  // based on the equation
  // beta_i = beta_0 * sin (2 * M_PI * i / N - 2 * M_PI * f * t)
  // the range of t will be [0, 1/f]
  for(float i = 0; i <= 10; i += 0.1){
    angle1 = beta_0 * sin (- 2 * M_PI * f * i);
    dxl.setGoalPosition(motor1, angle1 + offset1, UNIT_DEGREE);
    angle2 = beta_0 * sin (2 * M_PI * 1 / N - 2 * M_PI * f * i);
    dxl.setGoalPosition(motor2, angle2 + offset2, UNIT_DEGREE);
    angle3 = beta_0 * sin (2 * M_PI * 2 / N - 2 * M_PI * f * i);
    dxl.setGoalPosition(motor3, angle3 + offset3, UNIT_DEGREE);
    angle4 = beta_0 * sin (2 * M_PI * 3 / N - 2 * M_PI * f * i);
    dxl.setGoalPosition(motor4, angle4 + offset4, UNIT_DEGREE);
    // delay(100);
  }
}


