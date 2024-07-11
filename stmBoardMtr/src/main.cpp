#include <Arduino.h>
#include <SimpleFOC.h>
#include <string>
#include "stm32g4xx_hal.h"

//local variable definitions
float target = 0.0;     // used to change currently tested variable
int spinIt = 1;         // iterator for the startup
int spinItSpeed = 1500; // rate of the startup, higher = slower
float radPerSec = 0;    // driver the motor speed
int mxRads = 0;         // max motor speed
int outIt = 0;          //used for communication rate iteration

//Set up the motor and driver, motor is 7 pp, .12 phase resistor, and 1400KV 
BLDCMotor motor = BLDCMotor(7, .12, 1400);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

/*controls the motor using the UART. 
-g starts the motor and revs up to 300 rad/s, 
-s stops the motor, 
-and entering a number sets the P in pid to that number
  -(currently not useful, but implemented for later tuning)
*/
void serialControl(){
  static String received_chars;
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    received_chars += inChar;
    if (inChar == '\n')
    {
      float num = received_chars.toFloat();
      if(received_chars == "s\n"){
        radPerSec = 0;
        mxRads = 0;
        received_chars = "";
        Serial.println("Stopping Motor.");
        break;
      }
      else if(received_chars == "g\n"){
        radPerSec = 0;
        mxRads = 300;
        received_chars = "";
        Serial.println("Starting Motor.");
        break;
      }
      else{
        target = num;
        Serial.print("PID.P = ");
        Serial.println(target);
        received_chars = "";
      }    
    }
  }
}

void setup()
{
  Serial.begin(115200);
  SimpleFOCDebug::enable();

  //initializes the psu and driver
  driver.voltage_power_supply = 12.8;
  driver.init();

  motor.linkDriver(&driver);
  motor.voltage_limit = 14.8;
  motor.velocity_limit = 2212;

  //Sets up openloop FOC
  motor.controller = MotionControlType::velocity_openloop;
  // set FOC modulation type to sinusoidal
  motor.foc_modulation = FOCModulationType::SinePWM;


  motor.init();

  motor.initFOC();

  
  delay(1000);

}


void loop()
{
  //starts the motor moving at radPerSec
  motor.loopFOC();
  motor.move(radPerSec);
  //this loop controls the rate at which the motor speeds up
  if (spinIt%spinItSpeed == 0 && radPerSec<mxRads){
    radPerSec++;
    spinIt = 1;
  }
  else{
    spinIt++;
  }

  //motor.PID_velocity.P = target;
  
  serialControl();
}
