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
String received_chars;
#define INCREMENT true
#define DECREMENT false
bool opMode = true;
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
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    received_chars.concat(inChar);
    if (inChar == '\n')
    {
	  char command = received_chars[0];
	  received_chars.remove(0,1);
      float num = received_chars.toInt();
	  Serial.print("Command == ");
	  Serial.println(command);
	  Serial.print("Num == ");
	  Serial.println(num);
      if(command == 's'){//stop
        radPerSec = 0;
        mxRads = 0;
        received_chars = "";
        Serial.println("Stopping Motor.");
        break;
      }
      else if(command == 'g'){//go
        radPerSec = 0;
        mxRads = 300;
        received_chars = "";
        Serial.println("Starting Motor.");
        break;
      }
	  else if(command == 't'){
		if(mxRads < num){
			opMode = INCREMENT;
		}
		else{
			opMode = DECREMENT;
		}
		mxRads = num;
		spinIt = 1;

		Serial.print("Changing target speed to ");
		Serial.print(num*9.549297);
		Serial.println("RPM.");
        received_chars = "";
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
  driver.voltage_power_supply = 14.8;
  driver.pwm_frequency = 50000;
  driver.init();

  motor.linkDriver(&driver);
  motor.voltage_limit = 14.8;
  //motor.current_limit = 1;
  motor.velocity_limit = 2212;
  

  //Sets up openloop FOC
  motor.controller = MotionControlType::velocity_openloop;
  // set FOC modulation type to sinusoidal
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;


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
  switch(opMode){
	case INCREMENT:
		if (spinIt%spinItSpeed == 0 && radPerSec<1580){
		radPerSec++;
		spinIt = 1;
		}
		else{
		spinIt++;
		}
		break;
	case DECREMENT:
		if (spinIt%spinItSpeed == 0 && radPerSec>=mxRads){
		radPerSec--;
		spinIt = 1;
		}
		else{
		spinIt++;
		}
		break;
  }
  //motor.PID_velocity.P = target;
  
  //serialControl();
}
