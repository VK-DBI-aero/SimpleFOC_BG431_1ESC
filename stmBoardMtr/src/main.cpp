
#include <Arduino.h>
#include <SimpleFOC.h>
#include <string>
#include "stm32g4xx_hal.h"
#include <SimpleFOCDrivers.h>
#include <encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h>
#include <RTTStream.h>


RTTStream rtt;

//local variable definitions
float target = 0.0;     // used to change currently tested variable
int spinIt = 1;         // iterator for the startup
int spinItSpeed = 1500; // rate of the startup, higher = slower
float radPerSec = 0;    // driver the motor speed
int mxRads = 0;         // max motor speed
int outIt = 0;          //used for communication rate iteration

//Set up the motor and driver, motor is 7 pp, .12 phase resistor, and 1400KV 
BLDCMotor motor = BLDCMotor(7, .12, 1400, .000011);
MXLEMMINGObserverSensor sensor = MXLEMMINGObserverSensor(motor);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

/*controls the motor using the UART. 
-g starts the motor and revs up to 300 rad/s, 
-s stops the motor, 
-and entering a number sets the P in pid to that number
  -(currently not useful, but implemented for later tuning)
*/

// current sensor
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// Commander interface constructor
Commander command = Commander(rtt);

void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doMotor(char* cmd){ command.motor(&motor,cmd); }


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
  SimpleFOCDebug::enable();
  motor.linkSensor(&sensor);



  //initializes the psu and driver
  driver.voltage_power_supply = 14.8;
	driver.pwm_frequency = 25000;
	driver.dead_zone = 0.03;
  driver.init();

	// link the driver to the current sense
  current_sense.linkDriver(&driver);

  motor.linkDriver(&driver);
	motor.voltage_sensor_align  = 2;                            // aligning voltage
	motor.foc_modulation        = FOCModulationType::SpaceVectorPWM; // Only with Current Sense
	motor.controller            = MotionControlType::torque;    // set motion control loop to be used
	motor.torque_controller     = TorqueControlType::foc_current;


	if (motor.controller == MotionControlType::torque || motor.controller == MotionControlType::angle || motor.controller == MotionControlType::velocity){
		if (motor.torque_controller == TorqueControlType::foc_current || motor.torque_controller == TorqueControlType::dc_current){
		// When current sensing is used, reduce the voltage limit to have enough low side ON time for phase current sampling  
		    motor.voltage_limit = driver.voltage_power_supply * 0.54;
		}else{
		    motor.voltage_limit = driver.voltage_power_supply * 0.58;
		}
	}else{
		// For openloop angle and velocity modes, use very small limit
		motor.voltage_limit = driver.voltage_power_supply * 0.05;
	}

  SimpleFOCDebug::enable(&rtt);

	motor.monitor_downsample = 100; // set downsampling can be even more > 100
  motor.monitor_variables =  _MON_VEL;// _MON_CURR_Q | _MON_CURR_D; // set monitoring of d and q currents _MON_TARGET | _MON_VEL | _MON_ANGLE |
	motor.monitor_decimals = 2; //!< monitor outputs decimal places
    



  motor.init();

	// current sense init hardware
	current_sense.skip_align = true;
	current_sense.init();
	// link the current sense to the motor
	motor.linkCurrentSense(&current_sense);

	// !!! The MXLEMMING observer sensor doesn't need sensor alignment
	motor.sensor_direction= Direction::CW;
  motor.zero_electric_angle = 0;


  motor.initFOC();

  	// set the initial motor target
	motor.target = 0; // unit depends on control mode 

	// add target command T
	command.add('T',doTarget, "target angle");
	command.add('M',doMotor,"my motor motion");

  Serial.begin(115200);
  delay(1000);

}

LowPassFilter LPF_target(0.5);  //  the higher the longer new values need to take effect
PhaseCurrent_s currents;
DQCurrent_s dqcurrents;
float dccurrent, dcpower;

float angle_el;


void loop()
{
	if (motor.controller == MotionControlType::torque || motor.controller == MotionControlType::angle || motor.controller == MotionControlType::velocity){
		if (motor.torque_controller == TorqueControlType::foc_current || motor.torque_controller == TorqueControlType::dc_current){
		// When current sensing is used, reduce the voltage limit to have enough low side ON time for phase current sampling  
		    motor.voltage_limit = driver.voltage_power_supply * 0.54;
		}else{
		    motor.voltage_limit = driver.voltage_power_supply * 0.58;
		}
	}else{
		// For openloop angle and velocity modes, use very small limit
		motor.voltage_limit = driver.voltage_power_supply * 0.05;
	}

	motor.loopFOC();
	motor.move(LPF_target(target));
	//motor.monitor();
	command.run();

	if (current_sense.initialized){
		currents = current_sense.getPhaseCurrents();
		//dqcurrents = current_sense.getFOCCurrents(motor.electrical_angle);
		//dqcurrents.q = motor.LPF_current_q(dqcurrents.q);
		//dqcurrents.d = motor.LPF_current_d(dqcurrents.d);
	
		dcpower = 1.5f * (motor.current.q * motor.voltage.q) + 0.8 + (0.1664 + 0.003 * motor.current.q * motor.current.q);
		dccurrent = dcpower / driver.voltage_power_supply; 
    }

  
  //serialControl();
}

