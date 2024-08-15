#include <Arduino.h>
#include <SimpleFOC.h>
#include <string>
#include "stm32g4xx_hal.h"

//Operation Mode Definitions
#define INCREMENT true
#define DECREMENT false

// BEMF for 20 volts @ 3282 RPM
//  Vbemf = BEMF constant * angular velocity
//  2.68 = K * 343.690236 rad/s
//  Bemf Constant = .007797719339
// local variable definitions
int spinIt = 1;             // iterator for the startup
int spinItSpeed = 200;      // rate of the startup, higher = slower
int mxRads = 0;             // max motor speed, set but uart commands later
float radPerSec = 0;   
String received_chars = ""; // stores the uart recieved chars
bool opMode = INCREMENT;    // operation mode, increment or decrement
float ddZn = 0.0;           // used to change dead zone. TODO: Implement
float numInt;               // converts the uart in to int 
float numFlt;               // converts the uart in to float

// Set up the motor and driver, motor is 7 pp, .12 phase resistor, and 1400KV
BLDCMotor motor = BLDCMotor(7, .12, 1400);
// Used pins defined by stmboard to be controlled by the driver
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// Set up the ablility to sense current TODO:Make work, this is right but later use does not
LowsideCurrentSense currentsense = LowsideCurrentSense(0.003, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);



/********************************UART COMMANDS*********************************/
//for optimal speeds the UART commands use function pointers in a LUT that is selected by the first character of the uart command 

typedef void (*FunctionPointer)();

//stops the motor by setting the current speed and max speed to 0
void stopMtr()
{
  radPerSec = 0;
  mxRads = 0;
  received_chars = "";
  Serial.println("Stopping Motor.");
}

//start the motor by setting the max speed to a default of 300 rad/s and current speed to 0
void startMtr()
{
  radPerSec = 0;
  mxRads = 300;
  received_chars = "";
  Serial.println("Starting Motor.");
}

//will be used to adjust the deadzone for fine tuning
void ddZnSet()
{
  driver.dead_zone = numFlt;
  radPerSec = 0;
  received_chars = "";
  Serial.print("Setting dead zone to ");
  Serial.println(numFlt);
}

// Sets the motors speed to a user defined value recieved over UART
// it first checks if the current value is larger of smaller than the current target and set the operation mode. The mxRads is then set to the target
void setMtrTrgt()
{
  if (mxRads < numInt)
  {
    opMode = INCREMENT;
  }
  else
  {
    opMode = DECREMENT;
  }

  mxRads = numInt;
  spinIt = 1;

  Serial.print("Changing target speed to ");
  Serial.print(numInt * 9.549297);
  Serial.println("RPM.");
  received_chars = "";
}

/* LUT that is used to select the function. 
    0 = stop motor
    1 = start motor
    2 = set the dead zone
    3 = set the motor speed

*/
FunctionPointer uartLUT[] = {stopMtr, startMtr, ddZnSet, setMtrTrgt};

//UART reciever driver. The UART recieves and concats each char into received_chars untill the EOl is received\n it then extracts the command id and the value sent with it if there is one and calls the command functiom
void serialControl()
{
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    received_chars.concat(inChar);
    if (inChar == '\n')
    {
      String command = (String)received_chars[0];
      int cmdInt = command.toInt();
      received_chars.remove(0, 1);
      numInt = received_chars.toInt();
      numFlt = received_chars.toFloat();
      uartLUT[cmdInt]();
      received_chars = "";
    }
  }
}


void setup()
{
  //UART set to a baud of 4800
  Serial.begin(4800);

  // initializes the psu and driver
  driver.voltage_power_supply = 20;
  driver.pwm_frequency = 50000;
  // driver.dead_zone = 0.02;
  driver.init();

  motor.linkDriver(&driver);
  currentsense.linkDriver(&driver);
  motor.voltage_limit = 20;
  // motor.current_limit = 1;
  motor.velocity_limit = 2212;

  // Sets up openloop FOC
  motor.controller = MotionControlType::velocity_openloop;
  // set FOC modulation type to sinusoidal
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.init();
  currentsense.init();
  motor.linkCurrentSense(&currentsense);
  currentsense.driverAlign(1);

  motor.initFOC();

  delay(1000);
}

void loop()
{
  // starts the motor moving at radPerSec
  motor.monitor();
  motor.loopFOC();
  motor.move(radPerSec);
  // this loop controls the rate at which the motor speeds up
  switch (opMode)
  {
  case INCREMENT:
    if (spinIt % spinItSpeed == 0 && radPerSec < mxRads)
    {
      radPerSec++;
      spinIt = 1;
    }
    else
    {
      spinIt++;
    }
    break;
  case DECREMENT:
    if (spinIt % spinItSpeed == 0 && radPerSec >= mxRads)
    {
      radPerSec--;
      spinIt = 1;
    }
    else
    {
      spinIt++;
    }
    break;
  }

  serialControl();
}
