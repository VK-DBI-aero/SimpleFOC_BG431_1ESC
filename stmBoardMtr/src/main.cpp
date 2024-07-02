
#include <Arduino.h>
#include <SimpleFOC.h>

float target = 0.0;

BLDCMotor motor = BLDCMotor(7, .12, 1400);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// user defined function for reading the phase currents
// returning the value per phase in amps
PhaseCurrent_s readCurrentSense(){
  PhaseCurrent_s c;
  // dummy example only reading analog pins
  c.a = analogRead(A_OP1_OUT);
  c.b = analogRead(A_OP2_OUT);
  c.c = analogRead(A_OP3_OUT); // if no 3rd current sense set it to 0
  return(c);
}

// user defined function for intialising the current sense
// it is optional and if provided it will be called in current_sense.init()
void initCurrentSense(){
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
}

GenericCurrentSense current_sense = GenericCurrentSense(readCurrentSense, initCurrentSense);


void serialLoop()
{
  static String received_chars;
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    received_chars += inChar;
    if (inChar == '\n')
    {
      target = received_chars.toFloat();
      Serial.print("Target = ");
      Serial.println(target);
      received_chars = "";
    }
  }
}


void setup()
{

  driver.voltage_power_supply = 12;
  driver.init();

  motor.linkDriver(&driver);
  current_sense.linkDriver(&driver);

  motor.voltage_limit = 14.8;
  motor.velocity_limit = 2212;

  motor.controller = MotionControlType::velocity_openloop;

  motor.init();
  motor.initFOC();

  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_VEL, _MON_CURR_D, _MON_CURR_Q;
  motor.monitor_downsample = 10000; // default 10

  Serial.begin(115200);
  
  delay(1000);

}

int spinIt = 1;
float radPerSec = 0;

void loop()
{
  serialLoop();

  //motor.PID_velocity.P = target;

  motor.loopFOC();
  motor.move(radPerSec);
  motor.monitor();
  if (spinIt%1500 == 0 && radPerSec<500){
    radPerSec++;
    spinIt = 1;
    // Serial.println(radPerSec*9.549297);
  }
  else{
    spinIt++;
  }

}