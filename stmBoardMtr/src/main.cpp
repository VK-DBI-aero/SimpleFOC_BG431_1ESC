
#include <Arduino.h>
#include <SimpleFOC.h>

float target = 0.0;

BLDCMotor motor = BLDCMotor(7, .12, 1400);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);


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

  motor.voltage_limit = 14.8;
  motor.velocity_limit = 2212;

  // motor.PID_velocity.P = 0.0;
  // motor.PID_velocity.I = 0.0;
  // motor.LPF_velocity.Tf = 0.01;

  motor.controller = MotionControlType::velocity_openloop;

  motor.init();
  motor.initFOC();

  motor.useMonitoring(Serial);

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
  if (spinIt%1500 == 0 && radPerSec<500){
    radPerSec++;
    spinIt = 1;
    // Serial.println(radPerSec*9.549297);
  }
  else{
    spinIt++;
  }

}