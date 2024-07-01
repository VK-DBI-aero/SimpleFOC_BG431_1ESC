
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

  motor.controller = MotionControlType::velocity_openloop;

  motor.init();
  motor.initFOC();

  Serial.begin(115200);
  
  delay(1000);

}

float target_velocity = 0;
int rpmIt = 1;

void loop()
{
  serialLoop();

  motor.loopFOC();
  motor.move(target);
  if (rpmIt%1000 == 0 && target<1300){
    target++;
    rpmIt = 1;
  }
  else{
    rpmIt++;
  }
}