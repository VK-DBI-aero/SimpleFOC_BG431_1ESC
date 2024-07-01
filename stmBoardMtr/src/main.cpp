// BLDC driver standalone example
#include <SimpleFOC.h>


// BLDC motor instance BLDCMotor(polepairs, (R), (KV))
BLDCMotor motor = BLDCMotor(7, .12, 1400);

// BLDC driver instance BLDCDriver6PWM(phA_h, phA_l, phB_h, phB_l, phC_h, phC_l)
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

/*current sensor*/ 
// (shunt resistor value, gain value, pins phase A,B,C)
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003f, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);


void setup() {

  //DRIVER INITIALIZATION
  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 30000;
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;
  // driver init
  driver.init();

  //MOTOR INITIALIZATION
  //Link the motor to the Driver and Current Sensor
  motor.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);
  //set motor control type
  motor.controller = MotionControlType::velocity;

  //setup serial monitoring
  Serial.begin(115200);
  motor.useMonitoring(Serial);

  motor.init();
  // Will eventually go here
  //motor.initFOC();

  //CURRENT SENSOR INITIALIZATION
  current_sense.linkDriver(&driver);
  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  _delay(1000);
}

void loop() {
    // setting pwm (A: 3V, B: 1V, C: 5V)
    driver.setPwm(3,1,5);
    delay(8);
    Serial.print("Current ==");
    PhaseCurrent_s currents = current_sense.getPhaseCurrents();
    float current_mag = current_sense.getDCCurrent();
    Serial.print(currents.a*1000); // milli Amps
    Serial.print("\t");
    Serial.print(currents.b*1000); // milli Amps
    Serial.print("\t");
    Serial.print(currents.c*1000); // milli Amps
    Serial.print("\t");
    Serial.println(current_mag*1000); // milli Amps
    Serial.println();
    driver.setPwm(5,3,1);
    delay(8);
    driver.setPwm(1,5,3);
    delay(8);
}
