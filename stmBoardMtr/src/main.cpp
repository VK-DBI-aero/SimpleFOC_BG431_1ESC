// BLDC driver standalone example
#include <SimpleFOC.h>

// BLDC motor instance BLDCMotor(polepairs, (R), (KV))
BLDCMotor motor = BLDCMotor(7, .12, 1400);

// BLDC driver instance BLDCDriver6PWM(phA_h, phA_l, phB_h, phB_l, phC_h, phC_l, (en))
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);


/*current sensor*/ 
// (shunt resistor value,
// gain value,
// pins phase A,B)
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003f, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);


void setup() {
  
  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 30000;
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;
  // daad_zone [0,1] - default 0.02 - 2%
  driver.dead_zone = 0.05;

  // driver init
  driver.init();
  current_sense.linkDriver(&driver);

  motor.init();

  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  //motor.initFOC();


  // enable driver
  driver.enable();
  
  Serial.begin(115200);
  Serial.println("Setup ready.");


}

void loop() {
    // setting pwm
    // phase A: 3V, phase B: 6V, phase C: 5V
    // driver.setPwm(6,6,5);


    // foc and motion controls
    //motor.loopFOC();
    motor.move();

    PhaseCurrent_s currents = current_sense.getPhaseCurrents();
    float current_magnitude = current_sense.getDCCurrent();

    Serial.print(currents.a*1000); // milli Amps
    Serial.print("\t");
    Serial.print(currents.b*1000); // milli Amps
    Serial.print("\t");
    Serial.print(currents.c*1000); // milli Amps
    Serial.print("\t");
    Serial.println(current_magnitude*1000); // milli Amps


}