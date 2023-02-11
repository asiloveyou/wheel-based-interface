#include <SimpleFOC.h>

//////////////////////
/// Initialization ///
//////////////////////

// init BLDC motor
BLDCMotor motor = BLDCMotor( 11 );
// init driver
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 9, 6, 8);
// init encoder
Encoder encoder = Encoder(2, 3, 2048);
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

////////////////////////////////////
/// Global Variable and Function ///
////////////////////////////////////

// // Commander interface //
Commander command = Commander(Serial);
void doMotor(char* cmd){ command.motor(&motor, cmd); }

void setup() {

  /////////////////
  /// LCD Setup ///
  /////////////////

  /////////////////////
  /// Encoder Setup ///
  /////////////////////

  encoder.init();
  encoder.enableInterrupts(doA, doB);
  motor.linkSensor(&encoder);

  ////////////////////
  /// Driver Setup ///
  ////////////////////

  driver.voltage_power_supply = 4;
  driver.voltage_limit = 4;
  if (driver.init()) Serial.println("Driver success!");
  else{
    Serial.println("Driver failed!");
    return;
  }

  driver.init();
  motor.linkDriver(&driver);

  ///////////////////
  /// Motor Setup ///
  ///////////////////

  motor.PID_velocity.P = 0.1f;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.D = 0;
  motor.voltage_limit = 1;
  // jerk control using voltage voltage ramp
  motor.PID_velocity.output_ramp = 300;
  
  //default voltage_power_supply
  motor.voltage_limit = 1;

  // velocity low pass filtering
  motor.LPF_velocity.Tf = 0.01f;
  motor.P_angle.P = 20;
  motor.velocity_limit = 10;
  
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  ////////////////////
  /// Serial Setup ///
  ////////////////////

  Serial.begin(115200);

  command.add('M',doMotor,'motor');
  motor.useMonitoring(Serial);

  _delay(1000);
}

void loop() {

  motor.loopFOC();
  motor.monitor();
  motor.move();

  command.run();
}