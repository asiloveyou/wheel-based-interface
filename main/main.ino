#include <SimpleFOC.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//////////////////////
/// Initialization ///
//////////////////////

// init BLDC motor
BLDCMotor motor = BLDCMotor( 11 );
// init driver
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
// init encoder
Encoder encoder = Encoder(2, 3, 2048);
// init LCD
LiquidCrystal_I2C lcd(0x27,20,4);
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

////////////////////////////////////
/// Global Variable and Function ///
////////////////////////////////////

// angle set point variable
float target_value = 0;
// commander interface
Commander command = Commander(Serial);
void doTarget(char* cmd){ command.scalar(&target_value, cmd); }
// write to thet screen
void writeLcd(char* firstLine, char* secondLine){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(firstLine);
  lcd.setCursor(0, 1);
  lcd.print(secondLine);
}

void setup() {

  ////////////////////
  /// Serial Setup ///
  ////////////////////

  Serial.begin(115200);

  /////////////////
  /// LCD Setup ///
  /////////////////
  
  lcd.init();
  writeLcd("Program", "Initializing");

  /////////////////////
  /// Encoder Setup ///
  /////////////////////

  // initialize encoder hardware
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  ////////////////////
  /// Driver Setup ///
  ////////////////////

  // power supply voltage
  // default 12V
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;
  // Initialize driver
  if (driver.init()) Serial.println("Driver success!");
  else{
    Serial.println("Driver failed!");
    return;
  }

  // Enable driver
  driver.enable();
  // link the motor to the driver
  motor.linkDriver(&driver);

  ///////////////////
  /// Motor Setup ///
  ///////////////////

  // set control loop to be used
  // case of angle control
  // motor.controller = MotionControlType::angle;
  // case of velocity control
  motor.controller = MotionControlType::velocity_openloop;
  
  // controller configuration based on the control type 
  // velocity PI controller parameters
  // default P=0.5 I = 10
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
  
  //default voltage_power_supply
  motor.voltage_limit = 6;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;

  // angle P controller 
  // default P=20
  motor.P_angle.P = 20;
  //  maximal velocity of the position control
  // default 20
  motor.velocity_limit = 4;
  
  // initialize motor
  motor.init();
  // align encoder and start FOC
  // motor.initFOC();

  ////////////////////
  /// Serial Setup ///
  ///////////////////.

  // add target command T
  command.add('T', doTarget, "target value");
  
  Serial.println("Motor ready.");
  Serial.println("Set the target angle using serial terminal:");

  writeLcd("Initialization", "Finished");
  _delay(1000);
}

float curAngle = 0.0f;

void loop() {
  encoder.update();
  if(curAngle != encoder.getAngle()){
    curAngle = encoder.getAngle();
    Serial.print(curAngle);
    Serial.print("\n");
  }

  // // iterative FOC function
  motor.loopFOC();

  // // function calculating the outer position loop and setting the target position 
  // motor.move(target_value);
  motor.move();

  // user communication
  command.run();
}