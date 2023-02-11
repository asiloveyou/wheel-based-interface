#include <SimpleFOC.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


//////////////////////
/// Initialization ///
//////////////////////

// init driver
  BLDCDriver3PWM driver = BLDCDriver3PWM(5, 9, 6, 8);
// init encoder
Encoder encoder = Encoder(3, 2, 2048, 4);
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
// init motor
BLDCMotor motor = BLDCMotor(11);
// init LCD
LiquidCrystal_I2C lcd(0x27,20,4);

////////////////////////////////////
/// Global Variable and Function ///
////////////////////////////////////

// Other Usage //
char printBuffer[10];
bool displayOnRefresh = false;

// Write to LCD screen //
void writeLcd(char* firstLine, char* secondLine){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(firstLine);
  lcd.setCursor(0, 1);
  lcd.print(secondLine);
}

// Write to LCD screen on loop //
// Since loop()runs on 117 kHz (117,000 times per second)
// The effect is defined as it is
// *delay(ms) = (x)ms interval
void writeLcdLoop(char* firstLine, char* secondLine){
  if(!displayOnRefresh){
    displayOnRefresh = true;
    writeLcd(firstLine, secondLine);
    delay(200);
    displayOnRefresh = false;
  }
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
  lcd.backlight();
  writeLcd("Program", "Initializing");

  /////////////////////
  /// Encoder Setup ///
  /////////////////////

  encoder.init();
  encoder.enableInterrupts(doA, doB);

  ////////////////////
  /// Driver Setup ///
  ////////////////////

  driver.init();
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;

  // driver init
  if(driver.init()) Serial.println("Driver Initialization [success]");
  else{
    Serial.println("Driver Initialization [fail]");
    return;
  }

  // enable driver
  driver.enable();

  ///////////////////
  /// Motor Setup ///
  ///////////////////

  motor.voltage_limit = 3;
  motor.velocity_limit = 5;
  motor.controller = MotionControlType::velocity_openloop;

  motor.init();  

  writeLcd("Active: ", "Driver Test");
  _delay(1000);
}

void loop() {
  encoder.update(); 

  /////////////////////
  // Encoder Testing //
  /////////////////////

  // ANGLE PRINTING //
  float angle = encoder.getAngle();
  dtostrf(angle, 5, 3, printBuffer);
  writeLcdLoop("Current Angle", printBuffer);

  ////////////////////
  // Motor Testing //
  ////////////////////

  motor.move();

}