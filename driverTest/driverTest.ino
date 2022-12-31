#include <SimpleFOC.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//////////////////////
/// Initialization ///
//////////////////////

// init driver
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
// init LCD
LiquidCrystal_I2C lcd(0x27,20,4);

////////////////////////////////////
/// Global Variable and Function ///
////////////////////////////////////

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

  ////////////////////
  /// Driver Setup ///
  ////////////////////

  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;

  // driver init
  driver.init();

  // enable driver
  driver.enable();

  // driver.setPhaseState(_HIGH_IMPEDANCE , _HIGH_IMPEDANCE, _HIGH_IMPEDANCE);
  // driver.setPwm(0,0,0);

  writeLcd("Active: ", "Driver Test");
  _delay(1000);
}

void loop() {

}