#include <SimpleFOC.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//////////////////////
/// Initialization ///
//////////////////////

// init driver
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
// init encoder
Encoder encoder = Encoder(2, 3, 2048);
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
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
  // driver.setPwm(3, 3, 3);

  driver.setPhaseState(_HIGH_IMPEDANCE , _HIGH_IMPEDANCE, _HIGH_IMPEDANCE);
  driver.setPwm(0,0,0);

  writeLcd("Active: ", "Driver Test");
  _delay(1000);
}

void loop() {
  encoder.update(); 
  driver.setPhaseState(_HIGH_IMPEDANCE , _HIGH_IMPEDANCE, _HIGH_IMPEDANCE);
  driver.setPwm(0,0,0);

  /////////////////////
  // Encoder Testing //
  /////////////////////

  // ANGLE PRINTING //
  float ea = encoder.getAngle();
  dtostrf(ea, 5, 3, printBuffer);
  writeLcdLoop("Current Angle", printBuffer);

  // VELOCITY PRINTING //
  // float ev = encoder.getVelocity();
  // dtostrf(ev, 5, 3, printBuffer);
  // writeLcdLoop("Current Velocity", printBuffer);

  ////////////////////
  // Driver Testing //
  ////////////////////

}