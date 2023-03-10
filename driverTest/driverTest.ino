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
  // driver.setPhaseState(_HIGH_IMPEDANCE , _HIGH_IMPEDANCE, _HIGH_IMPEDANCE);
  // driver.setPwm(1,1,1);

  writeLcd("Active: ", "Driver Test");
  _delay(1000);
}

void loop() {
  encoder.update(); 

  /////////////////////
  // Encoder Testing //
  /////////////////////

  // ANGLE PRINTING //
  float ea = encoder.getAngle();
  dtostrf(ea, 5, 3, printBuffer);
  writeLcdLoop("Current Angle", printBuffer);

  ////////////////////
  // Driver Testing //
  ////////////////////

      // phase (A: 3V, B: 6V, C: high impedance )  
    // set the phase C in high impedance mode - disabled or open
    driver.setPhaseState(_ACTIVE , _ACTIVE , _HIGH_Z); // _HIGH_Z or _HIGH_IMPEDANCE
    driver.setPwm(3, 6, 0); 
    // _delay(1000);

    // // phase (A: 3V, B: high impedance, C: 6V )  
    // // set the phase B in high impedance mode - disabled or open
    // driver.setPhaseState(_ACTIVE , _HIGH_IMPEDANCE, _ACTIVE);
    // driver.setPwm(3, 0, 6);
    // _delay(1000);

    // // phase (A: high impedance, B: 3V, C: 6V )  
    // // set the phase A in high impedance mode - disabled or open
    // driver.setPhaseState(_HIGH_IMPEDANCE, _ACTIVE, _ACTIVE);
    // driver.setPwm(0, 3, 6);
    // _delay(1000);

}