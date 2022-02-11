
#include <LiquidCrystal_I2C.h>
#include <LowPower.h>
#include <Wire.h>

const int arefEnablePin = 2;
const int sensorHighInputPin = 3;
const int sensorLowInputPin = 4;
const int pumpEnablePin = 5;
const int alarmPin = 6;


/* This value is unique to each board, or at least each board family. Make a
 * measurement using the included sample program across a .1uF cap connected
 * between AREF and GND, * 1000.
 */
/* Arduino UNO dev board */
const long InternalReferenceVoltage = 1109L;

LiquidCrystal_I2C lcd(0x27,16,2);

int displayCount = 0;           /* visual indication that we're running */

void setup() {
  // put your setup code here, to run once:

  lcd.init();
  lcd.clear();
  lcd.backlight();
  // lcd.noBacklight();

  // Ensure pump is off before setting pin as an output
  pumpOff();

  // Alarm off
  digitalWrite(alarmPin, LOW);

  pinMode(arefEnablePin, OUTPUT);
  pinMode(sensorHighInputPin, INPUT_PULLUP);
  pinMode(sensorLowInputPin, INPUT_PULLUP);
  pinMode(pumpEnablePin, OUTPUT);
  pinMode(alarmPin, OUTPUT);

  // Connect AREF directly to our (unregulated) input voltage
  digitalWrite(arefEnablePin, HIGH);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Either power-down via interrupt and have a low water level trigger said
  // interrupt, or poll every 8 seconds for water level. Interrupt sounds
  // better for power saving.

  // From https://docs.arduino.cc/learn/electronics/low-power
  int bandGap = getBandgap();
  // displayBandgap(bandGap);

  if (bandGap < 300) {
    lowBatteryWarning();
  }

  if (waterTooLow()) {
    fillTank();
  }

  manualLowPowerMode(1);
}

int waterTooLow() {
  // Return a 1 if the tank needs water added, 0 otherwise
  return !waterAtLowLevel();
}

int tankFull() {
  // Return a 1 if the tank is full (and we should stop the pump), 0 otherwise
  return waterAtHighLevel();
}

int waterAtHighLevel() {
  // Return a 1 if the water is at/above the HIGH level sensor, 0 otherwise
  return digitalRead(sensorHighInputPin);
}

int waterAtLowLevel() {
  // Return a 1 if the water is at/above the LOW level sensor, 0 otherwise
  return digitalRead(sensorLowInputPin);
}

void fillTank() {
  pumpOn();
  while(!tankFull()) {
    delay(1000);
  }
  pumpOff();
}

void pumpOn() {
  // Serial.println("Pump on");
  digitalWrite(pumpEnablePin, HIGH);
}

void pumpOff() {
  // Serial.println("Pump off");
  digitalWrite(pumpEnablePin, LOW);
}

void lowBatteryWarning() {
  digitalWrite(alarmPin, HIGH);
  delay(10);
  digitalWrite(alarmPin, LOW);
}

void manualLowPowerMode(uint8_t multiplier) {
  delay(70);  // Requires at least 68ms of buffer head time for module booting time
  for (int i=0; i<multiplier; i++) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}

void wakeUp() {
  // Empty interrupt handler
}


int getBandgap() {
  // https://forum.arduino.cc/t/low-battery-warning/360639/7

  // https://forum.arduino.cc/t/measurement-of-bandgap-voltage/38215

  // REFS0 off: Selects AREF, internal VRef turned off
  // REFS0 on: Selects AVcc reference, with external cap at AREF
  // MUX3 MUX2 MUX1: Selects 1.1v (VBG)

  // Internal AREF
  /* ADMUX = bit(REFS0) | bit(MUX3) | bit(MUX2) | bit(MUX1); */
  // External AREF
  ADMUX = bit(MUX3) | bit(MUX2) | bit(MUX1);

  ADCSRA |= bit(ADSC);  // start conversion
  while (ADCSRA & bit(ADSC)) {
  }  // wait for conversion to complete
  int results = (((InternalReferenceVoltage * 1024) / ADC) + 5) / 10;
  return results;
}

void displayBandgap(int bandGap) {
  // Uses slow division, don't this function except for debugging

  int voltInt = bandGap / 100;
  int voltFrac = bandGap % 100;

  char voltage[16];
  snprintf(voltage, 16, "%d.%02dv", voltInt, voltFrac);

  lcd.setCursor(2,0);
  lcd.print("Supply voltage:");
  lcd.setCursor(2,1);
  lcd.print("     ");           /* clear the line */
  lcd.setCursor(2,1);
  lcd.print(voltage);

  char uptime[10];
  unsigned long seconds = millis() / 1000; 
  unsigned int hours, mins;

  hours = seconds / 3600;
  seconds %= 3600;
  mins = seconds / 60;
  seconds = seconds % 60;

  if (hours) {
    snprintf(uptime, 10, "%u:%02u:%02lu", hours, mins, seconds);
  } else {
    snprintf(uptime, 10, "%02u:%02lu", mins, seconds);
  }

  lcd.setCursor(2,2);
  lcd.print(uptime);

  displayCount++;
}

// Reference
// Find internal 1.1 reference voltage on AREF pin (external cap needed from AREF to GND)
/* void setup () */
/* { */
/*   ADMUX = bit (REFS0) | bit (REFS1); */
/* } */

/* void loop () { } */


// Measure input voltage
/* void setup( void ) */
/* { */
/*   Serial.begin( 38400 ); */
/*   Serial.println( "\r\n\r\n" ); */

/*   // REFS1 REFS0          --> 0 0 AREF, Internal Vref turned off */
/*   // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG) */
/*   ADMUX = (0<<REFS1) | (0<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0); */
/* } */

/* void loop( void ) */
/* { */
/*   int value; */

/*   // Start a conversion */
/*   ADCSRA |= _BV( ADSC ); */

/*   // Wait for it to complete */
/*   while( ( (ADCSRA & (1<<ADSC)) != 0 ) ); */

/*   // Scale the value */
/*   value = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L; */

/*   Serial.println( value ); */
/*   delay( 1000 ); */
/* } */

/*
/dev/cu.usbmodem1421 - Arduino UNO, right usb port
/dev/cu.usbmodem1411 - Arduino UNO, left usb port
/dev/cu/wchusbserial1420 - Arduino NANO, right usb port
*/

/* Local Variables: */
/* arduino-cli-default-port: "/dev/cu.usbmodem1421" */
/* arduino-cli-default-fqbn: "arduino:avr:uno" */
/* End: */
