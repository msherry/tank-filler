#include <LowPower.h>

#define DEBUG_ENABLE
#include "DebugUtils.h"

const int arefEnablePin = 2;
const int pumpEnablePin = 5;
const int alarmPin = 6;
const int sensorEnablePin = 7;
const int sensorInputPin = A6;

/* This value is unique to each board, or at least each board family. Make a
 * measurement using the included sample program across a .1uF cap connected
 * between AREF and GND, * 1000.
 */
/* Arduino UNO dev board */
const long InternalReferenceVoltage = 1109L;

typedef enum TankState {
  TANK_FULL,
  TANK_PARTIAL,
  TANK_FILLING,
} TankState;

TankState tankState = TANK_FULL;
int waterAtHighLevel = 0, waterAtLowLevel = 0;

void setup() {
  // put your setup code here, to run once:

  // Ensure pump is off before setting pin as an output
  pumpOff();

  // Alarm off
  digitalWrite(alarmPin, LOW);

  pinMode(arefEnablePin, OUTPUT);
  pinMode(sensorInputPin, INPUT);
  pinMode(pumpEnablePin, OUTPUT);
  pinMode(sensorEnablePin, OUTPUT);
  pinMode(alarmPin, OUTPUT);

  // Disconnect AREF from the input voltage until we're ready to read
  digitalWrite(arefEnablePin, LOW);

  Serial.begin(9600);
  DEBUGLN("\n\n\n\nStarted");
}

void loop() {
  // put your main code here, to run repeatedly:

  int bandGap = getBandgap();
  displayBandgap(bandGap);

  if (bandGap < 300) {
    lowBatteryWarning();
  }

  readSensors();
  runStateMachine();

  Serial.flush();
  doSleep(64, 4000);
}

void runStateMachine() {
  printTankState();

  switch (tankState) {
  case TANK_FULL:
    /* beep(1, 500); */
    if (tankFull()) {
      /* beep(1, 100); */
      DEBUGLN("Still full");
    } else {
      /* beep(2, 100); */
      DEBUGLN("Only partially full");
      tankState = TANK_PARTIAL;
    }
    break;
  case TANK_PARTIAL:
    /* beep(2, 500); */
    if (!waterTooLow()) {
      /* beep(1, 100); */
      DEBUGLN("Still partially full");
    } else {
      /* beep(2, 100); */
      DEBUGLN("Too low, pump on");
      tankState = TANK_FILLING;
    }
    break;
  case TANK_FILLING:
    /* beep(3, 500); */
    if (!tankFull()) {
      /* beep(1, 100); */
      DEBUGLN("Still filling, pump still on");
    } else {
      /* beep(2, 100); */
      DEBUGLN("Now full, pump off");
      tankState = TANK_FULL;
    }
    break;
  }

  if (tankState == TANK_FILLING) {
    pumpOn();
  } else {
    pumpOff();
  }
}

void readSensors() {
  // Sets waterAtHighLevel and waterAtLowLevel variables.
  // Values are 1 if the sensor (high or low) detects water, 0 otherwise

  // The sensor is generally off to prevent corrosion
  digitalWrite(sensorEnablePin, HIGH);

  // We use the aref to measure battery voltage, so be sure we're back to
  // normal here first.
  analogReference(DEFAULT);
  delay(250);

  int result;
  for (int i = 0; i<3; i++) {
     result = analogRead(sensorInputPin);
  }
  waterAtHighLevel = result > 120;
  waterAtLowLevel = result > 90;

  char out[64];
  snprintf(out, 64, "Reading:  %d   High: %d  Low: %d",
      result, waterAtHighLevel, waterAtLowLevel);
  DEBUGLN(out);

  digitalWrite(sensorEnablePin, LOW);
}

void doSleep(int powerDownTime, int sleepTime) {
  if (tankState == TANK_FILLING) {
    DEBUG("Sleeping... ");
    delay(sleepTime);
  } else {
    DEBUG("PowerDown... ");
    manualLowPowerMode(powerDownTime);
  }
  DEBUGLN("DONE");
}

int waterTooLow() {
  // Return a 1 if the tank needs water added, 0 otherwise
  return !waterAtLowLevel;
}

int tankFull() {
  // Return a 1 if the tank is full (and we should stop the pump), 0 otherwise
  return waterAtHighLevel;
}

void pumpOn() {
  DEBUGLN("Pump on");
  digitalWrite(pumpEnablePin, HIGH);
}

void pumpOff() {
  DEBUGLN("Pump off");
  digitalWrite(pumpEnablePin, LOW);
}

int isPumpOn() {
  return digitalRead(pumpEnablePin);
}

void lowBatteryWarning() {
  DEBUGLN("Battery low");
  beep(1, 20);
}

void beep(int count, int delay_ms) {
  for (int i=0; i<count; i++) {
    digitalWrite(alarmPin, HIGH);
    delay(6);
    digitalWrite(alarmPin, LOW);
    delay(200);
  }
  delay(delay_ms);
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

void printTankState() {
  DEBUG("Tank state: ");
  switch (tankState) {
  case TANK_FULL:
    DEBUGLN("TANK_FULL");
    break;
  case TANK_PARTIAL:
    DEBUGLN("TANK_PARTIAL");
    break;
  case TANK_FILLING:
    DEBUGLN("TANK_FILLING");
    break;
  default:
    DEBUGLN("TANK_UNKNOWN");
    break;
  }
}

int getBandgap() {
  // https://docs.arduino.cc/learn/electronics/low-power

  // https://forum.arduino.cc/t/low-battery-warning/360639/7

  // https://forum.arduino.cc/t/measurement-of-bandgap-voltage/38215

  // REFS0 off: Selects AREF, internal VRef turned off
  // REFS0 on: Selects AVcc reference, with external cap at AREF
  // MUX3 MUX2 MUX1: Selects 1.1v (VBG)

  // Connect AREF directly to our (unregulated) input voltage
  digitalWrite(arefEnablePin, HIGH);
  delay(50);

  // Internal AREF
  /* ADMUX = bit(REFS0) | bit(MUX3) | bit(MUX2) | bit(MUX1); */
  // External AREF
  ADMUX = bit(MUX3) | bit(MUX2) | bit(MUX1);

  ADCSRA |= bit(ADSC);  // start conversion
  while (ADCSRA & bit(ADSC)) {
  }  // wait for conversion to complete
  int results = (((InternalReferenceVoltage * 1024) / ADC) + 5) / 10;

  // Disconnect AREF from power
  digitalWrite(arefEnablePin, LOW);
  return results;
}

void displayBandgap(int bandGap) {
  // Simple serial display
  DEBUG("Bandgap: ");
  DEBUGLN(bandGap);
  return;
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
/*   DEBUGLN( "\r\n\r\n" ); */

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

/*   DEBUGLN( value ); */
/*   delay( 1000 ); */
/* } */

/*
/dev/cu.usbmodem1421 - Arduino UNO, right usb port
/dev/cu.usbmodem1411 - Arduino UNO, left usb port
/dev/cu.wchusbserial1420 - Arduino NANO, right usb port
/dev/cu.wchusbserial1410 - Arduino NANO, right usb port
*/

/* Local Variables: */
/* arduino-cli-default-port: "/dev/cu.wchusbserial1420" */
/* arduino-cli-default-fqbn: "arduino:avr:nano:cpu=atmega328old" */
/* End: */
