
#include <LiquidCrystal_I2C.h>
#include <LowPower.h>

const int arefEnablePin = 2;
const int sensorHighInputPin = 3;
const int sensorLowInputPin = 4;
const int pumpEnablePin = 5;


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
  /* lcd.backlight(); */
  lcd.noBacklight();

  // Ensure pump is off before setting pin as an output
  pumpOff();

  pinMode(arefEnablePin, OUTPUT);
  pinMode(pumpEnablePin, OUTPUT);

  // Connect AREF directly to our (unregulated) input voltage
  digitalWrite(arefEnablePin, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Either power-down via interrupt and have a low water level trigger said
  // interrupt, or poll every 8 seconds for water level. Interrupt sounds
  // better for power saving.

  // From https://docs.arduino.cc/learn/electronics/low-power
  int bandGap = getBandgap();

  displayBandgap(bandGap);

  if (bandGap < 300) {
    lowBatteryWarning();
  }

  if (waterLevelLow()) {
    fillTank();
  }

  manualLowPowerMode(1);

  // Read sensor here? We were woken up on an interrupt presumably triggered
  // by the sensor, but it's cheap to reread here.

  // Start pump, stop when high water lever sensor trips.

  // Check battery voltage (through voltage divider,
  // see https://forum.arduino.cc/t/low-battery-warning/360639/7 ),
  // do something on low voltage:
  // a) light LED? at 20mA, a 2000-3000 AA battery at low capacity won't last long
  // b) intermittently beep buzzer? Good, but how to keep intermittent operation during powerDown?

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
  unsigned long secs;
  unsigned int hours, mins;

  secs = displayCount * 8; /* account for the 8s sleeps */
  hours = secs % 3600;
  secs -= hours * 3600;
  mins = secs / 60;
  secs -= mins * 60;

  if (hours) {
    snprintf(uptime, 10, "%d:%d:%02d", hours, mins, secs);
  } else {
    snprintf(uptime, 10, "%d:%02d", mins, secs);
  }

  lcd.setCursor(2,2);
  lcd.print(uptime);

  displayCount++;
}

int waterLevelLow() {
  return 0;
}

void fillTank() {

}

void pumpOn() {
  Serial.println("Pump on");
  digitalWrite(pumpEnablePin, HIGH);
}

void pumpOff() {
  Serial.println("Pump off");
  digitalWrite(pumpEnablePin, LOW);
}

int getBandgap() {
  // https://forum.arduino.cc/t/measurement-of-bandgap-voltage/38215

  // REFS0 of: Selects AREF, internal VRef turned off
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

void lowBatteryWarning() {

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
/dev/cu.usbmodel1421 - Arduino UNO, right usb port
/dev/cu.usbmodel1411 - Arduino UNO, left usb port
/dev/cu/wchusbserial1420 - Arduino NANO, right usb port
*/

/* Local Variables: */
/* arduino-cli-default-port: "/dev/cu/wchusbserial1420" */
/* arduino-cli-default-fqbn: "arduino:avr:nano" */
/* End: */
