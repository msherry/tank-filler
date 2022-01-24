#include <LowPower.h>

const int wakeUpPin = 2;
const int pumpRelayPin = 3;
const int voltageReadPin = A0;

void setup() {
  // put your setup code here, to run once:

  pinMode(wakeUpPin, INPUT_PULLUP);
  pinMode(pumpRelayPin, OUTPUT);
  pinMode(voltageReadPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Either power-down via interrupt and have a low water level trigger said
  // interrupt, or poll every 8 seconds for water level. Interrupt sounds
  // better for power saving.
  
  // https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, LOW);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  detachInterrupt(digitalPinToInterrupt(wakeUpPin));

  // Read sensor here? We were woken up on an interrupt presumably triggered
  // by the sensor, but it's cheap to reread here.

  // Start pump, stop when high water lever sensor trips.

  // Check battery voltage (through voltage divider,
  // see https://forum.arduino.cc/t/low-battery-warning/360639/7 ),
  // do something on low voltage:
  // a) light LED? at 20mA, a 2000-3000 AA battery at low capacity won't last long
  // b) intermittently beep buzzer? Good, but how to keep intermittent operation during powerDown?
  
}

void wakeUp() {
  // Empty interrupt handler
}
