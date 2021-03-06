//////////////////////////////////////////////////////////////////////////
// xino RF PIR Sensor
//
// Uses the Ciseco LLAPSerial library
// Uses a separated LLAPSerial and Sleeper library
// Example by Jeremy Green
//
// https://code.google.com/p/tinkerit/wiki/SecretVoltmeter
//////////////////////////////////////////////////////////////////////////


#include <LLAPSerial.h>
#include <Sleeper.h>

#define DEVICEID "AC"
#define PIR_PIN 2

#define WAKEC 10
byte battc = 9;

#define PIRBLOCKTIME 20000          // time in ms to block after a triger

// Instantiate the Serial and sleeper classes

LLAPSerial LLAP;
Sleeper sleeper;

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  LLAP.SerialEvent();
}

void setup() {
  Serial.begin(9600);

  pinMode(8, OUTPUT);             // pin 8 controls the radio
  digitalWrite(8, HIGH);          // select the radio

  pinMode(4, OUTPUT);             // pin 4 controls the radio sleep
  digitalWrite(4, LOW);           // wake the radio

  delay(450);                     // allow the radio to startup
  LLAP.init(DEVICEID);

  pinMode(PIR_PIN, INPUT);        // PIR Input pin
  digitalWrite(PIR_PIN, LOW);     // no pullup

  LLAP.sendMessage(F("STARTED")); // This needs to loop more
}

void loop() {
  pinMode(4, INPUT);                          // sleep the radio

  sleeper.sleep(PIR_PIN, RISING, false);         // deep sleep until PIR causes interupt
  battc++;                                    // increase battery count

  pinMode(4, OUTPUT);                         // wake the radio

  delay(450);                                 // give it time to wake up

  LLAP.sendMessage(F("PIRTRIG"));             // the pir trigered send a message

  if (battc >= WAKEC) {                       // is it time to send a battery reading
    battc = 0;
    LLAP.sendIntWithDP("BATT", int(readVcc()), 3);   // read the battery voltage and send
  }
  pinMode(4, INPUT);                          // sleep the radio again
  sleeper.sleepForaWhile(PIRBLOCKTIME);          // sleep for a little while before we go back to listening for the PIR

}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}
