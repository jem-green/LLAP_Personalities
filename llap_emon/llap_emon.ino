//////////////////////////////////////////////////////////////////////////
// xino RF EnergyMon
//
// Uses the Ciseco LLAPSerial library
// Uses a separated LLAPSerial and Sleeper library
// Uses Voltage code from https://code.google.com/p/tinkerit/wiki/SecretVoltmeter
// Uses the Open Energy Monitor EmonLibrary openenergymonitor.org, Licence GNU GPL V3
// Example by Jeremy Green
//
//////////////////////////////////////////////////////////////////////////


#include <LLAPSerial.h>
#include <Sleeper.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <EmonLib.h>

#define EEPROM_DEVICEID_ADDRESS 500       // Address where the two byte DEVICEID is stored
#define HEARTBEATINTERVAL 60000           // time in ms beween heatbeats so 60 seconds
#define EEPROM_CYCLE_MODE_ADDRESS 502     // Addrees where the single byte CYCLE is stored
#define EEPROM_INTERVAL_ADDRESS 503       // Address where the four byte interval is stored
#define EEPROM_INTERVAL_UNITS_ADDRESS 507  // Address where the single char UNITS is stored
#define EEPROM_WAKE_COUNTER_ADDRESS 508   // Address where the double byte WAKEC is stored
#define EEPROM_RETRY_COUNTER_ADDRESS 510  // Address where the double byte RETRIES is stored

// State model to keep track of where we are

enum class State
{
  Unknown = 0,
  Starting = 1,
  Started = 2,
  Registering = 3,
  Registered = 4,
  Restarting = 5,
  Initiating = 6,
  Initiated = 7,
  Sleeping = 8,
  Updating = 9,
  Woken = 10,
  Messaging = 11
};

//

char deviceId[2];                     // The device Id
byte battc = 9;			                  // Set the initial counter to 9 so will do battery next as count in incremented first
boolean cycling = false;              // Indicated that cycling
int interval = 0;                     // number of intervals in the chosen units T - Miliseconds, S - Seconds, M - Minutes, H - Hours, D - Days
char units = 'X';                     // period units
State deviceState = State::Unknown;   // device state
unsigned long previousMillis = 0;     // timout comparitor
unsigned long timeout = 0;            // could be large 999 days!!!! but unsigned long is still only 92 days
int retry = 0;                        // retry counter for start messages
int retries = 0;                      // The number of started messages to additionally send
int wakeCounter = 0;                  // The number of messages sent before a wake + battery

// Device data, this can be storred in program memory

const String deviceName PROGMEM = "ENERGYMON";  // The Device friendly naem
const String deviceType PROGMEM = "U00000002";  // The user defined device type
const int version PROGMEM = 100;                // LLAP version
const int firmware PROGMEM = 100;               // Manufacturer firmware version
const String serialNumber PROGMEM = "123456";   // Device Serial Number

// Instantiate the Serial, sleeper and Emon classes

LLAPSerial LLAP;
Sleeper sleeper;
EnergyMonitor emon;

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

  deviceState = State::Unknown;

  Serial.begin(115200);

  pinMode(8, OUTPUT);               // initialize pin 8 to control the radio
  digitalWrite(8, HIGH);            // select the radio

  pinMode(4, OUTPUT);               // pin 4 controls the radio sleep
  digitalWrite(4, LOW);             // wake the radio

  delay(450);                       // allow the radio to startup

  loadDeviceId(deviceId);           // Recover the deviceId or defaults to '--'
  LLAP.init(deviceId);              // Initiate the serial communications with a deviceId

  interval = loadInterval();        // Recover the messaging interval default to zero so none
  units = loadIntervalUnits();      // Recover the messaging interval units default to 'X' none
  retries = loadRetries();          // Recover the retries or default to 5
  wakeCounter = loadWakeCounter();  // Recover the wake counter or defaults to 10
  cycling = loadCycleMode();        // Recover the cycle mode 1 = cycling, 0 = not

  int retry = 0;                    // Initialise the retry count for the starting sequence
  LLAP.sendMessage(F("STARTED"));   // This needs to loop 5 times more, but leave this to the looping sequence
  timeout = 100;                    // Set the timeout interval between STARTED waiting for ACK
  previousMillis = millis();        // Set the initial values
  deviceState = State::Starting;    // Set the state so can do the STARTING / ACK sequence

  emon.voltage(0, 268.97, 1.7);  	  // Voltage: input pin, calibration, phase_shift
  emon.current(1, 60.606);       	  // Current: input pin, calibration.

  if (cycling == true)
  {
    // I prefer this to continuing to send started's as goes straight into sleep
    LLAP.sendMessage(F("SLEEPING"));
    deviceState = State::Sleeping;  // Now sleeping
  }

  LLAP.bMsgReceived == false;
  pinMode(13, OUTPUT);            // initialize pin 13 as digital output (LED)
}

// APVER - LLAP version
// BATT - get the battery status
// CHDEVID - Change device ID
// CYCLE - Activate cyclic sleeping, the device will wake every INTVL - Once this command has been issued the device will remain asleep for the majority of the time. Whilst asleep everything is turned off, that includes the radio.
// FVER - Firmware version
// DEVNAME - Device name
// DEVTYPE - Device type
// HELLO - Hello (PING)
// INTVLxxxx - Sets the interval for reading the temperature - 999S would be 999 seconds - T=ms, S=secs, M=mins, H=hours, D=Days - Once the command is sent the device will begin to send readings at the frequency requested. Whilst you are testing it's useful not to yet issue CYCLE, once asleep it's not an easy task to wake it.
// PANID - Change PANID
// REBOOT - Restart the device
// SER - Serial number
// WAKE - Wake from sleep

void loop() {

  switch (deviceState)
  {
    case State::Starting:
      {
        // 9/3/2015 Move stating logic to loop and remove delays
        // as interfeering with the serial interrupts

        if (LLAP.bMsgReceived == true)   // Check if there is a message
        {
          if (strncmp_P(LLAP.sMessage.c_str(), PSTR("ACK"), 3) == 0)
          {
            deviceState = State::Started;
            LLAP.bMsgReceived = false;
          }
        }
        else
        {
          unsigned long currentMillis = millis();
          if (currentMillis - previousMillis >= timeout) {
            LLAP.sendMessage(F("STARTED")); // This needs to loop more
            previousMillis = currentMillis;
            retry = retry + 1;
            if (retry >= retries)
            {
              deviceState = State::Started;
            }
          }
        }
        break;
      }
    case State::Started:
      {
        // Check if the interval is set and begin the process of sending responses
        // A timeout of zero means off as per the specification
        timeout = getIntervalMillis(interval, units);
        deviceState = State::Initiated;
        previousMillis = millis();      // Set the initial values
        break;
      }
    case State::Sleeping:
      {
        // Go into a deep sleep
        // if the sleep is beyond 2^16 = 65536 miliseconds = 1 minute then will need multiple loops
        if (interval > 0) {
          //LLAP.sendMessage(F("AWAKE0")); // Message to say now awake
          unsigned long intervalMillis = getIntervalMillis(interval, units);
          if (intervalMillis > 60000) {
            long iterations = intervalMillis / 60000;
            for (long i = 0; i < iterations; i++)
            {
              sleeper.sleepForaWhile(60000);
              LLAP.sendMessage(F("AWAKE1")); // send back error status
            }
          }
          else {
            sleeper.sleepForaWhile(intervalMillis);
            //LLAP.sendMessage(F("AWAKE2")); // Message to say now awake
          }
          deviceState = State::Updating;
        }
        break;
      }
    case State::Updating:
      {
        //LLAP.sendInt("UPDATING",(int)cycling); // Message to say now awake
        if (cycling == true) {
          battc++;
          if (battc >= wakeCounter) {                       // is it time to send a battery reading
            battc = 0;                                      // restart the counter
            LLAP.sendMessage(F("AWAKE"));                   // Message to say now awake
            LLAP.sendIntWithDP("BATT", int(emon.readVcc()), 3);  // read the battery voltage and send
            deviceState = State::Woken;                      //
            timeout = 100;                                  // Enter the response loop allowing 100 ms to check for interruptions
            previousMillis = millis();                      // Enable monitoring to occur for serial events
          }
          else
          {
        		timeout = getIntervalMillis(interval,units);    // reset the timeout just-in-case
            deviceState = State::Messaging;                 // Now back to messaging
          }
        }
        break;
      }
      default:
        {
          if (LLAP.bMsgReceived == true)   // Check if there is a message
          {
            if (strncmp_P(LLAP.sMessage.c_str(), PSTR("APVER"), 5) == 0) // LLAP version always 1.0
            {
              LLAP.sendIntWithDP("APVER", version, 2);
              LLAP.bMsgReceived = false;
            }
            else if (strncmp_P(LLAP.sMessage.c_str(), PSTR("BATT"), 4) == 0) // Battery voltage
            {
              LLAP.sendIntWithDP("BATT", int(emon.readVcc()), 3);   // read the battery voltage and send
              LLAP.bMsgReceived = false;
            }
            else if (strncmp_P(LLAP.sMessage.c_str(), PSTR("CHDEVID"), 7) == 0) // Change the DEVICEID
            {
              // The specification implies the the DEVICEID doesnt get used until a reboot.
              // So update the EEPROM with the new ID but don't apply until the reboot has occured.

              if (strchr_P(PSTR("-#@?\\*ABCDEFGHIJKLMNOPQRSTUVWXYZ"), LLAP.sMessage.c_str()[7]) != 0 && strchr_P(PSTR("-#@?\\*ABCDEFGHIJKLMNOPQRSTUVWXYZ"), LLAP.sMessage.c_str()[8]) != 0)
              {
                char tempId[2];
                tempId[0] = LLAP.sMessage[7];
                tempId[1] = LLAP.sMessage[8];
                saveDeviceId(tempId);
                LLAP.sendMessage("CHDEVID", tempId); // echo back the message
              }
              else
              {
                LLAP.sendMessage(F("ERRROR001")); // echo beck the instruction
              }
              LLAP.bMsgReceived = false;
            }
            else if (strncmp_P(LLAP.sMessage.c_str(), PSTR("CYCLE"), 5) == 0) // Matches using program memory
            {
              // Enter cycle mode
              cycling = true;
              saveCycleMode(cycling);
              deviceState = State::Sleeping;
              LLAP.sendMessage(F("CYCLE")); // echo beck the instruction
              LLAP.bMsgReceived = false;
            }
            else if (strncmp_P(LLAP.sMessage.c_str(), PSTR("DEVNAME"), 7) == 0) // Device type
            {
              LLAP.sendMessage(deviceName); // echo beck the device friendly name
              LLAP.bMsgReceived = false;
            }
            else if (strncmp_P(LLAP.sMessage.c_str(), PSTR("DEVTYPE"), 7) == 0) // Battery voltage
            {
              LLAP.sendMessage(deviceType); // echo beck the device friendly name
              LLAP.bMsgReceived = false;
            }
            else if (strncmp_P(LLAP.sMessage.c_str(), PSTR("FVER"), 4) == 0) // manufacturer firmware version
            {
              LLAP.sendIntWithDP("FVER", firmware, 2);
              LLAP.bMsgReceived = false;
            }
            else if (strncmp_P(LLAP.sMessage.c_str(), PSTR("HELLO"), 5) == 0) // Wake from the sleep cycle
            {
              // Echo back
              LLAP.sendMessage(F("HELLO")); // echo beck the instruction
              LLAP.bMsgReceived = false;
            }
            else if (strncmp_P(LLAP.sMessage.c_str(), PSTR("INTVL"), 5) == 0) // Specify the messaging interval
            {
              // need to do some better checks here, padding problems etc
              int digit;
              interval = 0;
              for (byte i = 5; i < 8; i++) {
                digit = (int)LLAP.sMessage.c_str()[i] - 48;
                if ((digit < 0) || (digit > 9)) {
                  digit = 0;
                }
                interval = 10 * interval + digit;
              }
              if (strchr_P(PSTR("TSMHDX"), LLAP.sMessage.c_str()[8]) != 0) {
                units = LLAP.sMessage.c_str()[8];
              }
              saveInterval(interval);
              saveIntervalUnits(units);
              timeout = getIntervalMillis(interval, units);
              // Echo back
              LLAP.sendInt("INTVL", interval); // echo back the instruction
              LLAP.bMsgReceived = false;
            }
            else if (strncmp_P(LLAP.sMessage.c_str(), PSTR("REBOOT"), 6) == 0) // Tell arduino to reboot, use the watchdog to do this
            {
              //wdt_enable(WDTO_60MS);
              // wait for the prescaller time to expire
              // without sending the reset signal by using
              // the wdt_reset() method
              //while (1) {}
              asm volatile ( "jmp 0");
              LLAP.sendMessage(F("REBOOT"));
            }
            else if (strncmp_P(LLAP.sMessage.c_str(), PSTR("RETRIES"), 7) == 0) // Set the started retry count
            {
              // need to do some better checks here, padding problems etc
              int digit;
              retries = 0;
              for (byte i = 7; i < 9; i++) {
                digit = (int)LLAP.sMessage.c_str()[i] - 48;
                if ((digit < 0) || (digit > 9)) {
                  digit = 0;
                }
                retries = 10 * retries + digit;
              }
              saveRetries(retries);
              LLAP.sendInt("RETRIES", retries);
              LLAP.bMsgReceived = false;
            }
            else if (strncmp_P(LLAP.sMessage.c_str(), PSTR("SER"), 3) == 0) // Get the device serial number
            {
              LLAP.sendMessage(serialNumber);
              LLAP.bMsgReceived = false;
            }
            else if (strncmp_P(LLAP.sMessage.c_str(), PSTR("WAKEC"), 5) == 0) // Wake from the sleep cycle
            {
              int digit;
              wakeCounter = 0;
              for (byte i = 5; i < 8; i++) {
                digit = (int)LLAP.sMessage.c_str()[i] - 48;
                if ((digit < 0) || (digit > 9)) {
                  digit = 0;
                }
                wakeCounter = 10 * wakeCounter + digit;
              }
              // Store this in the EEPROM
              saveWakeCounter(wakeCounter);
              LLAP.sendInt("WAKEC", wakeCounter); // echo back the instruction
              LLAP.bMsgReceived = false;
            }
            else if (strncmp_P(LLAP.sMessage.c_str(), PSTR("WAKE"), 4) == 0) // Wake from the sleep cycle
            {
              cycling = false;
              // Store this in the EEPROM
              saveCycleMode(cycling);
              LLAP.sendMessage(F("WAKE")); // echo back the instruction
              LLAP.bMsgReceived = false;
              timeout = getIntervalMillis(interval, units);
              deviceState = State::Initiated;
            }
            else
            {
              // Echo back
              LLAP.sendMessage(F("ERRROR999")); // send back error status
              LLAP.bMsgReceived = false;
            }
          }
          else
          {
            // check if a message needs to be sent
            if (timeout > 0) {
              unsigned long currentMillis = millis();
              if (currentMillis - previousMillis >= timeout) {
	              LLAP.sendIntWithDP("PWR", int(readPower()), 3);   // read the power and output as kilowatts PWRA1.234
                previousMillis = currentMillis;

                if (cycling == true){
                  if (deviceState == State::Woken) { // check if cycling
                    LLAP.sendMessage(F("SLEEPING")); // echo beck the instruction
                  }
                  timeout = getIntervalMillis(interval, units); // reset the timeout
                  deviceState = State::Sleeping;
                }
                previousMillis = currentMillis;
              }
            }
          }
          break;
        }
      }
  }

  /**
     readVcc
     Reads the battery/power voltage
     Code from https://code.google.com/p/tinkerit/wiki/SecretVoltmeter
  */
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

  long readPower() {

    emon.calcVI(20,2000);         // Calculate all. No.of half wavelengths (crossings), time-out

    float realPower       = emon.realPower;        //extract Real Power into variable
    //float apparentPower   = emon.apparentPower;    //extract Apparent Power into variable
    //float powerFActor     = emon.powerFactor;      //extract Power Factor into Variable
    //float supplyVoltage   = emon.Vrms;             //extract Vrms into Variable
    //float Irms            = emon.Irms;             //extract Irms into Variable

    return((long)realPower);  // assume that this is in watts

  }

  /**
     getIntervalMillis(int interval, char units)
  */
  unsigned long getIntervalMillis(int interval, char units)
  {
    unsigned long intervalMillis = 0;
    switch (units)
    {
      case 'T':
        {
          intervalMillis = (unsigned long)interval;
          break;
        }
      case 'S':
        {
          intervalMillis = (unsigned long)interval * 1000;
          break;
        }
      case 'M':
        {
          intervalMillis = (unsigned long)interval * 60000;
          break;
        }
      case 'H':
        {
          intervalMillis = (unsigned long)interval * 3600000;
          break;
        }
      case 'D':
        {
          intervalMillis = (unsigned long)interval * 86400000;
          break;
        }
      default:
        {
          intervalMillis = 0;
          break;
        }
    }
    return (intervalMillis);
  }

  /**
     saveDeviceId
     Stores DEVICEID into EEPROM
  */
  void saveDeviceId(char* id) {
    char temp[2];
    EEPROM.get(EEPROM_DEVICEID_ADDRESS, temp[0]);
    EEPROM.get(EEPROM_DEVICEID_ADDRESS + 1, temp[1]);
    if (id[0] != temp[0]) {
      EEPROM.put(EEPROM_DEVICEID_ADDRESS, id[0]);
    }
    if (id[1] != temp[1]) {
      EEPROM.put(EEPROM_DEVICEID_ADDRESS + 1, id[1]);
    }
  }

  /**
     loadDeviceId
     Recovers DEVICEID from EEPROM
  */
  void loadDeviceId(char* id) {
    EEPROM.get(EEPROM_DEVICEID_ADDRESS, id[0]);
    EEPROM.get(EEPROM_DEVICEID_ADDRESS + 1, id[1]);
    if (strchr_P(PSTR("-#@?\\*ABCDEFGHIJKLMNOPQRSTUVWXYZ"), id[0]) == 0 || strchr_P(PSTR("-#@?\\*ABCDEFGHIJKLMNOPQRSTUVWXYZ"), id[1]) == 0) {
      id[0] = id[1] = '-';
      saveDeviceId(id);
    }
  }

  /**
     saveCycleMode
     Stores cycling into EEPROM
  */
  void saveCycleMode(boolean mode) {
    byte temp;
    EEPROM.get(EEPROM_CYCLE_MODE_ADDRESS, temp);
    if (mode == true) {
      if (temp == 0) {
        temp = 1;
        EEPROM.put(EEPROM_CYCLE_MODE_ADDRESS, temp);
      }
    }
    else {
      if (temp == 1) {
        temp = 0;
        EEPROM.put(EEPROM_CYCLE_MODE_ADDRESS, temp);
      }
    }
  }

  /**
     loadCycleMode
     Recovers cycling from EEPROM
  */
  boolean loadCycleMode() {
    boolean mode = false;
    byte temp = 0;
    EEPROM.get(EEPROM_CYCLE_MODE_ADDRESS, temp);
    //LLAP.sendInt("CYCLE", (int)temp); // send back error status
    if (temp == 1)
    {
      mode = true;
    }
    else
    {
      mode = false;
    }
    mode = false;
    return (mode);
  }

  /**
     saveInterval
     Stores interval into EEPROM
  */
  void saveInterval(int interval) {
    int temp;
    EEPROM.get(EEPROM_INTERVAL_ADDRESS, temp);
    if (temp != interval)
    {
      EEPROM.put(EEPROM_INTERVAL_ADDRESS, interval);
    }
  }

  /**
     loadInterval
     Recovers interval from EEPROM
  */
  int loadInterval() {
    int interval;
    EEPROM.get(EEPROM_INTERVAL_ADDRESS, interval);
    //LLAP.sendInt("INTVL", interval); // send back error status
    return (interval);
  }

  /**
     saveIntervalUnits
     Stores interval units into EEPROM
  */
  void saveIntervalUnits(char units) {
    char temp;
    EEPROM.get(EEPROM_INTERVAL_UNITS_ADDRESS, temp);
    if (temp != units) {
      EEPROM.put(EEPROM_INTERVAL_UNITS_ADDRESS, units);
    }
  }

  /**
     loadIntervalUits
     Recovers interval units from EEPROM
  */
  char loadIntervalUnits() {
    char units = 'X';
    EEPROM.get(EEPROM_INTERVAL_UNITS_ADDRESS, units);
    //LLAP.sendInt("UNITS", (int)units); // send back error status
    return (units);
  }

  /**
     saveIntervalMode
     Stores interval into EEPROM
  */
  void saveRetries(int retries) {
    int temp;
    EEPROM.get(EEPROM_RETRY_COUNTER_ADDRESS, temp);
    if (temp != retries) {
      EEPROM.put(EEPROM_RETRY_COUNTER_ADDRESS, retries);
    }
  }

  /**
     loadCycleMode
     Recovers interval from EEPROM
  */
  int loadRetries() {
    int retries = 0;
    EEPROM.get(EEPROM_RETRY_COUNTER_ADDRESS, retries);
    //LLAP.sendInt("RETRY", retries); // send back error status
    if ((retries <= 0) || (retries > 99)) {
      retries = 5;
    }
    return (retries);
  }

  /**
     saveWakeCounter
     Stores wake counter into EEPROM
  */
  void saveWakeCounter(int wakes) {
    int temp;
    EEPROM.get(EEPROM_WAKE_COUNTER_ADDRESS, temp);
    if (temp != wakes)
    {
      EEPROM.put(EEPROM_WAKE_COUNTER_ADDRESS, wakes);
    }
  }

  /**
     loadWakeCounter
     Recovers wake counter from EEPROM
  */
  int loadWakeCounter() {
    int wakes = 0;
    EEPROM.get(EEPROM_WAKE_COUNTER_ADDRESS, wakes);
    //LLAP.sendInt("WAKES", wakes); // send back error status
    if ((wakes <= 0) || (wakes > 999)) {
      wakes = 10;
    }
    return (wakes);
  }
