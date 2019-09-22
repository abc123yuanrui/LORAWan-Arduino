#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

int EEPROMcount1 = 5;
int EEPROMcount5 = 20;
int measureAry[50];
int measureAryF[15];
float oldLat;
float oldLon;

TinyGPS gps;
SoftwareSerial ss(3, 4); // Arduino TX, RX for GPS

static void smartdelay(unsigned long ms);
static void print_float(float val, float invalid, int len, int prec);
static void print_date(TinyGPS &gps);

// No longer required for LoRaServer but still needed in LMIC.
// Keep it all zero.
// This should be in little endian format.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

// First registrate your device on the Homepage and copy the DEVEUI to the { }.
// Don't forget the commas!
// The entered values is default and only for example.
static const u1_t PROGMEM DEVEUI[8] = { 0x11, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x88 };
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format .
// Registrate your device on the Homepage and copy the DEVKEY to the { }.
// Don't forget the comamas!
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

static uint8_t sendData[12];
static osjob_t sendjob;

const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = { 2, 6, 7 },
};

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// watchdog interrupt *****************************************************
ISR(WDT_vect)
{
  wdt_disable();  // disable watchdog
}

//BEGIN OF METHODES *****************************************************

void setup()
{
  Serial.begin(9600);
  ss.begin(9600);
  Serial.println("Start CoLoMe ****************************************");
  Serial.println("LoRa Frequency Min: " + String(EU868_FREQ_MIN) + ", LoRa Frequency Max: " + String(EU868_FREQ_MAX));
  Serial.println("*****************************************************");
  clearEEPROM();

  os_init();
    // reset MAC state 
    LMIC_reset(); 
    // start joining 
    //LMIC_startJoining(); 
  

  
  //LMIC_reset();

  //smartdelay(10000);
}

void loop()
{
  //Execute run-time jobs from the timer and from the run queues.
  os_runloop_once();
//  
//  //loop the functions below with the rigth timing 
//  float flat, flon;
//  unsigned long age, date, time, chars = 0;
//
//
//  for (int i = 1; i <= 60; i++)
//  {
//    if (i % 60 == 0)
//    {
//      Serial.println("60 min");
//      Serial.println();
//      ss.end();
//      myWatchdogEnable(0b000111);  // 2 seconds
//      ss.begin(9600);
//
//    }
//    else if (i % 15 == 0)
//    {
//      quarteHourFnc();
//      smartdelay(100);
//      ss.end();
//      myWatchdogEnable(0b000111);  // 2 seconds
//      ss.begin(9600);
//    }
//    else if (i % 5 == 0)
//    {
//      fiveMinuteFnc();
//      Serial.println(String(EEPROM.read(EEPROMcount5)) + " 5 minute min");
//      Serial.println(String(EEPROM.read(EEPROMcount5 + 1)) + " 5 minute max");
//      Serial.println(String(EEPROM.read(EEPROMcount5 + 2)) + " 5 minute avg");
//      Serial.println();
//      smartdelay(100);
//      ss.end();
//      myWatchdogEnable(0b000111);  // 2 seconds
//      EEPROMcount5 += 3;
//      if (EEPROMcount5 == 29) {
//        EEPROMcount5 = 20;
//        ss.begin(9600);
//      }
//    }
//    else
//    {
//      everyMinuteFnc();
//      Serial.println("");
//      Serial.println(String(EEPROM.read(EEPROMcount1)) + " 1 minute min");
//      Serial.println(String(EEPROM.read(EEPROMcount1 + 1)) + " 1 minute max");
//      Serial.println(String(EEPROM.read(EEPROMcount1 + 2)) + " 1 minute avg");
//      Serial.println();
//      smartdelay(100);
//      ss.end();
//     
//      EEPROMcount1 += 3;
//      if (EEPROMcount1 == 20) {
//        EEPROMcount1 = 5;
//        ss.begin(9600);
//      }
//    }
//
//    //myWatchdogEnable(0b100001);  // 8 seconds
//    myWatchdogEnable(0b000111);  // 2 seconds
//  }
  
  

}

//GET MIN/MAX/AVG *************************************************
int getMin(int* array, int size) {
  int min = array[0];
  for (int i = 0; i<size; i++) {
    if (array[i] < min) {
      min = array[i];
    }
  }
  return min / 4;
}

int getMax(int* array, int size) {
  int max = array[0];
  for (int i = 0; i<size; i++) {
    if (array[i] > max) {
      max = array[i];
    }
  }
  return max / 4;
}

int getAvg(int* array, int size) {
  int avg;
  for (int i = 0; i<size; i++) {
    avg += array[i];
  }
  return (avg / 4) / size;
}

//CLEAR EEPROM *****************************************************
void clearEEPROM()
{
  for (int i = 0; i < EEPROM.length(); i++) {
    if (EEPROM.read(i) != 0)                     //skip already "empty" addresses
    {
      EEPROM.write(i, 0);                       //write 0 to address i
    }
  }
  Serial.println("EEPROM erased");
}

//GPS Function *****************************************************  
void getGPS()
{
  TinyGPS gps;
  SoftwareSerial ss(3, 4); // Arduino TX, RX ,


  float flat, flon;
  unsigned long age, date, time, chars = 0;

  gps.f_get_position(&flat, &flon, &age);      //retrieves latitude and longitude
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 3);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 10, 3);

  print_date(gps);

  Serial.println();

  smartdelay(1000);
}

//WATCHDOGENABLE *****************************************************
void myWatchdogEnable(const byte interval)
{
  MCUSR = 0;                          // reset various flags
  WDTCSR |= 0b00011000;               // set WDCE (enables configuration mode for 4 clock cycles), WDE(enables system reset on time-out)
  WDTCSR = 0b01000000 | interval;    // set WDIE (enales interrupts for last action before reset), and appropriate delay

  wdt_reset();                        // resets watchdogtimer
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();            // now goes to Sleep and waits for the interrupt
}

//SMARTDELAY *****************************************************
void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
    {
      gps.encode(ss.read());
    }
  } while (millis() - start < ms);
}

//PRINTFLOAT********************************************************
static void print_float(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i<len; ++i)
      Serial.print(' ');
  }
}


//LoRaWAN *****************************************************
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, sendData, sizeof(sendData)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


//EVERY MINUTE ***************************************************
void everyMinuteFnc()
{
  // To-Do: measure for 10 seconds, save min., max. & avg. value to EEPROM (4 times)

  for (int i = 0; i < 50; i++)
  {

    measureAry[i] = analogRead(0);
    smartdelay(25);

  }

  for (int j = 0; j < 50; j++)
  {
    Serial.print(String(measureAry[j]) + " ");
  }

  Serial.println("50 values measured");

  EEPROM.write(EEPROMcount1, getMin(measureAry, 50));
  EEPROM.write(EEPROMcount1 + 1, getMax(measureAry, 50));
  EEPROM.write(EEPROMcount1 + 2, getAvg(measureAry, 50));
  EEPROM.write(30, EEPROMcount1);

  Serial.println("3 values writen to EEPROM");

}

//FIVe MINUTE*************************************************************
void fiveMinuteFnc()
{
  //To-Do: call everyMinuteFnc, read EEPROM, save min., max. & avg. value of the last 5 measures to EEPROM, drop unused values (2 times)
  everyMinuteFnc();

  for (int j = 0; j <= 15; j++) {

    measureAryF[j] = EEPROM.read(j + 5);

  }
  EEPROM.write(EEPROMcount5, getMin(measureAryF, 15));
  EEPROM.write(EEPROMcount5 + 1, getMax(measureAryF, 15));
  EEPROM.write(EEPROMcount5 + 2, getAvg(measureAryF, 15));


}

//QUARTER FUNCTION
void quarteHourFnc()
{

  float flat, flon;
  unsigned long age, date, time, chars = 0;

  //To-Do: call fiveMinuteFnc, build sendpackage (SeSt-ID, timereference, position, 9 measured values, 
  //     send package via LoRa for 0.3125 seconds, clear EEPROM from measured values (3 times)
  fiveMinuteFnc();



  int savedValues[9];

  for (int i = 0; i <= sizeof(savedValues) / sizeof(int); i++)
  {
    savedValues[i] = EEPROM.read(20 + i);
  }


  for (int j = 0; j < sizeof(savedValues) / sizeof(int); j++)
  {
    sendData[j] = savedValues[j];
  }

  gps.f_get_position(&flat, &flon, &age);      //retrieves latitude and longitude

  int gpsOffsetLat = (flat - oldLat) * 100;
  int gpsOffsetLon = (flon - oldLon) * 100;

  oldLat = flat;
  oldLon = flon;

  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 3);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 10, 3);
  Serial.println();
  Serial.println(gpsOffsetLat);
  Serial.println(gpsOffsetLon);
  Serial.println();

  smartdelay(100);


  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
  Serial.println();

}


void everyHourFnc()
{
  //To-Do: call quarterHourFnc, get GPS contact to get new time & check position, if new pos clear EEPROM and write new pos
}
