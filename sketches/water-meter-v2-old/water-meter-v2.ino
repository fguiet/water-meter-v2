
/* 
Water meter v2

   Author :             F. Guiet 
   Creation           : 20200627
   Last modification  : 
  
  Version            : 1.0
  History            : 1.0 - First version          
                       2.0 - 20200830 - Remove all delay statement...

  Note:

    Consumption : 36 mA

    Use "Arduino Pro or Pro Mini, 3.3v, 8Mhz" to upload this sketch 

    Check config.h : #define CFG_eu868 1   
  
    See "config.h" in
      - C:\Users\fguie\Documents\Arduino\libraries\arduino-lmic-master\src\lmic
                       
References :  

  //Using https://github.com/matthijskooijman/arduino-lmic
  //But https://github.com/mcci-catena/arduino-lmic should be prefered if enough memory is available (See https://www.thethingsnetwork.org/forum/t/overview-of-lorawan-libraries-howto/24692/2)
  //In our case, I use https://github.com/matthijskooijman/arduino-lmic because I am using an Arduino Pro Mini with limited memory

  Code based on
  - https://github.com/matthijskooijman/arduino-lmic/blob/master/examples/ttn-abp/ttn-abp.ino

  Last version of (see below) as of 2020/08/19
  - https://github.com/matthijskooijman/arduino-lmic

  - Sample code with sleeping stuff : 
    https://www.thethingsnetwork.org/forum/t/lmics-tx-complete-event-takes-20-30-seconds-to-fire/2639
    https://tum-gis-sensor-nodes.readthedocs.io/en/latest/adafruit_32u4_with_display/README.html
    https://github.com/CongducPham/LMIC_low_power/blob/master/Arduino_LoRa_LMIC_ABP_temp/Arduino_LoRa_LMIC_ABP_temp.ino

  - Removing duty cycle limit 
  https://github.com/matthijskooijman/arduino-lmic/issues/121
  https://github.com/matthijskooijman/arduino-lmic/issues/251
  
*/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
//#include <CayenneLPP.h>

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0xA2, 0xE9, 0xB3, 0x2D, 0xE6, 0xCE, 0xA0, 0x73, 0x70, 0xA5, 0x21, 0xAB, 0xCF, 0x51, 0x88, 0x2A };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0xC3, 0x0B, 0xEA, 0xEE, 0xE8, 0xBA, 0x50, 0xA8, 0x31, 0x5B, 0x1E, 0x05, 0x9B, 0xAA, 0xE0, 0xB8 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x260111F4;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//CayenneLPP lpp(51);
char buff[30];

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
//const unsigned TX_INTERVAL = 10;

/************* WATER_METER_STUFF *************/
#define DEBUG 0

const String FIRMWARE_VERSION= "2.6";

const int LED_PIN = 7;
const int INTERRUPT_PIN = 3; //Comes from sensor
//const int SENSOR_PIN = A0; //Comes from sensor

const int VOLTAGE_PIN = A1;

//volatile bool rising = true;
//volatile bool literConsumed = false;
unsigned long literConsumedFromStart = 0;
unsigned int literConsumedCounter = 0;

unsigned long sendTime = millis();
unsigned long idleTime = millis();
unsigned long blinkTime = millis();
unsigned long debouncedTime = millis();
unsigned long ledOnTime = millis();
unsigned long currentTime = millis();

const unsigned long sendInterval = 5UL*1000UL;   //5s
const unsigned long debounceInterval = 150UL;   //150ms
const unsigned long ledOnInterval = 100UL;   //100ms
const unsigned long blinkInterval = 10UL*1000UL;   //10s
//const unsigned long idleInterval = 60UL * 1000UL * 60UL;  //1h (EV_TXCOMPLETE takes 11 minutes to occur!)...Test not passed (2020/08/22)
//const unsigned long idleInterval = 60UL * 1000UL * 15UL;  //15min ...a test first test...Test passed (2020/08/22)
//const unsigned long idleInterval = 60UL * 1000UL * 45UL;  //45min ...a test first test...Test passed (2020/08/22)
const unsigned long idleInterval = 60UL * 1000UL * 30UL;  //30min ...a test first test...Test passed (2020/09/19)

unsigned int cybleDetectionCounter = 0;
const int RISING_THRESHOLD_COUNTER = 5;

bool risingDetected = false;
bool fallingDetected = false;
int ledState = LOW;             // ledState used to set the LED

//To test ... every 5 minutes
//const unsigned long idleInterval = 300000UL;

/*struct Sensor {
    String Name;    
    uint8_t SensorId;
};

#define SENSORS_COUNT 1
Sensor sensors[SENSORS_COUNT];*/

/*void InitSensors() {
  
  sensors[0].Name = "WM";
  sensors[0].SensorId = 19;
}*/

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,    
    .dio = {2, 4, LMIC_UNUSED_PIN}
};

void onEvent (ev_t ev) {
    debug_message(String(os_getTime()), false);
    debug_message(": ", false);
    //Serial.print(os_getTime());
    //Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            debug_message(F("EV_SCAN_TIMEOUT"),true);
            //Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            debug_message(F("EV_BEACON_FOUND"),true);
            //Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            debug_message(F("EV_BEACON_MISSED"), true);
            //Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            debug_message(F("EV_BEACON_TRACKED"), true);
            //Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            debug_message(F("EV_JOINING"), true);
            //Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            debug_message(F("EV_JOINED"), true);
            //Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            debug_message(F("EV_RFU1"), true);
            //Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            debug_message(F("EV_JOIN_FAILED"), true);
            //Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            debug_message(F("EV_REJOIN_FAILED"), true);
            //Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            debug_message(F("EV_TXCOMPLETE (includes waiting for RX windows)"), true);
            debug_message("Time is : ", false);   
            Serial.println(millis());
            //Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              debug_message(F("Received ack"), true);
              //Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              debug_message(F("Received"), false);
              debug_message(String(LMIC.dataLen), false);
              debug_message(F("bytes of payload"), true);
              //Serial.println(F("Received "));
              //Serial.println(LMIC.dataLen);
              //Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            debug_message(F("EV_LOST_TSYNC"), true);
            //Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            debug_message(F("EV_RESET"), true);
            //Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            debug_message(F("EV_RXCOMPLETE"), true);            
            //Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            debug_message("EV_LINK_DEAD", true);
            //Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            debug_message(F("EV_LINK_ALIVE"), true);
            //Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            debug_message(F("Unknown event"), true);
            //Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        debug_message(F("OP_TXRXPEND, not sending"), true);
        //Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        /*LMIC.bands[BAND_MILLI].avail = os_getTime();
        LMIC.bands[BAND_CENTI].avail = os_getTime();
        LMIC.bands[BAND_DECI].avail = os_getTime();*/
      
        // Prepare upstream data transmission at the next possible time.        
        //strlen() searches for that NULL character and counts the number of memory address passed, So it actually counts the number of elements present in the string before the NULL character, here which is 8.
        LMIC_setTxData2(1, buff, strlen(buff), 0);
        debug_message(F("Packet queued"), true);                
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

/*void OnRisingChange() {
   literConsumed = true;          
}*/

void setup_lorawan_system() {
    //Serial.begin(115200);
    //Serial.println(F("Starting"));    

    /*#ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif*/

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Use with Arduino Pro Mini ATmega328P 3.3V 8 MHz
    // Let LMIC compensate for +/- 10% clock error
    // 2020/08/23 : Trying 10% instead of 1%
    // 2020/09/13 : BAck to 1% instead of 1%
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100); 
    
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    //do_send(&sendjob);
}

void setup_water_meter_system() {
  
  analogReference(INTERNAL);

  //To monitor analog input voltage  
  //pinMode(SENSOR_PIN, INPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  pinMode(INTERRUPT_PIN, INPUT);

  //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN),OnRisingChange, RISING);
 
  // Initialize Serial Port
  if (DEBUG) {
    Serial.begin(115200);    
  }
  
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);

  //InitSensors();  
}

void setup() {

    //debug_message(F("Starting..."),true);  
    
    setup_lorawan_system();
    
    setup_water_meter_system();

    debug_message(F("Setup completed, starting..."),true);  
}

void loop() {   
     
    os_runloop_once(); 

    currentTime = millis();
    
    water_meter_core();

    currentTime = millis();
    
    makeLedBlink();
}

void water_meter_core() {
 
  //DEBUG STUFF
  /*int intPinValue = digitalRead(INTERRUPT_PIN);
  debug_message("Interrupt Pin Status :  " + String(intPinValue), true);    
  
  int inputVoltage = analogRead(SENSOR_PIN);  
  debug_message("Input Voltage :  " + String(inputVoltage), true);  

  if (literConsumed) {
    debug_message("Liter consumed :  VRAI", true); 
    literConsumed = false;
  }
  else {
    debug_message("Liter consumed :  FALSE", true);
  }

  delay(200);

  return; */
  //END DEBUG STUFF  

  //Read interrupt PIN every x ms...
  if (currentTime - debouncedTime > debounceInterval || debouncedTime > currentTime) {  
      
    debouncedTime = millis();    
    
    //Read INTERRUPT_PIN
    int intPinValue = digitalRead(INTERRUPT_PIN);
    
    if (intPinValue == HIGH) {

      //cybleDetectionCounter means HIGH has been detected !
      if (cybleDetectionCounter < RISING_THRESHOLD_COUNTER)
        cybleDetectionCounter++;         
    } 

    if (cybleDetectionCounter == RISING_THRESHOLD_COUNTER) {
       risingDetected = true;
    }

    if (intPinValue == LOW) {
      if (cybleDetectionCounter > 0) 
        cybleDetectionCounter--;
    }   

    if (cybleDetectionCounter == 0) {
       fallingDetected = true;
       risingDetected = false; //reset rising as well
    }
  }

  //One liter has been consummed...
  if (risingDetected && fallingDetected) {

      sendTime = millis();

      //Reset falling / rising
      risingDetected = false;
      fallingDetected = false;
      
      literConsumedFromStart++;
      literConsumedCounter++;    

      debug_message(F("Liter consummed detected"), true);
  }

  //Wait 5 secondes before sending another message (in order not to flood the system in case someone is taking
  //a bath and is consumiming a lot of water)
  if (literConsumedCounter > 0 && (currentTime - sendTime > sendInterval)) { 
       
    sendTime = millis();
    idleTime = millis();
    sendMessage(literConsumedCounter);    
    literConsumedCounter=0;
        
    //delay(500); Removed in v2.1
  }  

  //Send battery voltage every hour
  //if ((millis() - idleTime) > idleInterval || idleTime > millis()) {
  if ((currentTime - idleTime > idleInterval) || (idleTime > currentTime)) {
    sendTime = millis();
    idleTime = millis();
    debug_message(F("Sending battery voltage info"), true);       
    sendMessage(0);
  } 
}

void makeLedBlink() {
  //Alive blink!
  if ((currentTime - blinkTime) > blinkInterval || blinkTime > currentTime) {
    
    if (ledState == LOW) {
      ledOnTime = millis();
      ledState = HIGH;
    }
    else {
      if (currentTime - ledOnTime > ledOnInterval) {       
        ledState = LOW;
        blinkTime = millis(); //turn off for x seconds...
      }
    }
    
    digitalWrite(LED_PIN, ledState);    
  }
}

void sendMessage(unsigned int liter) {

  debug_message(F("Sending message"), true);
  debug_message("Time is : ", false);   
  Serial.println(millis());
  
  float voltage = ReadVoltage();

  //Reset buffer
  memset(buff, '\0', sizeof(buff));

  //First sensor id
  strcpy(buff, "19");
  
  //Firmware version
  strcat(buff, " ");
  strcat(buff, FIRMWARE_VERSION.c_str());
  //dtostrf(FIRMWARE_VERSION, 3, 1, buff + strlen(buff));

  char temp[33] = {};

  //Reset Array
  memset(temp, '\0', sizeof(temp));
  
  //Voltage
  strcat(buff, " ");
  dtostrf(voltage, 4, 2, temp);
  strcat(buff, temp);  
  
  //Liter
  //Reset Array
  memset(temp, '\0', sizeof(temp));
  
  strcat(buff, " ");
  String intConvToString = itoa(liter, temp, 10);
  strcat(buff, intConvToString.c_str());
  
  //Liter Consumed From Start 
  
  //Reset Array
  memset(temp, '\0', sizeof(temp));   
  
  String longConvToString = ltoa(literConsumedFromStart, temp, 10);
  strcat(buff, " ");
  strcat(buff, longConvToString.c_str());

  //Maybe not necessary but...anyway...
  //strcat(buff, '0');

  /*literConsumedFromStart = 186000L;
  String test = String(literConsumedFromStart);
  Serial.println(test.length());*/

  //Serial.println(buff);
  //Serial.flush();
  
  /*strcat(buff, " ");
  dtostrf(hum, 5, 1, buff + strlen(buff));
  strcat(buff, " ");
  dtostrf(moist1, 5 ,1, buff + strlen(buff));
  strcat(buff, " ");
  dtostrf(moist2, 5 ,1, buff + strlen(buff));*/
  
  
  /*lpp.reset();
  lpp.addDigitalInput(1,19); //Sensor Id
  lpp.addDigitalInput(2,26); //Firmware version
  lpp.addDigitalInput(3,voltage);
  lpp.addDigitalInput(4,liter);
  lpp.addLuminosity(5,literConsumedFromStart);*/
  
  do_send(&sendjob);  
}

float ReadVoltage() {

  //AnalogRead = 721 pour 4.12v

  //R1 = 33kOhm
  //R2 = 7.5kOhm

  
  //float vin = 0.f;

  delay(100); //Tempo so analog reading will be correct!
  unsigned int sensorValue = analogRead(VOLTAGE_PIN);
  //analog_vcc = sensorValue;
  
  //debug_message("Analog Reading : " + String(sensorValue,2), true);
  //vin = ;
  //debug_message("Voltage Reading : " + String(vin,2), true);
  
  //return String((sensorValue * 4.12) / 210, 2).toInt() *10  ;

  /*Serial.println(sensorValue);  
  Serial.println(String((sensorValue * 4.12) / 721, 2));
  Serial.flush();*/

  return (sensorValue * 4.12) / 721;  
}

void debug_message(String message, bool doReturnLine) {
  if (DEBUG) {
    if (doReturnLine) {
      //mySerial.println(message);
      Serial.println(message);
    }
    else {
      //mySerial.println(message);
      Serial.print(message);
    }

    Serial.flush();    
  }
}
