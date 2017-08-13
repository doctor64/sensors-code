/**
 * DESCRIPTION
 * Basic sensor for DevDuino4 board
 *
 */

// Enable debug prints to serial monitor
//#define MY_DEBUG 
//#define DEBUG_PRINT
#define MY_SPECIAL_DEBUG
#define MY_BAUD_RATE 57600

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
#define MY_RF24_CE_PIN 8
#define MY_RF24_CS_PIN 7
//#define MY_RF24_PA_LEVEL RF24_PA_LOW
#define MY_RF24_CHANNEL  113
#define MY_NODE_ID 2

// Pin definitions
#define DUINO4_LED 9
#define DUINO4_BUTTON 4

// How many milli seconds between each measurement
#define MEASURE_INTERVAL 60000
#define HEARTBEAT_INTERVAL 30
float TEMP_CAL = 0.6;
int HUM_CAL = 9;

// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is forced to report all values to the controller
#define FORCE_TRANSMIT_INTERVAL 30 

#define SKETCH_NAME "devDuino4 basic Sensor"
#define SKETCH_VERSION "1.7"

#include <SPI.h>
#include <MySensors.h>  
#include <Wire.h>
#include "SparkFunHTU21D.h"
#include <Bounce2.h>
#include <RunningAverage.h>

#define TEMP0_HTU_CHILD 0
#define HUM0_HTU_CHILD 1
#define BATT_CHILD 2
#define LED_CHILD 3
#define BUTTON_CHILD 4

HTU21D htu;
Bounce debouncer = Bounce(); 

//float lastTemp_htu = -1;
//float lastHum = -1;

bool metric;
// LED blinks during data transmission. Greater battery energy consumption!
//#define LED_BLINK_WAIT_TRANSMIT  //turn off for external use (Street)

#define TEMP_TRANSMIT_THRESHOLD 0.5
#define HUMI_TRANSMIT_THRESHOLD 1//0.5

MyMessage tempHTUMsg(TEMP0_HTU_CHILD, V_TEMP);
MyMessage humHTUMsg(HUM0_HTU_CHILD, V_HUM);
//MyMessage buttonMsg(BUTTON_CHILD, V_STATUS);
MyMessage ledMsg(LED_CHILD, V_STATUS);
#ifdef BATT_CHILD
MyMessage msgBatt(BATT_CHILD, V_VOLTAGE);
#endif

//#define LED_ON 1  // GPIO value to write to turn on attached relay
//#define LED_OFF 0 // GPIO value to write to turn off attached relay

// Global settings
int measureCount = 0;
int measureCountTemp = 0;
int measureCountHum = 0;

//boolean ota_enabled = false; 
int sendBattery = 0;
boolean highfreq = true;
boolean transmission_occured = false;

// Storage of old measurements
float lastTemperature = 0;
int lastHumidity = 0;
long lastBattery = 0;
float sendVCC; 
#define AVERAGES 5
RunningAverage raHum(AVERAGES);
int cycles;

void setup() 
{
  Serial.begin(MY_BAUD_RATE);
  htu.begin();
  // initialize digital pin 9 as an output.
  pinMode(DUINO4_LED, OUTPUT);
  // Setup the button
  pinMode(DUINO4_BUTTON,INPUT);
  // Activate internal pull-up
  digitalWrite(DUINO4_BUTTON,HIGH);
  
  // After setting up the button, setup debouncer
  debouncer.attach(DUINO4_BUTTON);
  debouncer.interval(5);

  debouncer.update();
  // Get the update value
  int value = debouncer.read();
  if (value==0) {
    Serial.println("Started clearing. Please wait...");
    for (int i=0; i<EEPROM_LOCAL_CONFIG_ADDRESS; i++) {
      hwWriteConfig(i,0xFF);
      if (i%16==0) {   
        digitalWrite(DUINO4_LED, HIGH);
        wait(50);
        digitalWrite(DUINO4_LED, LOW);
        wait(50);
      }
    }
    Serial.println("Clearing done. You're ready to go!");
    void(* resetFunc) (void) = 0;//declare reset function at address 0
    resetFunc(); //call reset 
  }
    
  metric = getControllerConfig().isMetric;
  raHum.clear();
  sendTempHumidityMeasurements(true);
  sendBattLevel(true);
  
  digitalWrite(DUINO4_LED, HIGH);
  wait(500);
  send(msgBatt.set(sendVCC,2));
  digitalWrite(DUINO4_LED, LOW);
  cycles=1;
	
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);   

  // Register sensors to gw (they will be created as child devices)
  present(TEMP0_HTU_CHILD, S_TEMP, "Temperature");
  present(HUM0_HTU_CHILD, S_HUM, "Humidity");
  //present(BUTTON_CHILD, S_BINARY, "Button");
  present(LED_CHILD, S_BINARY,"LED");
#ifdef BATT_CHILD
   present(BATT_CHILD, S_MULTIMETER, "Batt Volt");
#endif    
}  

void loop() 
{
  measureCount ++;
  measureCountTemp ++;
  measureCountHum ++;

  sendBattery ++;
  
  bool forceTransmit = false;
  transmission_occured = false;
  #ifndef DEBUG_PRINT
  if ((measureCount == 3) && highfreq) 
  {
    clock_prescale_set(clock_div_8); // Switch to 1Mhz for the reminder of the sketch, save power.
    highfreq = false;
  } 
  #endif
  
  if (measureCount > FORCE_TRANSMIT_INTERVAL) { // force a transmission
    forceTransmit = true; 
    measureCount = 0;
  }
    
  sendTempHumidityMeasurements(forceTransmit);
  
  if (sendBattery > 40) 
  {
     sendBattLevel(forceTransmit); // Not needed to send battery info that often
     sendBattery = 0;
  }

	if (cycles==HEARTBEAT_INTERVAL) {
    cycles=1;
    smartSleep(MEASURE_INTERVAL);
	}
  else {
    cycles++;
    sleep(MEASURE_INTERVAL);
  }
	//sleep(/*digitalPinToInterrupt(2)*/0, RISING , MEASURE_INTERVAL);
  //wait(MEASURE_INTERVAL);
}
void receive(const MyMessage &message)
{
  // We only expect one type of message from controller. But we better check anyway.
  // And acks are not accepted as control messages
  if (message.type==V_STATUS && message.sensor==LED_CHILD && !mGetAck(message)) {
    // Change relay state
    digitalWrite(DUINO4_LED, message.getBool()?HIGH:LOW);
    // Store state in eeprom
    //saveState(message.sensor, message.getBool());
    // Write some debug info
    #ifdef DEBUG_PRINT
    Serial.print("Incoming change for led:");
    Serial.print(message.sensor);
    Serial.print(", New status: ");
    Serial.println(message.getBool());
    #endif
  }
}

/*********************************************
 * Sends temperature and humidity sensor
 *
 * Parameters
 * - force : Forces transmission of a value (even if it's the same as previous measurement)
 *********************************************/
void sendTempHumidityMeasurements(bool force)
{
  bool tx = force;
  bool tx1 = force;

  //get the Temperature and Humidity from the onboard sensor.
  float temp = htu.readTemperature()+TEMP_CAL;
  int humidity = htu.readHumidity()+HUM_CAL;
  //Serial.print("hum:");
  //Serial.println(humidity);
  if (!metric) 
  {
    // Convert to fahrenheit
    temp = temp * 9.0 / 5.0 + 32.0;
  }
  
  raHum.addValue(humidity);
  
  float diffTemp = abs(lastTemperature - temp);
  float diffHum = abs(lastHumidity - raHum.getAverage());

  #ifdef DEBUG_PRINT
  Serial.print(F("TempDiff :"));Serial.println(diffTemp);
  Serial.print(F("HumDiff  :"));Serial.println(diffHum); 
  #endif

  if (diffTemp > TEMP_TRANSMIT_THRESHOLD) tx = true;
  if (tx) {
    measureCountTemp = 0;  
    // LED 
  #ifdef LED_BLINK_WAIT_TRANSMIT
    digitalWrite(DUINO4_LED, HIGH);      
    send(tempHTUMsg.set(temp,1));
    digitalWrite(DUINO4_LED, LOW);
  }
  #else
    send(tempHTUMsg.set(temp,1));
  }
  #endif  
  
  if (isnan(diffHum)) tx1 = true; 
  if (diffHum > HUMI_TRANSMIT_THRESHOLD) tx1 = true;

  if (tx1) {
     measureCountHum = 0;
     // LED 
     #ifdef LED_BLINK_WAIT_TRANSMIT
     digitalWrite(DUINO4_LED, HIGH);      
     send(humHTUMsg.set(humidity));
     digitalWrite(DUINO4_LED, LOW);
     #else
     send(humHTUMsg.set(humidity));
     #endif 
  }  
  #ifdef DEBUG_PRINT
  Serial.print("Temperature HTU = ");
  Serial.print(temp);
  Serial.println(metric ? " *C" : " *F");
  Serial.print("Humidity HTU = ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print("measureCount: ");Serial.println(measureCount);
  Serial.print("sendBattery: ");Serial.println(sendBattery);
  #endif


  lastTemperature = temp;
  lastHumidity = humidity;
  transmission_occured = true;
}
/********************************************
 *
 * Sends battery information (battery percentage)
 *
 * Parameters
 * - force : Forces transmission of a value
 *
 *******************************************/
void sendBattLevel(bool force)
{ 
  if (force) lastBattery = -1;
  long vcc = readVcc();
  sendVCC = vcc/1000.0;

  if (vcc != lastBattery) {
    lastBattery = vcc;

  #ifdef BATT_CHILD
    send(msgBatt.set(sendVCC,2));
  #endif

    // Calculate percentage
    vcc = vcc - 1900; // subtract 1.9V from vcc, as this is the lowest voltage we will operate at
    
    long percent = vcc / 14.0;
    
    #ifdef DEBUG_PRINT
    Serial.print("Batt voltage = ");
    Serial.print(sendVCC);
    Serial.println(" V");
    Serial.print("Batt percentage = ");
    Serial.print(percent);
    Serial.println(" %");
    #endif
    
    sendBatteryLevel(percent);
    transmission_occured = true;
  }
}
/*******************************************
 *
 * Internal battery ADC measuring 
 *
 *******************************************/
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADcdMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
//  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
   result = 1149900L / result; // Calculate Vcc (in mV); 1 149 900 = 1.124*1023*1000
  return result; // Vcc in millivolts
 
}


