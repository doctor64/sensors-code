/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * Version 2.0 - doctor64
 * 
 * DESCRIPTION
 * Sketch for a "light switch" where you can control light or something 
 * else from both HA controller and a physical switch. Switch is used 
 * as a trigger, not on/off
 * This node also works as a repeader for other nodes
 */ 

// Enable debug prints to serial monitor
//#define MY_DEBUG 
// Enable and select radio type attached
#define MY_RADIO_NRF24
#define MY_RF24_CHANNEL  113
#define MY_NODE_ID 5
//#define MY_RADIO_RFM69

// Enabled repeater feature for this node
#define MY_REPEATER_FEATURE

#include <SPI.h>
#include <MySensors.h>
#include <Bounce2.h>

#define RELAY_PIN1 3  // Arduino Digital I/O pin number for relay 

#define BUTTON_PIN1 4  // Arduino Digital I/O pin number for button 

//#define LED_PIN 6  // Arduino Digital I/O pin number for led 

#define CHILD_ID1 0   // Id of the sensor child

#define RELAY_ON 1   
#define RELAY_OFF 0

Bounce debouncer1 = Bounce();

MyMessage msg1(CHILD_ID1, V_STATUS);

bool initial_state_sent;
int oldValue1 = 0;

void changeState(int chiled, int newState) {
#ifdef MY_DEBUG
  Serial.print("Set state: ");
  Serial.println(newState);
#endif
  saveState(chiled, newState);
  switch(chiled){
    case CHILD_ID1:
      digitalWrite(RELAY_PIN1, newState?0:1); //relay active low
      //digitalWrite(LED_PIN, newState);
      send(msg1.set(newState));
      break;
    default:
      break;
  } 
}
void before() {
  // Setup the button
  pinMode(BUTTON_PIN1, INPUT_PULLUP);
  // Activate internal pull-up
  digitalWrite(BUTTON_PIN1, HIGH);
  
  // After setting up the button, setup debouncer
  debouncer1.attach(BUTTON_PIN1);
  debouncer1.interval(5);
  
  // Make sure relays are off when starting up
  digitalWrite(RELAY_PIN1, 1); //relay active low
  // Then set relay pins in output mode
  pinMode(RELAY_PIN1, OUTPUT);

  debouncer1.update();
  // Get the update value
  oldValue1 = debouncer1.read();
  //digitalWrite(LED_PIN, RELAY_OFF);
  // // Then set relay pins in output mode
  //pinMode(LED_PIN, OUTPUT);
}

void setup()  {  
  // Set relay to last known state (using eeprom storage) 
  int newState = loadState(CHILD_ID1);
  if (newState == 0Xff) {//if eeprom is empty
    newState = RELAY_OFF;
  }
  changeState(CHILD_ID1, newState); 
}
void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("IkeaLampSwitch", "2.0");

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID1, S_BINARY);  
}
void loop() {
  if (!initial_state_sent) {
#ifdef MY_DEBUG    
    Serial.println("Sending initial value");
#endif    
    send(msg1.set(loadState(CHILD_ID1) ? true : false));
    
    initial_state_sent = true;
  }

  debouncer1.update();
  // Get the update value
  int value = debouncer1.read();
  if (value != oldValue1){// && value==0) {
    //toggle switch state
    changeState(CHILD_ID1, loadState(CHILD_ID1) ? RELAY_OFF : RELAY_ON);
      //digitalWrite(RELAY_PIN, state ? RELAY_OFF : RELAY_ON);
      //send(msg.set(state?false:true)); // Send new state and request ack back
  }
  oldValue1 = value;
} 

void receive(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
  if (message.isAck()) {
#ifdef MY_DEBUG
     Serial.println("This is an ack from gateway");
#endif
  }

  if (message.type == V_LIGHT) {
#ifdef MY_DEBUG
    Serial.print("Incoming change for sensor:");
    Serial.print(message.sensor);
    Serial.print(", New status: ");
    Serial.println(message.getBool());
#endif
    // chnage the state of the related sensor
    changeState(message.sensor, message.getBool() ? RELAY_ON : RELAY_OFF);
    // state = message.getBool();
    // switch(message.sensor) {
    //   case CHILD_ID1:
    //     changeState(CHILD_ID1, state ? RELAY_ON : RELAY_OFF);
    //     break;
    //   case CHILD_ID2:
    //     changeState(CHILD_ID2, state ? RELAY_ON : RELAY_OFF);
    //     break;
    //   case CHILD_ID3:
    //     changeState(CHILD_ID3, state ? RELAY_ON : RELAY_OFF);
    //     break;
    //   default:
    //    break;
    // }
     // Change relay state
     
     
     // Write some debug info
    
    
   } 
}
