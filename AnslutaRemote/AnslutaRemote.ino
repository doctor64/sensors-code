
// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
//#define MY_RF24_PA_LEVEL RF24_PA_HIGH
#define MY_RF24_CHANNEL  113

#define MY_SOFTSPI
#define MY_SOFT_SPI_SCK_PIN 14
#define MY_SOFT_SPI_MISO_PIN 15
#define MY_SOFT_SPI_MOSI_PIN 16
#define MY_RF24_CE_PIN 5
#define MY_RF24_CS_PIN 6

#define MY_NODE_ID 6

#define ANSLUTA_ADDRA 0x75
#define ANSLUTA_ADDRB 0xA1

// Enabled repeater feature for this node
//#define MY_REPEATER_FEATURE

#include <SPI.h>
#include <MySensors.h>
#include "AnslutaLib.h"

#define CHILD_ID_LIGHT 0   // Id of the sensor child

#define SN "Ikea Ansluta Light"
#define SV "1.0"

MyMessage lightMsg(CHILD_ID_LIGHT, V_STATUS);
MyMessage dimmerMsg(CHILD_ID_LIGHT, V_DIMMER);


#define LIGHT_OFF 0
#define LIGHT_ON 1

int16_t LastLightState=LIGHT_OFF;
int16_t LastDimValue=100;

#define EPROM_LIGHT_STATE 1
#define EPROM_DIMMER_LEVEL 2



void setup() {
  // put your setup code here, to run once:
  initAnsluta();

}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SN, SV);

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_LIGHT, S_DIMMER);  
}

void loop() {
  // put your main code here, to run repeatedly:

}

void receive(const MyMessage &message)
{
  if (message.type == V_LIGHT) {
    Serial.println( "V_LIGHT command received..." );

    int lstate= atoi( message.data );
    if ((lstate<0)||(lstate>1)) {
      Serial.println( "V_LIGHT data invalid (should be 0/1)" );
      return;
    }
    LastLightState=lstate;
    saveState(EPROM_LIGHT_STATE, LastLightState);

    if ((LastLightState==LIGHT_ON)&&(LastDimValue==0)) {
      //In the case that the Light State = On, but the dimmer value is zero,
      //then something (probably the controller) did something wrong,
      //for the Dim value to 100%
      LastDimValue=100;
      saveState(EPROM_DIMMER_LEVEL, LastDimValue);
    }

    //When receiving a V_LIGHT command we switch the light between OFF and the last received dimmer value
    //This means if you previously set the lights dimmer value to 50%, and turn the light ON
    //it will do so at 50%
  } else if (message.type == V_DIMMER) {
    Serial.println( "V_DIMMER command received..." );
    int dimvalue= atoi( message.data );
    if ((dimvalue<0)||(dimvalue>100)) {
      Serial.println( "V_DIMMER data invalid (should be 0..100)" );
      return;
    }
    if (dimvalue==0) {
      LastLightState=LIGHT_OFF;
    } else {
      LastLightState=LIGHT_ON;
      LastDimValue=dimvalue;
      saveState(EPROM_DIMMER_LEVEL, LastDimValue);
    }
  } else {
    Serial.println( "Invalid command received..." );
    return;
  }

  //Here you set the actual light state/level
  SetCurrentState2Hardware();
}
void SetCurrentState2Hardware()
{
  if (LastLightState==LIGHT_OFF) {
    Serial.println( "Light state: OFF" );
    SendCommand(ANSLUTA_ADDRA, ANSLUTA_ADDRB, Ansluta_Light_OFF);
  } else {
    Serial.print( "Light state: ON, Level: " );
    Serial.println( LastDimValue );
    if (LastDimValue < 60) {
      SendCommand(ANSLUTA_ADDRA, ANSLUTA_ADDRB, Ansluta_Light_ON_50);     
    } else SendCommand(ANSLUTA_ADDRA, ANSLUTA_ADDRB, Ansluta_Light_ON_100);     
  }

  //Send current state to the controller
  SendCurrentState2Controller();
}

void SendCurrentState2Controller()
{
  if ((LastLightState==LIGHT_OFF)||(LastDimValue==0)) {
    send(dimmerMsg.set((int16_t)0));
  } else {
    send(dimmerMsg.set(LastDimValue));
  }
  send(lightMsg.set(LastLightState));
}

