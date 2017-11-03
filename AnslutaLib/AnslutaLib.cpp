#include "cc2500_REG.h"
#include "cc2500_VAL.h"
#include <SPI.h>

const byte delayA = 1;  //1++ 0-- No delay is also possible
const unsigned int delayB = 20000; // 10000-- 20000++ 15000++     //KRITISCH
const byte delayC = 10;  //255++ 128++ 64--
const byte delayD = 0;  //200++ 128+++ 64+++ 32+++ 8++
const byte delayE = 200;

const boolean DEBUG = true;   //Some simple communication by RS232

void initAnsluta() {
  pinMode(SS,OUTPUT);
  if(DEBUG) {
    //Serial.begin(115200);
    Serial.println("Debug mode");
    Serial.print("Initialisation");
  }
  SPI.begin();
  SPI.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0));    //Faster SPI mode, maximal speed for the CC2500 without the need for extra delays
  digitalWrite(SS,HIGH);
  SendStrobe(CC2500_SRES); //0x30 SRES Reset chip.
  init_CC2500();
  //  SendStrobe(CC2500_SPWD); //Enter power down mode    -   Not used in the prototype
  WriteReg(0x3E,0xFF);  //Maximum transmit power - write 0xFF to 0x3E (PATABLE)
  if(DEBUG) {
    Serial.println(" - Done");
  }
}

void ReadAddressBytes() {     //Read Address Bytes From a remote by sniffing its packets wireless
   byte tries=0;
   boolean AddressFound = false;

   if(DEBUG) {
    Serial.print("Listening for an Address");
   }
   
   while((tries<200)&&(!AddressFound)) { //Try to listen for the address 200 times
      if(DEBUG){
        Serial.print(".");
      }
      SendStrobe(CC2500_SRX);
      WriteReg(REG_IOCFG1,0x01);   // Switch MISO to output if a packet has been received or not
      delay(20);
      if (digitalRead(MISO)) {      
        byte PacketLength = ReadReg(CC2500_FIFO);
        byte recvPacket[PacketLength];
        if(DEBUG) {
          Serial.println();
          Serial.print("Packet received: ");
          Serial.print(PacketLength,DEC);
          Serial.println(" bytes");
        }
        
        if(PacketLength <= 8) {                       //A packet from the remote cant be longer than 8 bytes
          for(byte i = 1; i <= PacketLength; i++) {   //Read the received data from CC2500
            recvPacket[i] = ReadReg(CC2500_FIFO);
            if(DEBUG){
              if(recvPacket[i]<0x10){Serial.print("0");}
              Serial.print(recvPacket[i],HEX);
            }
          }
          if(DEBUG) {
            Serial.println();
          }
         }
          
          byte start=0;
          while((recvPacket[start]!=0x55) && (start < PacketLength)) {   //Search for the start of the sequence
            start++;
          }
          if(recvPacket[start+1]==0x01 && recvPacket[start+5]==0xAA) {   //If the bytes match an Ikea remote sequence
            AddressFound = true;
            AddressByteA = recvPacket[start+2];                // Extract the addressbytes
            AddressByteB = recvPacket[start+3];
            RecvCommand = recvPacket[start+4];
            if(DEBUG) {
              Serial.print("Address Bytes found: ");
              if(AddressByteA<0x10){Serial.print("0");}
              Serial.print(AddressByteA,HEX);
              if(AddressByteB<0x10){Serial.print("0");}
              Serial.println(AddressByteB,HEX);
              if(RecvCommand<0x10){Serial.print("0");}
              Serial.println(RecvCommand,HEX);
              if(RecvCommand==Light_OFF){Serial.println("Command:OFF");}
              if(RecvCommand==Light_ON_50){Serial.println("Command:50%");}
              if(RecvCommand==Light_ON_100){Serial.println("Command:100%");}
              if(RecvCommand==Light_PAIR){Serial.println("Command:Pair");}
            }
          } 
        SendStrobe(CC2500_SIDLE);      // Needed to flush RX FIFO
        SendStrobe(CC2500_SFRX);       // Flush RX FIFO
      } 
      tries++;  //Another try has passed
   }
   if(DEBUG) {
    Serial.println(" - Done");
   }
}

byte ReadReg(byte addr) {
  addr = addr + 0x80;
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) { };
  byte x = SPI.transfer(addr);
  delay(10);
  byte y = SPI.transfer(0);
  digitalWrite(SS,HIGH);
  return y;  
}

void SendStrobe(byte strobe) {
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) { };
  SPI.transfer(strobe);
  digitalWrite(SS,HIGH);
  delayMicroseconds(delayB);
}

void SendCommand(byte AddressByteA, byte AddressByteB, byte Command) {
    if(DEBUG) {
      Serial.print("Send command ");
      Serial.print(Command,HEX);
      Serial.print(" to ");
      if(AddressByteA<0x10){Serial.print("0");}  //Print leading zero
      Serial.print(AddressByteA,HEX);
      if(AddressByteB<0x10){Serial.print("0");}
      Serial.print(AddressByteB,HEX);
    }
    for(byte i=0;i<50;i++) {       //Send 50 times
      Serial.print(".");
      SendStrobe(CC2500_SIDLE);   //0x36 SIDLE Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
      SendStrobe(CC2500_SFTX);    //0x3B SFTX Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states.
      digitalWrite(SS,LOW);
      while (digitalRead(MISO) == HIGH) { };  //Wait untill MISO high
      SPI.transfer(0x7F);
      delayMicroseconds(delayA);
      SPI.transfer(0x06);
      delayMicroseconds(delayA);
      SPI.transfer(0x55);
      delayMicroseconds(delayA);
      SPI.transfer(0x01);                 
      delayMicroseconds(delayA);
      SPI.transfer(AddressByteA);                 //Address Byte A
      delayMicroseconds(delayA);
      SPI.transfer(AddressByteB);                 //Address Byte B
      delayMicroseconds(delayA);
      SPI.transfer(Command);                      //Command 0x01=Light OFF 0x02=50% 0x03=100% 0xFF=Pairing
      delayMicroseconds(delayA);
      SPI.transfer(0xAA);
      delayMicroseconds(delayA);
      SPI.transfer(0xFF);
      digitalWrite(SS,HIGH);
      SendStrobe(CC2500_STX);                 //0x35 STX In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled: Only go to TX if channel is clear
      delayMicroseconds(delayC);      //Longer delay for transmitting
    }
    if(DEBUG) {
      Serial.println(" - Done");
    }
}

void WriteReg(byte addr, byte value) {
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
    };
  SPI.transfer(addr);
  delayMicroseconds(delayE);
  SPI.transfer(value);
  digitalWrite(SS,HIGH);
  delayMicroseconds(delayD);
}

void init_CC2500() {
  WriteReg(REG_IOCFG2,VAL_IOCFG2);
  WriteReg(REG_IOCFG0,VAL_IOCFG0);
  WriteReg(REG_PKTLEN,VAL_PKTLEN);
  WriteReg(REG_PKTCTRL1,VAL_PKTCTRL1);
  WriteReg(REG_PKTCTRL0,VAL_PKTCTRL0);
  WriteReg(REG_ADDR,VAL_ADDR);
  WriteReg(REG_CHANNR,VAL_CHANNR);
  WriteReg(REG_FSCTRL1,VAL_FSCTRL1);
  WriteReg(REG_FSCTRL0,VAL_FSCTRL0);
  WriteReg(REG_FREQ2,VAL_FREQ2);
  WriteReg(REG_FREQ1,VAL_FREQ1);
  WriteReg(REG_FREQ0,VAL_FREQ0);
  WriteReg(REG_MDMCFG4,VAL_MDMCFG4);
  WriteReg(REG_MDMCFG3,VAL_MDMCFG3);
  WriteReg(REG_MDMCFG2,VAL_MDMCFG2);
  WriteReg(REG_MDMCFG1,VAL_MDMCFG1);
  WriteReg(REG_MDMCFG0,VAL_MDMCFG0);
  WriteReg(REG_DEVIATN,VAL_DEVIATN);
  WriteReg(REG_MCSM2,VAL_MCSM2);
  WriteReg(REG_MCSM1,VAL_MCSM1);
  WriteReg(REG_MCSM0,VAL_MCSM0);
  WriteReg(REG_FOCCFG,VAL_FOCCFG);
  WriteReg(REG_BSCFG,VAL_BSCFG);
  WriteReg(REG_AGCCTRL2,VAL_AGCCTRL2);
  WriteReg(REG_AGCCTRL1,VAL_AGCCTRL1);
  WriteReg(REG_AGCCTRL0,VAL_AGCCTRL0);
  WriteReg(REG_WOREVT1,VAL_WOREVT1);
  WriteReg(REG_WOREVT0,VAL_WOREVT0);
  WriteReg(REG_WORCTRL,VAL_WORCTRL);
  WriteReg(REG_FREND1,VAL_FREND1);
  WriteReg(REG_FREND0,VAL_FREND0);
  WriteReg(REG_FSCAL3,VAL_FSCAL3);
  WriteReg(REG_FSCAL2,VAL_FSCAL2);
  WriteReg(REG_FSCAL1,VAL_FSCAL1);
  WriteReg(REG_FSCAL0,VAL_FSCAL0);
  WriteReg(REG_RCCTRL1,VAL_RCCTRL1);
  WriteReg(REG_RCCTRL0,VAL_RCCTRL0);
  WriteReg(REG_FSTEST,VAL_FSTEST);
  WriteReg(REG_TEST2,VAL_TEST2);
  WriteReg(REG_TEST1,VAL_TEST1);
  WriteReg(REG_TEST0,VAL_TEST0);
  WriteReg(REG_DAFUQ,VAL_DAFUQ);
}
