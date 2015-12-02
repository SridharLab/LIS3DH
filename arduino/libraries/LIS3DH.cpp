/*
  LIS3DH_core_spi.cpp - Library for interacting with the STMicro LIS3DH
                        MEMS digital output motion sensor, ultra low-power
                        high performance 3-axes “nano” accelerometer.
  Created by Craig Wm. Versek, 2014-12-04
 */
#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include "LIS3DH.h"
 
LIS3DH_CoreSPIClass::LIS3DH_CoreSPIClass(const int slaveSelectLowPin,
                                         const int dataReadyLowPin
                                        ){
  _slaveSelectLowPin = slaveSelectLowPin;
  _dataReadyLowPin   = dataReadyLowPin;
}

void LIS3DH_CoreSPIClass::begin(int startup_delay_ms) {
  //wait for the LIS3DH to be ready - it can take a while to charge caps
  delay(startup_delay_ms); 
  // Configure the Arduino pins
  pinMode(_slaveSelectLowPin, OUTPUT);
  pinMode(_dataReadyLowPin, INPUT);
  digitalWrite(_slaveSelectLowPin, HIGH);  //comm. off
  //wake up SPI interface, by sending null byte
  //digitalWrite(_slaveSelectLowPin, LOW);   //comm. off
  //SPI.transfer(0x00);
  //digitalWrite(_slaveSelectLowPin, HIGH);  //comm. off



}

//void LIS3DH_CoreSPIClass::attach_dataReadyLow_interrupt(void (*function)(void)){
//  attachInterrupt(_dataReadyLowPin, function, FALLING);
//}

//void LIS3DH_CoreSPIClass::detach_dataReadyLow_interrupt(){
//  detachInterrupt(_dataReadyLowPin);
//}

byte LIS3DH_CoreSPIClass::_readRegister(int addr) {
  byte opcode,new_addr, data;
  new_addr=byte(addr); 
  opcode = 0b10000000 | new_addr;  //bit0 = 1 -> READ, bit1 = 0 do not increment address
#ifdef SPI_HAS_TRANSACTION
  //gain control of SPI bus
  SPI.beginTransaction(SPISettings(LIS3DH_SPI_CLOCK_SPEED, MSBFIRST, SPI_MODE3));
  //SPI.transfer(0x00);                         //FIXME issue transfer with no slave select to init clock polarity, is this needed for SPI_DATA_MODE 2 & 3
#endif
  digitalWrite(_slaveSelectLowPin, LOW);      //set chip as listener
  SPI.transfer(opcode);                       //send command
  data = SPI.transfer(0x00);
  digitalWrite(_slaveSelectLowPin, HIGH);     //release chip select
#ifdef SPI_HAS_TRANSACTION
  SPI.endTransaction();                       //release the SPI bus
#endif
  return data;
}


int LIS3DH_CoreSPIClass::_writeRegister(int addr, byte value) {
  if(addr > 0 && addr <= LIS3DH::ADDR_MAX){
    byte opcode;
    opcode = byte(addr);// & LIS3DH::ADDR_MASK);         //bit0 = 0 -> WRITE, bit1 = 0 do not increment address
#ifdef SPI_HAS_TRANSACTION
    //gain control of SPI bus
    SPI.beginTransaction(SPISettings(LIS3DH_SPI_CLOCK_SPEED, MSBFIRST, SPI_MODE3));
    //SPI.transfer(0x00);                         //FIXME issue transfer with no slave select to init clock polarity, is this needed for SPI_DATA_MODE 2 & 3
#endif
    digitalWrite(_slaveSelectLowPin, LOW);
    SPI.transfer(opcode);
    SPI.transfer(value);
    digitalWrite(_slaveSelectLowPin, HIGH);
#ifdef SPI_HAS_TRANSACTION
    SPI.endTransaction();                       //release the SPI bus
#endif
    return 0;
  }
  else{ return -1; }          //error, out of range
}

void LIS3DH_CoreSPIClass::configurePowerMode(int power_mode) {
    // Chip turns on and then goes into Power Down mode
    //Need to configure the Power mode and the data rate before the chip response correctly.
    // FIXME to allow for enabling x,y,z axes
    byte HP_value, LP_value; 
    if(power_mode==1) {
        LP_value = 0b00001000;
        HP_value = 0b00000000;
    }
    else if(power_mode==2) {
        LP_value = 0b00000000;
        HP_value = 0b00001000;
    }
    else{
        LP_value = 0b00000000;
        HP_value = 0b00000000;
    }
    //Serial.print("LP: ");
    //Serial.println(LP_value);
    //Serial.print("HP: ");
    //Serial.println(HP_value);
    LIS3DH_CoreSPIClass::_writeRegister(LIS3DH::CTRL_REG1,LP_value);
    LIS3DH_CoreSPIClass::_writeRegister(LIS3DH::CTRL_REG4,HP_value);
}

int LIS3DH_CoreSPIClass::_readAcceleration(int low_addr){
    byte output_l,output_h;
    int16_t output;
    output_l = LIS3DH_CoreSPIClass::_readRegister(low_addr);
    output_h = LIS3DH_CoreSPIClass::_readRegister(low_addr+1);
    output = (output_h<<8 | output_l);
    return output;
}

