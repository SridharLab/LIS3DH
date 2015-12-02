/*
  LIS3DH_core_spi.h - Library for interacting with the STMicro LIS3DH
                      MEMS digital output motion sensor, ultra low-power
                      high performance 3-axes “nano” accelerometer.
  Created by Craig Wm. Versek, 2014-12-04
 */

#include <SPI.h>
#include "LIS3DH.h"

//configure the accelerometer chip

//LIS3DH_CoreSPIClass acc(6,   //slaveSelectLowPin
//                        15    //dataReadyLowPin
//                       );
LIS3DH_CoreSPIClass acc(10,   //slaveSelectLowPin
                        6   //dataReadyLowPin
                       );
int k;
void setup() {
  Serial.begin(9600);
  //start up the SPI bus
  SPI.begin();
  SPI.setClockDivider(24); //96MHz clock /24 = 4 MHz
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  
  //SPI.setClockDivider(SPI_CLOCK_DIV21);  //FIXME SPI_CLOCK_DIV these names are not defined anymore in Arduino 1.5.4
  //start controlling the voltage supply
  acc.begin();
  acc.configurePowerMode(2);
  //need to configure data rate & enable x,y,z axes
  acc._writeRegister(32,0b01110111); // FIXME this is only for powerMode 2, HIGH power. bits2:0 enable x,y,z axes. Bit3 will need to be changed.
  delay(1000);
  

}

void loop() {
  delay(1000);
  byte data,who_am_i;
  int x_axis,y_axis,z_axis;
  who_am_i = acc._readRegister(LIS3DH::WHO_AM_I);
  //Serial.println(who_am_i);
  x_axis = acc._readAcceleration(LIS3DH::OUT_X_L);
  y_axis = acc._readAcceleration(LIS3DH::OUT_Y_L);
  z_axis = acc._readAcceleration(LIS3DH::OUT_Z_L);
  Serial.print("X: ");
  Serial.print(x_axis);
  Serial.print("     Y: ");
  Serial.print(y_axis);
  Serial.print("     Z: ");
  Serial.println(z_axis);
//  for (int i = 0; i < LIS3DH::ADDR_MAX; i++){
//    data = acc._readRegister(i);
//    Serial.print(i);
//    Serial.print(": ");
//    Serial.print(data);
//    Serial.print(": ");
//    print_binary(data, 8);
//    Serial.print("\n");
//  }
}

void print_binary(int v, int num_places)
{
    int mask=0, n;

    for (n=1; n<=num_places; n++)
    {
        mask = (mask << 1) | 0x0001;
    }
    v = v & mask;  // truncate v to specified number of places

    while(num_places)
    {

        if (v & (0x0001 << num_places-1))
        {
             Serial.print("1");
        }
        else
        {
             Serial.print("0");
        }

        --num_places;
        if(((num_places%4) == 0) && (num_places != 0))
        {
            Serial.print("_");
        }
    }
}
