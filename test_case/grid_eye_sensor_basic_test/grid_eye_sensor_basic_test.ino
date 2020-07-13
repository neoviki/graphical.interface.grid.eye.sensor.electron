#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>   
#include <Adafruit_ST7735.h> 
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

void setup() {
    delay(2000);
    Serial.begin(9600);
    Serial.println(" ## AMG THERMAL CAMERA TEST ##");
    Serial.println("");
    Serial.println("[ status ] Setting up sensor");
    Serial.println("");
    bool ret = amg.begin();
    
    if (!ret) {
        Serial.println("[ error ] Couldn't find AMG sensor");
        while (1);
    }else{
         Serial.println("[ status ] AMG Sensor Found");
    }
    
    
    Serial.println("[ status ] AMG sensor setup successful!! ");
    delay(250); 
}


void display_binary8(uint8_t value)
{

    if(value & 128)
        Serial.print("1");
    else
        Serial.print("0");

    if(value & 64)
        Serial.print("1");
    else
        Serial.print("0");

    if(value & 32)
        Serial.print("1");
    else
        Serial.print("0");

    if(value & 16)
        Serial.print("1");
    else
        Serial.print("0");

    if(value & 8)
        Serial.print("1");
    else
        Serial.print("0");

    if(value & 4)
        Serial.print("1");
    else
        Serial.print("0");

    if(value & 2)
        Serial.print("1");
    else
        Serial.print("0");

    if(value & 1)
        Serial.print("1");
    else
        Serial.print("0");

}

void loop() {
  /* Read 64 pixels  */
  amg.readPixels(pixels);
  uint8_t msb, lsb;
  //for(int i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++){
   /* Print Only Sensor at pixel 40*/
   uint16_t val = 0xF0F0;
   msb = (val >> 8) & 0xFF;
   lsb = val & 0xFF;
   Serial.print("SENSOR_VALUE : [ "); display_binary8(msb);
   Serial.print(" "); display_binary8(msb);
   Serial.print(" ]    [ ");
   Serial.print(pixels[40]);
   Serial.print(" ] ");
   Serial.println("");

   //}
   delay(250); 

}
