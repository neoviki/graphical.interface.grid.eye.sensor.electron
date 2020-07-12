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

void loop() {
  /* Read 64 pixels  */
  amg.readPixels(pixels);

  //for(int i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++){
   /* Print Only Sensor at pixel 40*/
   Serial.println(pixels[40]);
   //}
   delay(250); 

}
