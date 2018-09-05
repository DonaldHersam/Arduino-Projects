#include "SPI.h"

#define RESOLUTION_16BIT (0x01)
#define RESOLUTION_20BIT (0x02)
#define Resolution (0x02)
#define CSPin 53
#define CLOCK_PIN 52
#define MOSI_PIN 51


void setup() {
  pinMode(CSPin, OUTPUT);
  SPI.begin();
  delay(100);
  //DACreset();

}




void DACreset(){
    SPI.end();
    //pinMode(CSPin, OUTPUT);
    digitalWrite(CSPin, LOW);

    //Reset DAC using clk line
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    delay(1);
    digitalWrite(13, HIGH);
    delayMicroseconds(240); //First high period (600 clocks)
    digitalWrite(13, LOW);
    delayMicroseconds(5);
    digitalWrite(13, HIGH);
    delayMicroseconds(480); //Second high period (1200 clocks)
    digitalWrite(13, LOW);
    delayMicroseconds(5);
    digitalWrite(13, HIGH);
    delayMicroseconds(960); //Second high period (2400 clocks)
    digitalWrite(13, LOW);
    delay(1);

    SPI.begin();
    SPI.setDataMode(SPI_MODE1);
    SPI.setClockDivider(240);
    SPI.setBitOrder(MSBFIRST);

    //Start Self-Calibration
    SPI.transfer(0x05);

//    if (Resolution == RESOLUTION_16BIT) {
//        SPI.transfer(0x21); //16-bit resolution
//    } else {
        SPI.transfer(0xA1); //20-bit resolution
//    }

    digitalWrite(CSPin, HIGH);

    //Wait 500ms for Self-Calibration to complete
    delay(500);
}







void writeCode(uint32_t code) {
    digitalWrite(CSPin, LOW);
    SPI.transfer(0x40);
    SPI.transfer((code&0x00FF0000)>>16);
    SPI.transfer((code&0x0000FF00)>>8);
    SPI.transfer((code&0x000000FF));
   digitalWrite(CSPin, HIGH);
}








void loop() {
  writeCode(0);
  delay(0x111111);
  writeCode(50000);
  delay(1000);
}
