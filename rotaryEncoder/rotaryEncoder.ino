/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(16, 15);
//   avoid using pins with LEDs attached

#include <Stepper.h>

#define STEPS 100
Stepper stepper(STEPS, 8, 10);
int val = 0;

void setup() {
  Serial.begin(9600);
  stepper.setSpeed(200);

  pinMode(14,OUTPUT);
}

long oldPosition  = -999;

void loop() {


  while(1){
    stepper.step(val);
    val += 10;
  }



  //Touch Probe Encoder
  long newPosition = myEnc.read();

  
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
//    Serial.println(newPosition);
  }


static int lastTime = 0;
int myTime = millis();
if(myTime - lastTime >= 3000){
  Serial.println(newPosition);
  lastTime=myTime;
}

  
}

