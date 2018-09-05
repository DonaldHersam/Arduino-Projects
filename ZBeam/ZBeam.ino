/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>
#include <PID_v1.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(15,16);
//   avoid using pins with LEDs attached

#define screwdriver_brake 7
#define screwdriver_dir 8

#define PUL_GANTRY_X 10
#define DIR_GANTRY_X 9

#define PUL_ACTUATOR_TRANSLATION 11
#define DIR_ACTUATOR_TRANSLATION 12

#define delay_time 250



double oldPosition  = -999;

//PID Stuff
double Setpoint, Input, Output;
double Kp=.01, Ki=0.001, Kd=0.001;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);



double outputMotorLinear;





void setup() {
  Serial.begin(9600);

  pinMode(PUL_GANTRY_X,OUTPUT);
  pinMode(DIR_GANTRY_X,OUTPUT);

  pinMode(PUL_ACTUATOR_TRANSLATION,OUTPUT);
  pinMode(DIR_ACTUATOR_TRANSLATION,OUTPUT);

  pinMode(screwdriver_brake, OUTPUT);
  pinMode(screwdriver_dir, OUTPUT);

  digitalWrite(DIR_ACTUATOR_TRANSLATION,1);
  digitalWrite(screwdriver_brake,1);


  //PID
  Input= myEnc.read();
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
}









void loop() {


//Screwdriver
//  digitalWrite(screwdriver_brake,0);  //Turn off brake to DC motor
//  digitalWrite(screwdriver_dir,1);  //Screw bolt in
//  delay(1000);
//
//  while(1){
//    digitalWrite(screwdriver_brake,1);
//  }







  //Touch Probe Encoder
  Input = myEnc.read();
  Input *= 0.06; //Steps to u-meter conversion
  myPID.Compute();


  outputMotorLinear = -Output/1.5; //outputMotorLinear in units of steps


//  Serial.print("Input: ");
//  Serial.println(Input);
//  Serial.print("Output: ");
//  Serial.println(outputMotorLinear);
//  delay(100);

//  if (newPosition != oldPosition) {
//    oldPosition = newPosition;
//    
////    Serial.println(newPosition);
//  }


//static int lastTime = 0;
//int myTime = millis();
//if(myTime - lastTime >= 1500){
//  Serial.println(newPosition);
//  lastTime=myTime;
//}





////Linear Actuator
//  static bool dir = 1;
//  
//  
//  static int counter = 0;
// 
//    if (counter>=12000){
////      dir = !dir;
//      if (dir == 0){
//        dir=1;
//      }
//      else{
//        dir=0;
//      }
//      counter=0;
      digitalWrite(DIR_ACTUATOR_TRANSLATION,1);
////      Serial.println(dir);
//    }
//
// 
//

for (int i = 0; i <= outputMotorLinear; i++){
    digitalWrite(PUL_ACTUATOR_TRANSLATION,LOW);
    delayMicroseconds(delay_time);
    digitalWrite(PUL_ACTUATOR_TRANSLATION,HIGH);
    delayMicroseconds(delay_time);
}
//    counter += 1;

  
}

