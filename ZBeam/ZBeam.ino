/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>
#include <PID_v1.h>


Encoder myEnc(15,16);


#define screwdriver_brake 7
#define screwdriver_dir 8

#define PUL_GANTRY_X 10
#define DIR_GANTRY_X 9

#define PUL_ACTUATOR_TRANSLATION 11
#define DIR_ACTUATOR_TRANSLATION 12







//PID Stuff
double Setpoint, Input, Output;
double Kp=.1, Ki=0, Kd=0;

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




  Input = myEnc.read();
  Input *= 0.06; //Steps to u-meter conversion
  myPID.Compute();


  outputMotorLinear = Output/1.5; //outputMotorLinear in units of steps


  LinearActuator(outputMotorLinear);



//static int lastTime = 0;
//int myTime = millis();
//if(myTime - lastTime >= 1500){
//  Serial.print("Input: ");
//  Serial.println(Input);
//  Serial.print("Output: ");
//  Serial.println(outputMotorLinear);
//  lastTime=myTime;
//}

}









void LinearActuator(double motorSpeed){
  
  
  if (motorSpeed > 0){
    digitalWrite(DIR_ACTUATOR_TRANSLATION,0);
  }
  if (motorSpeed < 0){
    digitalWrite(DIR_ACTUATOR_TRANSLATION,1);
  }

  int delay_time = (1/abs(motorSpeed))*150; //motor speed has units of revs per micro second
  if (delay_time <= 250){
    delay_time = 250;
  }

static int lastTime = 0;
int myTime = millis();
if(myTime - lastTime >= 1500){
  Serial.print("motorSpeed: ");
  Serial.println(motorSpeed);
  Serial.print("delay_time: ");
  Serial.println(delay_time);
  lastTime=myTime;
}
  
    digitalWrite(PUL_ACTUATOR_TRANSLATION,LOW);
    delayMicroseconds(delay_time);
    digitalWrite(PUL_ACTUATOR_TRANSLATION,HIGH);
    delayMicroseconds(delay_time);

}







void Screwdriver(){
  digitalWrite(screwdriver_brake,0);  //Turn off brake to DC motor
  digitalWrite(screwdriver_dir,1);  //Screw bolt in
}
