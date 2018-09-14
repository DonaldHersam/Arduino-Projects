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


#define TouchProbeLeft 33
#define TouchProbeRight 34

#define TouchDir 0
#define TouchPul 1
int count = 0;
int touchDirection = 1;
int state = touchDirection;

//PID Stuff
double Setpoint, Input, Output;
//double Kp=.5, Ki=0, Kd=0;
double Kp=.27, Ki=0.02, Kd=0.02;

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

  pinMode(TouchProbeLeft, INPUT);
  pinMode(TouchProbeRight, INPUT);

  pinMode(TouchDir,OUTPUT);
  pinMode(TouchPul,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(TouchProbeLeft), ProbeLeftLimit, RISING);
  attachInterrupt(digitalPinToInterrupt(TouchProbeRight), ProbeRightLimit, RISING);
  
  digitalWrite(DIR_ACTUATOR_TRANSLATION,1);
  digitalWrite(screwdriver_brake,1);


  //PID
  Input= myEnc.read();
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
}









void loop() {



//  Input = myEnc.read();
//  Input = Input * 0.06; //Steps to u-meter conversion
//  myPID.Compute();

int myTime = millis();
touchDirection = 1; //Move to the left

while(1){
  TouchProbeMotor(touchDirection);
  if (state != touchDirection){
    
    moveCentimeters(5,touchDirection);
    delay(100);
    Input = myEnc.read();
    Input = Input * 0.06; //Steps to u-meter conversion
    Serial.print(Input);
    
    moveCentimeters(10,touchDirection);
    delay(100);
    Input = myEnc.read();
    Input = Input * 0.06; //Steps to u-meter conversion
    Serial.print(" ");
    Serial.print(Input);
    Serial.print(" ");
    Serial.println(millis()-myTime);
    touchDirection = 1; //Move to the left

  }
}




//  outputMotorLinear = Output/1.5; //outputMotorLinear in units of steps


//  LinearActuator(outputMotorLinear);



//static int lastTime = 0;
//int myTime = millis();
//if(myTime - lastTime >= 750){
//  Serial.print("Input (microns): ");
//  Serial.println(Input);
//  Serial.print("Output: ");
//  Serial.println(outputMotorLinear);
//  lastTime=myTime;
//}

}



void moveCentimeters(int centimeters,int touchDirection){
  int milliseconds=centimeters*200;
  int startTime = millis();
while (1){
  if((millis()-startTime)<=milliseconds){
    TouchProbeMotor(touchDirection);
  }
  else{
    return;
  }
}
}



void ProbeLeftLimit(){
//  static int lastTime = millis();
//  if((millis()-lastTime)<=100){
//    Serial.println("LEFT!");
    touchDirection = 0;
//    lastTime=millis();
//  }
  }





void ProbeRightLimit(){
//  static int lastTime = millis();
//  if((millis()-lastTime)<=100){
//    Serial.println("RIGHT!");
    touchDirection = 1;
//    lastTime=millis();
//  }
}



//Touch probe motor moves at 5 cm/s with a combined delay of 4 millis
//(1 step/ x millis)*(360 deg/step)*(40 teeth/360 deg)*(2 mm/ tooth) = y mm/ms 
void TouchProbeMotor(int touchDirection){
  if (touchDirection == 0){
    digitalWrite(TouchDir,LOW);
  }
  if (touchDirection == 1){
    digitalWrite(TouchDir,HIGH);
  }

  int delayTouch=2;
  digitalWrite(TouchPul,LOW);
  delay(delayTouch);
  digitalWrite(TouchPul,HIGH);
  delay(delayTouch);
}



void LinearActuator(double motorSpeed){
  
  
  if (motorSpeed > 0){
    digitalWrite(DIR_ACTUATOR_TRANSLATION,0);
  }
  if (motorSpeed < 0){
    digitalWrite(DIR_ACTUATOR_TRANSLATION,1);
  }

  int delay_time = (1/abs(motorSpeed))*15; //motor speed has units of revs per micro second
  if (delay_time <= 125){
    delay_time = 125;
  }

//static int lastTime = 0;
//int myTime = millis();
//if(myTime - lastTime >= 1500){
//  Serial.print("motorSpeed: ");
//  Serial.println(motorSpeed);
//  Serial.print("delay_time: ");
//  Serial.println(delay_time);
//  lastTime=myTime;
//}
  
    digitalWrite(PUL_ACTUATOR_TRANSLATION,LOW);
    delayMicroseconds(delay_time);
    digitalWrite(PUL_ACTUATOR_TRANSLATION,HIGH);
    delayMicroseconds(delay_time);

}







void Screwdriver(){
  digitalWrite(screwdriver_brake,0);  //Turn off brake to DC motor
  digitalWrite(screwdriver_dir,1);  //Screw bolt in
}
