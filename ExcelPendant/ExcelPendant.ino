#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <avr/interrupt.h>


Adafruit_SSD1306 display = Adafruit_SSD1306();


  #define BUTTON_A 9
  #define BUTTON_B 6
  #define BUTTON_C 5
  #define LED      13
  #define ButtonFast  A0
  #define ButtonZ  A1
  #define Button1 A2
  #define Button2  A3
  #define xAxis    A4
  #define yAxis   A5
  #define ESTOP   SCK
  #define PWR     MOSI
//  #define LED_PIN 1

  #define VBATPIN A7

  #define maxDelay 45000


  #define CPU_HZ 48000000
  #define TIMER_PRESCALER_DIV 1024

  int isLEDOn = 1;
  volatile int totalTime=0;
  volatile int lastTime=0;

  uint32_t buttonNames[] = {A0,A1,A2,A3};

#if (SSD1306_LCDHEIGHT != 32)
 #error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void setup() {  

  Serial.begin(9600);


  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)

  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
//  display.display();
//  delay(1000);

  // Clear the buffer.
  display.clearDisplay();

  


  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(Button1, INPUT_PULLUP);
  pinMode(Button2, INPUT_PULLUP);
  pinMode(ButtonZ, INPUT_PULLUP);
  pinMode(ButtonFast, INPUT_PULLUP);
  pinMode(xAxis, INPUT);
  pinMode(yAxis, INPUT);
  pinMode(ESTOP, INPUT_PULLUP);
  pinMode(VBATPIN, INPUT);
  pinMode(PWR, OUTPUT);

//  pinMode(LED_PIN, OUTPUT);
  startTimer(1000000);

  
  digitalWrite(PWR, HIGH);
  
  // text display tests
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);

  insertPendant();
  
//  attachInterrupt(digitalPinToInterrupt(pushButton), Sleep, FALLING);

}












void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  Serial.println(TC->COUNT.reg);
  Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    // Write callback here!!!
//    if (isLEDOn==1){digitalWrite(LED_PIN, HIGH);}
//    if (isLEDOn==0){digitalWrite(LED_PIN, LOW);}
    if((totalTime-lastTime)>=maxDelay){digitalWrite(PWR,LOW);}
    
//    int x_val = analogRead(xAxis);
//    int y_val = analogRead(yAxis);
//    if((x_val<=100)&(y_val<=100)){runTester();}

//    Serial.print("Total Time: ");
//    Serial.print(totalTime);
//    Serial.print(", Previous Time: ");
//    Serial.println(lastTime);
//    isLEDOn = !isLEDOn;
  }
}












void BatteryVal(){
    volatile float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3 *(4.06/4.12);  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    measuredvbat -= 3.2; //Min bat voltage
    measuredvbat /= 1; //New max volatge is 1.2
    measuredvbat *= 100; //Convert to a percentage
    display.setTextSize(1);
    display.setCursor(86,0);
    display.print("Battery");
    display.setCursor(91,11);
    display.setTextSize(1);
    display.print(measuredvbat);
    display.setCursor(122,11);
    display.print("%");
    display.display();
}






void drawButtons(){
  //Draw Fast Forward
  drawFastForward();
  
  //Draw Z Axis Symbol
  drawZ();

  //Draw Single Circle
  drawButtonOne();

  //Draw Double Circle
  drawButtonTwo();

  display.display(); // actually display all of the above
}



void drawFastForward(){
  display.drawTriangle(2,5,17,12,2,20,1);
  display.drawTriangle(20,5,35,12,20,20,1);
}

void fillFastForward(){
  display.fillTriangle(2,5,17,12,2,20,1);
  display.fillTriangle(20,5,35,12,20,20,1);
}






void drawZ(){
  display.drawFastHLine(44,5,15,1);
  display.drawFastHLine(44,20,15,1);
  display.drawLine(44,20,59,5,1);
}

void fillZ(){
  display.writeFillRect(42,3,19,19,1);
  display.drawFastHLine(44,5,15,0);
  display.drawFastHLine(44,20,15,0);
  display.drawLine(44,20,59,5,0);
}






void drawButtonOne(){
  display.drawCircle(76,12,7,1);
}

void fillButtonOne(){
  display.fillCircle(76,12,7,1);
}







void drawButtonTwo(){
  display.drawCircle(100,12,7,1);
  display.drawCircle(116,12,7,1);
}

void fillButtonTwo(){
  display.fillCircle(100,12,7,1);
  display.fillCircle(116,12,7,1);
}




void testButton(int num){
  int a = 1;
  
  //while(1){
  int index=0;
  int possibles[4]={0,1,2,3};
  int newArray[3];
  for (int i=0;i<=3;i++){
    if(i!=num){
      newArray[index]=i;
      index++;
    }
  }
  //}

  
  while(a){
    totalTime=millis();
    if((digitalRead(buttonNames[num])==0)&(digitalRead(buttonNames[newArray[0]])==1)&(digitalRead(buttonNames[newArray[1]])==1)&(digitalRead(buttonNames[newArray[2]])==1)){
      a=0;
      
      switch (num){
      
      case (0):
        fillFastForward(); 
        lastTime=totalTime;
        break;

      case (1):
        fillZ();
        lastTime=totalTime;
        break;

      case (2):
        fillButtonOne();
        lastTime=totalTime;
        break;

      case (3):
        fillButtonTwo();
        lastTime=totalTime;
        break;      
      
      }
      display.display();
    }
  }
}




void testKeypad()
{
  for (int i=0; i<=3; i++){
    testButton(i);
  }
}



void drawRightArrow(){
  display.drawRect(37,7,19,12,1);
  display.drawTriangle(55,5,70,12,55,20,1);
  display.drawFastVLine(55,7,12,0);
}

void fillRightArrow(){
  display.writeFillRect(37,7,19,12,1);
  display.fillTriangle(55,5,70,12,55,20,1);
}

void drawLeftArrow(){
  display.drawRect(15,7,19,12,1);
  display.drawTriangle(15,5,15,20,2,12,1);
  display.drawFastVLine(15,7,12,0); 
}

void fillLeftArrow(){
  display.writeFillRect(15,7,19,12,1);
  display.fillTriangle(15,5,15,20,2,12,1);
}

void drawTopArrow(){
  display.drawRect(81,14,12,14,1);
  display.drawTriangle(79,14,94,14,86,1,1);
  display.drawFastHLine(81,14,12,0); 
}

void fillTopArrow(){
  display.writeFillRect(81,14,12,14,1);
  display.fillTriangle(79,14,94,14,86,1,1);
}

void drawBottomArrow(){
  display.drawRect(99,1,12,14,1);
  display.drawTriangle(97,14,112,14,104,27,1);
  display.drawFastHLine(99,14,12,0); 
}

void fillBottomArrow(){
  display.writeFillRect(99,1,12,14,1);
  display.fillTriangle(97,14,112,14,104,27,1);
}







void drawArrows(){

  drawRightArrow();
  drawLeftArrow();
  drawTopArrow();
  drawBottomArrow();
  
  display.display();
}



void testJoystick(){
  
  //Left Test
  int c =1;
  while(c){
    totalTime=millis();
  int x_val = analogRead(xAxis);
  if(x_val <= 470){
    lastTime=totalTime;
    c=0;
    fillLeftArrow();
    display.display();
  }
  }

  //Right Test
  int b = 1;
  while(b){
    totalTime=millis();
  int x_val = analogRead(xAxis);
  if(x_val >= 560){
    lastTime=totalTime;
    b=0;
    fillRightArrow();
    display.display();
  }
  }


  //Top Test
  int d = 1;
  while(d){
    totalTime=millis();
  int y_val = analogRead(yAxis);
  if(y_val >= 560){
    lastTime=totalTime;
    d=0;
    fillTopArrow();
    display.display();
  }
  }

  //Bottom Test
  int e = 1;
  while(e){
    totalTime=millis();
  int y_val = analogRead(yAxis);
  if(y_val <= 470){
    lastTime=totalTime;
    e=0;
    fillBottomArrow();
    display.display();
  }
  }
}



void testESTOP(){
  display.setTextSize(2);
  int currentState = digitalRead(ESTOP);


  if (currentState == 0){
  display.setCursor(0,0);
  display.print("Press\nESTOP");
  display.display();
  int estopPress = 1;
  while(estopPress){
    totalTime=millis();
    int estop_val=digitalRead(ESTOP);
    if(estop_val==1){
      lastTime=totalTime;
      estopPress=0;
      display.clearDisplay();
      display.setCursor(0,0);
      display.setTextSize(3);
      display.print("Passed");
      display.display();
    }
  }
  }



  if (currentState == 1){
  display.setCursor(0,0);
  display.print("Release\nESTOP");
  display.display();
  int estopPress = 1;
  while(estopPress){
    totalTime=millis();
    int estop_val=digitalRead(ESTOP);
    if(estop_val==0){
      lastTime=totalTime;
      estopPress=0;
      display.clearDisplay();
      display.setCursor(0,0);
      display.setTextSize(3);
      display.print("Passed");
      display.display();
    }
  }
  }


  
}



void removePendant(){
  delay(800);
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.print("Remove\nPendant");
  display.display();

  int pendantRemove=1;
  while(pendantRemove){
    totalTime=millis();
    int x_val = analogRead(xAxis);
    int y_val = analogRead(yAxis);
    if((x_val<=100)|(y_val<=100)){
      pendantRemove=0;
      lastTime=totalTime;
    }
  }
}



void insertPendant(){
  display.clearDisplay();
  BatteryVal();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.print("Insert\nPendant");
  display.display();
  int insertPendant=1;
  while(insertPendant){
    totalTime=millis();
    int x_val = analogRead(xAxis);
    int y_val = analogRead(yAxis);
    if((x_val>=400)|(y_val>=400)){
      insertPendant=0;
      lastTime=totalTime;
    }
  }
}


void runTester(){
  display.clearDisplay();
  display.setCursor(0,0);
  drawButtons();
  testKeypad();
  display.clearDisplay();
  drawArrows();
  testJoystick();
  display.clearDisplay();
  testESTOP();
}



void loop() {

//  while(1){
//    int x_axis = analogRead(xAxis);
//    int y_axis = analogRead(yAxis);
//    Serial.print("x: ");
//    Serial.print(x_axis);
//    Serial.print(", y: ");
//    Serial.println(y_axis);
//    delay(50);
//  }
  runTester();
  removePendant();
  delay(1000);
  insertPendant();
}
