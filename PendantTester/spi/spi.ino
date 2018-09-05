/*
Arduino code for the Vertex pendant tester

Donald Hersam
7.6.18
*/


// include the library code:
#include "Wire.h"
#include "Adafruit_LiquidCrystal.h"
#include <SPI.h>
//#include <MCP3008.h>
#include <MCP23S17.h>         //Keypad Port Expander
#include <SparkFun_ADXL345.h> // Accelerometer
#include <EEPROM.h>

#define CLOCK_PIN 13
#define MOSI_PIN 11
#define MISO_PIN 12

int addr = 0;

//Pin names used on the Teensy 3.1
#define IOCS 20    //Keyboard CS
#define JSCS 21    //Joystick CS
#define accelCS 22 //Accelerometer CS
#define ambrCS 23  //LED CS

#define ESTOP 19
#define greenLCD 18
#define blueLCD 17 
#define pushButton 10         //Interrupt line frm push button
#define pwrReset 6            //Output to the MOSFET control line
#define newPendant 5          //Digital input line from push button
#define startTesting 4

#define displayDelay 40
#define delay_time 350


#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);


//Keypad button return values
int button_values[8] = {256,512,1024,2048,4096,8192,16384,32768};


int accel_axis_values[20];

//Initialize variable for interrupt
volatile int buttonValue = 0;


//Variables defined for sw debouncing in ISR
int counter = 0;       // how many times we have seen new value
int reading;           // the current value read from the input pin
int current_state = LOW;    // the debounced input value

// the following variable is a long because the time, measured in milliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 5;         // the last time the output pin was sampled
int debounce_count = 10; // number of millis/samples to consider before declaring a debounced input





// Connect via SPI. Data pin is #7, Clock is #14 and Latch is #9
Adafruit_LiquidCrystal lcd(7,14,9);

//// constructor for the joystick
//MCP3008 adc(CLOCK_PIN, MOSI_PIN, MISO_PIN, JSCS);


//Constructors for the keypad
MCP inputchip (0, 20);             // Instantiate an object called "inputchip" on an MCP23S17 device at address 1
                    // and slave-select on Arduino pin 10
MCP outputchip(0, 20);            // Instantiate an object called "outputchip" on an MCP23S17 device at address 2
                    // and slave-select on Arduino pin 10



ADXL345 adxl = ADXL345(22);           // USE FOR SPI COMMUNICATION, ADXL345(CS_PIN); (accelerometer)









void setup() {


  //digitalWrite(pwrReset,HIGH);


  //Initialize LCD
  lcd.begin(16, 2);
  lcd.setBacklight(LOW);
  lcd.clear();
  




  
  //Chip select pins on the C8069230
  pinMode (IOCS, OUTPUT);
  pinMode (JSCS, OUTPUT);
  pinMode (accelCS, OUTPUT);
  pinMode (ambrCS, OUTPUT);

  //Peripherary Connections
  pinMode (ESTOP, INPUT);
  pinMode (greenLCD, OUTPUT);
  pinMode (blueLCD, OUTPUT);
  pinMode(pushButton, INPUT);
  pinMode(pwrReset, OUTPUT);
  pinMode(newPendant, INPUT);
  pinMode(startTesting, INPUT);


    //Diable all chip selects initially
  digitalWrite(IOCS,HIGH);
  digitalWrite(JSCS,HIGH);
  digitalWrite(accelCS,HIGH);
  digitalWrite(ambrCS,HIGH);


  digitalWrite(greenLCD, HIGH);
  digitalWrite(blueLCD, HIGH);
  

  attachInterrupt(digitalPinToInterrupt(pushButton), powerLatch, FALLING);


  SPI.begin();
  SPI . setClockDivider ( SPI_CLOCK_DIV4 ) ;
  
  Serial.begin(9600);
  




  //ACCELEROMETER
 adxl.powerOn();                     // Power on the ADXL345

adxl.setRangeSetting(4);           // Give the range settings
                                    // Accepted values are 2g, 4g, 8g or 16g
                                    // Higher Values = Wider Measurement Range
                                    // Lower Values = Greater Sensitivity

adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
                                    // Default: Set to 1
                                    // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library 


 
  
}











//ISR
void powerLatch(){
//  if (millis() != time){
//
//    if(buttonValue == current_state && counter > 0)
//    {
//      counter--;
//    }
//    if(buttonValue != current_state)
//    {
//       counter++; 
//    }
//     if(counter <= debounce_count)
//    {
//      counter = 0;
//      current_state = buttonValue;
      //Serial.println("Pressed");
      //delay(1000);
      digitalWrite(pwrReset,LOW);
      lcd.setBacklight(HIGH);
      digitalWrite(greenLCD,HIGH);
      digitalWrite(blueLCD,HIGH);
      //delay(1000);
//      int existPendant = 1;
//      //digitalWrite(greenLCD,LOW);
//      lcd.clear();
//      while (existPendant){
//        lcd.print("Replace Pendant");
//        //delay(400);
//        lcd.setCursor(0,1);
//        lcd.print("Push Restart    ");
//        //delay(400);
//        if(digitalRead(newPendant)==1){
//          existPendant = 0;
//          lcd.setBacklight(LOW);
//          digitalWrite(greenLCD,HIGH);
//        }
 CPU_RESTART;
      }
      //loop();
     
      //initializeTester();
//      setup();
//      testPendant();
      
//    }
//    time = millis();
//
//  }
////  delay(1000);
////}







//PCB LEDs
void greenLED(){
  digitalWrite (ambrCS, LOW);
}

void redLED(){
  digitalWrite (ambrCS, HIGH);
}





//Joystick ADC interface function
uint16_t mcp3008_read ( uint8_t  channel ) {  
  digitalWrite ( JSCS , LOW )  ;
  SPI . transfer ( 0x01 ) ;
  uint8_t  msb  =  SPI . transfer ( 0x80 + ( channel  << 4 ) )    ;
  uint8_t  lsb  =  SPI . transfer ( 0x00 ) ;
  digitalWrite ( JSCS , HIGH )  ;
  return ( ( msb  & 0x03 ) << 8 ) +  lsb ;     
}





//Accelerometer axis tester
void testAxis(int axis, int limit){
  
  int a=1;
  while(a){
  
  uint16_t  joystick_axis  = mcp3008_read ( axis ) ;  

  if (limit == 1){
  if (joystick_axis >= 1010){
    lcd.clear();
    delay(displayDelay);
    lcd.print("PASS");

    greenLED();
  
    
    digitalWrite(blueLCD,HIGH);
    digitalWrite(greenLCD, LOW);
    a=0;
    delay(delay_time);
    redLED();
    digitalWrite(greenLCD, HIGH);
    digitalWrite(blueLCD,LOW);
    }
  }

  if (limit == 0){
  if (joystick_axis <= 10){
    lcd.clear();
    delay(displayDelay);
    lcd.print("PASS");
    greenLED();
    digitalWrite(blueLCD,HIGH);
    digitalWrite(greenLCD, LOW);
    a=0;
    delay(delay_time);
    redLED();
    digitalWrite(greenLCD, HIGH);
    digitalWrite(blueLCD,LOW);
    }
  }

    
  }
}







void testJoystick(){

  uint16_t  x  = mcp3008_read ( 0 ) ; 
  digitalWrite(blueLCD,LOW);
  
  lcd.clear();
  delay(displayDelay);
  lcd.print("Joystick to");
  lcd.setCursor(0,1);
  lcd.print("Right");
  testAxis(0,1);

  lcd.clear();
  delay(displayDelay);
  lcd.print("Joystick to Left");
  testAxis(0,0);

  lcd.clear();
  delay(displayDelay);
  lcd.print("Joystick Forward");
  testAxis(1,1);

  lcd.clear();
  delay(displayDelay);
  lcd.print("Joystick");
  lcd.setCursor(0,1);
  lcd.print("Backward");
  testAxis(1,0);

  lcd.clear();
  delay(displayDelay);
  lcd.print("Joystick");
  lcd.setCursor(0,1);
  lcd.print("Clockwise");
  testAxis(2,1);

  lcd.clear();
  delay(displayDelay);
  lcd.print("Joystick Counter");
  lcd.setCursor(0,1);
  lcd.print("Clockwise");
  testAxis(2,0);
 }






//Keypad button tester
void testButton(int button){

  int g=1;
  while(g){
  uint16_t value = inputchip.digitalRead();  // read the input chip in word-mode, storing the result in "value"

  if (value == (button_values[button]+255)){
    lcd.clear();
    delay(displayDelay);
    lcd.print("PASS");
    greenLED();
    digitalWrite(blueLCD,HIGH);
    digitalWrite(greenLCD, LOW);
    g=0;
    outputchip.digitalWrite(0B0000000000001111111 >> button);
    delay(delay_time);
    outputchip.digitalWrite(0B0000000000011111111);
    redLED();
    digitalWrite(greenLCD, HIGH);
    digitalWrite(blueLCD,LOW);
    }
  }
  
}








void testAccel(int axis, int limit){

  int acc=1;

while(acc){
  // Accelerometer Readings
  int x,y,z;   
  adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z

  if (x >= 255) x =255;
  if (y >= 255) y =255;
  if (z >= 255) z =255;

  x-=255;
  y-=255;
  z-=255;

  x*=-1;
  y*=-1;
  z*=-1;

  int accel_axis;
  
  switch(axis){
  case 0:
    accel_axis = x;
  case 1:
    accel_axis = y;
  case 2:
    accel_axis = z;
  }


  for (int i=0;i<=100;i++){
    int n=i%20;
    accel_axis_values[n] = accel_axis;


    int accel_axis_sum = 0;

    
    for (int leng =0;leng<20;leng++){
      //Serial.println(accel_values[leng]);
      accel_axis_sum += accel_axis_values[leng];

    }
    int accel_axis_avg = accel_axis_sum / 20;


//    Serial.print("x average: ");
//    Serial.println(accel_axis_avg);



    
    if (limit == 135){
    if (accel_axis_avg >= 185){
      lcd.clear();
      delay(displayDelay);
      lcd.print("PASS");
      greenLED();
      digitalWrite(blueLCD,HIGH);
      digitalWrite(greenLCD, LOW);
      acc=0;
      delay(delay_time);
      redLED();
      digitalWrite(greenLCD, HIGH);
      digitalWrite(blueLCD,LOW);
      break;
    }
    }

      if (limit == 1){
      if (accel_axis_avg == 110){
//      digitalWrite(blueLCD,HIGH);
//      digitalWrite(greenLCD, LOW);
      delay(delay_time);
      acc = 0;
//      digitalWrite(greenLCD, HIGH);
//      digitalWrite(blueLCD,LOW);
      break;
    }
    }

    if (limit == 0){
    if (accel_axis_avg >= 130){
      lcd.clear();
      delay(displayDelay);
      lcd.print("PASS");
      greenLED();
      digitalWrite(blueLCD,HIGH);
      digitalWrite(greenLCD, LOW);
      acc=0;
      delay(delay_time);
      redLED();
      digitalWrite(greenLCD, HIGH);
      digitalWrite(blueLCD,LOW);
      break;      
    }
    }
  }
  
  // Output Results to Serial
  /* UNCOMMENT TO VIEW X Y Z ACCELEROMETER VALUES */  
//  Serial.print("x: ");
//  Serial.print(x);  //MIN: 0 (tilt forward)  MAX: 130 (tilt backward)
//  Serial.print(", y: ");
//  Serial.print(y);  //MIN: 0 (tilt left)  MAX: 140 (tilt right)
//  Serial.print(", Z: ");
//  Serial.println(z);  //MIN: 0 (rotate downward)  MAX: 140 (keep upright)



}
}









void testKeypad(){
  digitalWrite(blueLCD,LOW);
  
  inputchip.begin();
  outputchip.begin();
  inputchip.pinMode(0xFF00);     // Use word-write mode to set all of the pins on inputchip to be inputs
  inputchip.pullupMode(0xFF00);  // Use word-write mode to Turn on the internal pull-up resistors.
  inputchip.inputInvert(0xFF00); // Use word-write mode to invert the inputs so that logic 0 is read as HIGH
  outputchip.pinMode(0xFF00);    // Use word-write mode to Set all of the pins on outputchip to be outputs

  outputchip.digitalWrite(0B0000000000011111111);
  

  lcd.clear();
  delay(displayDelay);
  lcd.print("Press key 1");
  testButton(0);

  lcd.clear();
  delay(displayDelay);
  lcd.print("Press key 2");
  testButton(1);

  lcd.clear();
  delay(displayDelay);
  lcd.print("Press key 3");
  testButton(2);

  lcd.clear();
  delay(displayDelay);
  lcd.print("Press key 4");
  testButton(3);

  lcd.clear();
  delay(displayDelay);
  lcd.print("Press key 5");
  testButton(4);

  lcd.clear();
  delay(displayDelay);
  lcd.print("Press key 6");
  testButton(5);

  lcd.clear();
  delay(displayDelay);
  lcd.print("Press key 7");
  testButton(6);

  lcd.clear();
  delay(displayDelay);
  lcd.print("Press key 8");
  testButton(7);
}







void testESTOP(){

//  uint16_t  x  = mcp3008_read ( 0 ) ;
//  uint16_t  y  = mcp3008_read ( 1 ) ;
//  uint16_t  z  = mcp3008_read ( 2 ) ;
//
//  EEPROM.write(addr,x);
//  addr = addr + 1;
//  if (addr == EEPROM.length()) {
//    addr = 0;
//  }
//  EEPROM.write(addr,y);
//  addr = addr + 1;
//  if (addr == EEPROM.length()) {
//    addr = 0;
//  }
//  EEPROM.write(addr,z);
//  addr = addr + 1;
//  if (addr == EEPROM.length()) {
//    addr = 0;
//  }

  lcd.clear();
  delay(displayDelay);
  int estopTrigger = digitalRead(ESTOP);

  if (estopTrigger == 0){
  lcd.print("Press ESTOP");
  int h=1;
  while(h){
  estopTrigger = digitalRead(ESTOP);

  if (estopTrigger == 1){
    lcd.clear();
    lcd.print("PASS");
    greenLED();
    digitalWrite(blueLCD,HIGH);
    digitalWrite(greenLCD, LOW);
    h=0;
    delay(delay_time);
    redLED();
    digitalWrite(greenLCD, HIGH);
    digitalWrite(blueLCD,LOW);
    }
  }
  
  }

  else{
  lcd.print("Release ESTOP");
  int i=1;
  while(i){
  int estopTrigger = digitalRead(ESTOP);

  if (estopTrigger == 0){
    lcd.clear();
    lcd.print("PASS");
    greenLED();
    digitalWrite(blueLCD,HIGH);
    digitalWrite(greenLCD, LOW);
    i=0;
    delay(delay_time);
    digitalWrite(greenLCD, HIGH);
    digitalWrite(blueLCD,LOW);
    }
  }
  
  }
  
}








void accelFunc(){



  
  int accel_delay = 2.5*delay_time;
  
  lcd.clear();
  delay(displayDelay);
  lcd.print("Tilt Forward");
  delay(accel_delay);
  testAccel(0,135);

  lcd.clear();
  delay(displayDelay);
  lcd.print("Tilt Backward");
  testAccel(0,1);
  delay(accel_delay);
  testAccel(0,0);
  
  lcd.clear();
  delay(displayDelay);
  lcd.print("Tilt Right");
  delay(accel_delay);
  testAccel(1,135);
  
  lcd.clear();
  delay(displayDelay);
  lcd.print("Tilt Left");
  //testAccel(1,1);
  delay(accel_delay);
  testAccel(1,0);

//  lcd.clear();
//  lcd.print("Tilt Clockwise");
//  testAccel(2,135);
//  delay(accel_delay);
//
//  lcd.clear();
//  lcd.print("Tilt CCW");
//  testAccel(2,0);
//  delay(delay_time);
}









void displayPassed(){
  digitalWrite(blueLCD,HIGH);
  digitalWrite(greenLCD, LOW);
  digitalWrite(pwrReset,LOW);
  lcd.clear();
  lcd.print("Pendant Passed");
  delay(1000);

  lcd.setCursor(0,1);
  lcd.print("Push Finish     ");
  while(1){
    if (digitalRead(newPendant) == 1){
      powerLatch();
      
    }
    }
}





void initializeTester(){

  digitalWrite(pwrReset,HIGH);
  
  //Initialize LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setBacklight(LOW);


 adxl.powerOn();                     // Power on the ADXL345

adxl.setRangeSetting(4);           // Give the range settings
                                    // Accepted values are 2g, 4g, 8g or 16g
                                    // Higher Values = Wider Measurement Range
                                    // Lower Values = Greater Sensitivity

adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
                                    // Default: Set to 1
                                    // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library 

 testPendant();

}






void testPendant(){

  //KEYPAD
  testKeypad();
  
  //JOYSTICK
  testJoystick();

  //Accelerometer
  accelFunc();

  //ESTOP
  testESTOP();

  displayPassed();

}





void loop() {

  
//  for (int y=0; y<= 512 ;y++){
//    Serial.println(EEPROM.read(y));
//    Serial.println(mcp3008_read ( 0 )) ;
//    delay(500);
//  }
//  while(1);
  
  lcd.setBacklight(HIGH);
  lcd.print("Press Start");
  lcd.setCursor(0,1);
  lcd.print("to Begin");
while(1){
  if(digitalRead(startTesting)==1){
    initializeTester();
  }
  }
}
