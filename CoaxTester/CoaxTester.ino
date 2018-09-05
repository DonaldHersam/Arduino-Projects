//int CLKPR;
//#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))
//#define CPU_16MHz       0x00
//#define CPU_8MHz        0x01
//#define CPU_4MHz        0x02
//#define CPU_2MHz        0x03
//#define CPU_1MHz        0x04
//#define CPU_500kHz      0x05
//#define CPU_250kHz      0x06
//#define CPU_125kHz      0x07
//#define CPU_62kHz       0x08


#define PIN_CS 31 //enable
#define PIN_CLK 32 //spi clock
#define PIN_DAT 30 //spi data
#define existVector 29
#define existSol 34
#define lightSensor A14 //spi data
#define Restart 6
 
uint32_t Millis = 0;
uint8_t  Buf[10];
uint8_t  Len = 0;
float cal = 1.00015; //linear distortion
float offset = 0.0004; //offset (due to output drivers of DAC1220) -> depends on YOUR circuit! (ground level)
float voltageLimit = 4.99; //upper voltage limit (due to output drivers of DAC1220)
float offAvg, lowAvg, midAvg, highAvg;
float offSol, lowSol, midSol, highSol;
float lightVal;
int delayTime = 180;




void setup() {
//  CPU_PRESCALE(CPU_1MHz);

  
  Serial.begin(9600);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_DAT, OUTPUT);
  pinMode(lightSensor,INPUT);
  pinMode(12,OUTPUT);
  pinMode(Restart,INPUT);
  pinMode(existVector,INPUT);
  pinMode(existSol,INPUT_PULLUP);
  pinMode(A7,OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  reset(PIN_DAT);


  attachInterrupt(digitalPinToInterrupt(Restart), ClearLight, CHANGE);
}


void ClearLight(){
  analogWrite(A7,0);
}

void loop() {

//  while(1){
//    Serial.println(digitalRead(Restart));
//    delay(50);
//  }

  testCoax();

  if(digitalRead(existVector)==0){
  if((offAvg<=135)&(offAvg>=104)){
    if((lowAvg<=24)&(lowAvg>=0)){
      if((midAvg<=548)){
        if((highAvg<=1000)&(highAvg>=970)){
          analogWrite(A7,30);    
          delay(500);
        }
      }
    }
  }
  }





  if(digitalRead(existSol)==0){
  if((offAvg<=135)&(offAvg>=104)){
    if((lowAvg<=24)&(lowAvg>=0)){
      if((midAvg<=24)&(midAvg>=0)){
        if((highAvg<=1000)&(highAvg>=970)){
          analogWrite(A7,30);     
          delay(3500);
          analogWrite(A7,0);
        }
      }
    }
  }
  }
  //digitalWrite(12,LOW);
}


void PCBpass(){
  while(1){
    Serial.println(digitalRead(existVector));
    delay(50);
  }
}

void testCoax(){
  
  reset(PIN_DAT);
  delay(10);
  offAvg = 0;
  for (int i =0; i<= 10;i++){
    lightVal = analogRead(lightSensor);
    offAvg += lightVal;
  }
  offAvg /= 10;
  Serial.print("Reset Value: ");
  Serial.println(offAvg);
  delay(delayTime);



//  reset(SOL_DAT);
//  delay(10);
//  offSol = 0;
//  for (int i =0; i<= 10;i++){
//    lightVal = analogRead(lightSensor);
//    offSol += lightVal;
//  }
//  offSol /= 10;
//  Serial.print("Reset Value: ");
//  Serial.println(offSol);
//  delay(delayTime);

  
  if((offAvg<=135) and (offAvg>=104)){

  
  sendData(.1,PIN_DAT);
  delay(10);
  lowAvg = 0;
  for (int i =0; i<= 10;i++){
    lightVal = analogRead(lightSensor);
    lowAvg += lightVal;
  }
  lowAvg /= 10;
  Serial.print("Low Value: ");
  Serial.println(lowAvg);
  delay(delayTime);

//  sendData(.1,SOL_DAT);
//  delay(10);
//  lowSol = 0;
//  for (int i =0; i<= 10;i++){
//    lightVal = analogRead(lightSensor);
//    lowSol += lightVal;
//  }
//  lowSol /= 10;
//  Serial.print("Low Value: ");
//  Serial.println(lowSol);
//  delay(delayTime);


  
  
  sendData(0.6,PIN_DAT);
  delay(10);
  midAvg = 0;
  for (int j =0; j<= 10;j++){
    lightVal = analogRead(lightSensor);
    midAvg += lightVal;
  }
  midAvg /= 10;
  Serial.print("Mid Value: ");
  Serial.println(midAvg);
  delay(delayTime);

//  sendData(0.6,SOL_DAT);
//  delay(10);
//  midSol = 0;
//  for (int j =0; j<= 10;j++){
//    lightVal = analogRead(lightSensor);
//    midSol += lightVal;
//  }
//  midSol /= 10;
//  Serial.print("Mid Value: ");
//  Serial.println(midSol);
//  delay(delayTime);




  
  
  sendData(48,PIN_DAT);
  delay(10);
  highAvg = 0;
  for (int k =0; k<= 10;k++){
    lightVal = analogRead(lightSensor);
    highAvg += lightVal;
  }
  highAvg /= 10;
  Serial.print("High Value: ");
  Serial.println(highAvg);
  delay(delayTime);

//  sendData(48,SOL_DAT);
//  delay(10);
//  highSol = 0;
//  for (int k =0; k<= 10;k++){
//    lightVal = analogRead(lightSensor);
//    highSol += lightVal;
//  }
//  highSol /= 10;
//  Serial.print("High Value: ");
//  Serial.println(highSol);
//  delay(delayTime);
}
}





void sendData (float setVoltage, int data_pin) {
  if (setVoltage < offset) {
    setVoltage = offset;
  }
//  else if (setVoltage > voltageLimit) {
//    setVoltage = voltageLimit;
//  }
  uint32_t bitCode = voltageToBits(setVoltage * cal - offset);
  if (bitCode > 0xFFFFF) {
    bitCode = 0xFFFFF;
    //Serial.println(bitCode);
  }
  
//  Serial.print("Voltage: ");
//  Serial.print(setVoltage, 4);
//  Serial.print("  Bits: ");
//  Serial.println(bitCode);
//  Serial.println("-----------------------");
//  Serial.println("");
  bitCode = bitCode << 4;
  //Serial.println(bitCode);
 

  digitalWrite(PIN_CS, LOW);
  shiftOut(data_pin, PIN_CLK, MSBFIRST, 0x40);
  shiftOut(data_pin, PIN_CLK, MSBFIRST, (bitCode & 0x00FF0000) >> 16);
  shiftOut(data_pin, PIN_CLK, MSBFIRST, (bitCode & 0x0000FF00) >> 8);
  shiftOut(data_pin, PIN_CLK, MSBFIRST, (bitCode & 0x000000FF));
  digitalWrite(PIN_CS, HIGH);

}
 
uint32_t voltageToBits(float voltage) {
  return (voltage / 50.0)* pow(2, 20);
}
 
void reset (int datPin) {

  digitalWrite(PIN_CS, LOW);
  //Reset DAC
  pinMode(PIN_CLK, OUTPUT);
  digitalWrite(PIN_CLK, LOW);
  delay(1);
  digitalWrite(PIN_CLK, HIGH);
  delayMicroseconds(300); //First high period (600 clocks)
  digitalWrite(PIN_CLK, LOW);
  delayMicroseconds(7);
  digitalWrite(PIN_CLK, HIGH);
  delayMicroseconds(520); //Second high period (1200 clocks)
  digitalWrite(PIN_CLK, LOW);
  delayMicroseconds(7);
  digitalWrite(PIN_CLK, HIGH);
  delayMicroseconds(1000); //Second high period (2400 clocks)
  digitalWrite(PIN_CLK, LOW);
  delay(1);
//  digitalWrite(PIN_CS, HIGH);
  //Start Self-Calibration
  shiftOut(datPin, PIN_CLK, MSBFIRST, 0x05);
  //20-bit resolution
  shiftOut(datPin, PIN_CLK, MSBFIRST, 0xA1); //20-bit resolution
  digitalWrite(PIN_CS, HIGH);
//  delay(600);

//  Serial.println("DAC reset sucessful");
//  sendData(2.5);
}
 
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, int val, uint8_t bits = 8, uint8_t del = 10) {
  uint8_t i;
  for (i = 0; i < bits; i++)  {
    if (bitOrder == LSBFIRST)
      digitalWrite(dataPin, !!(val & (1 << i)));
    else
      digitalWrite(dataPin, !!(val & (1 << ((bits - 1 - i)))));
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(del);
    digitalWrite(clockPin, LOW);
  }
}
 
uint8_t parseFloat(uint8_t *pbuf, uint8_t len, float *f) {
  uint8_t  i;
  uint8_t  c;
  uint8_t  period = false;
  uint8_t  n = 0;
  uint32_t m = 0;
  uint32_t divider = 1;
 
  for (i = 0; i < len; i++) {
    c = pbuf[i];
 
    if (c == '.') {
      if (period == false) {
        period = true;
        n = len - i - 1;
      } else {
        return false;
      }
    } else if (pbuf[i] >= 0x30 && pbuf[i] <= 0x39) {
      m = m * 10 + (c - 0x30);
    } else {
      return false;
    }
  }
 
  for (i = 0; i < n; i++) {
    divider *= 10;
  }
 
  *f = (float)m / divider;
  return true;
}
