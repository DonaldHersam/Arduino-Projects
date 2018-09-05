#define PIN_CS 53 //enable
#define PIN_CLK 52 //spi clock
#define PIN_DAT 51 //spi data
 
uint32_t Millis = 0;
uint8_t  Buf[10];
uint8_t  Len = 0;
float cal = 1.00015; //linear distortion
float offset = 0.0004; //offset (due to output drivers of DAC1220) -> depends on YOUR circuit! (ground level)
float voltageLimit = 4.99; //upper voltage limit (due to output drivers of DAC1220)
 
void setup() {
  Serial.begin(9600);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_DAT, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  reset();
}
 
void loop() {

  int delayTime = 750;
  
  reset();
  sendData(10);
  delay(delayTime);
  sendData(26);
  delay(delayTime);
  sendData(49);
  delay(delayTime);


//  float v=0;
//
//  if (millis() - Millis > 10) {
//    if (Len) {
//      if (parseFloat(Buf, Len, &v)) {
//        Serial.print("set voltage: ");
//        Serial.print(v, 4);
//        Serial.print("  set bits: ");
//        uint32_t bitCode1 = (uint32_t)(v / 5 * 0xFFFFF + 0.5);
//        Serial.println(bitCode1);
// 
//        //reset();
//        sendData(v);
//      }
//    }
//    Len = 0;
//  }
// 
//  while (Serial.available()) {
//    if (Len < 10) {
//      Buf[Len++] = Serial.read();
//      if (Buf[0] == 'r') reset();
//    } else {
//      Serial.read();
//    }
//    Millis = millis();
//  }
}
 
void sendData (float setVoltage) {
//  if (setVoltage < offset) {
//    setVoltage = offset;
//  }
//  else if (setVoltage > voltageLimit) {
//    setVoltage = voltageLimit;
//  }
  uint32_t bitCode = voltageToBits(setVoltage);// * cal - offset);
  if (bitCode > 0xFFFFF) {
    bitCode = 0xFFFFF;
    Serial.println(bitCode);
  }
  
//  Serial.print("Voltage: ");
//  Serial.print(setVoltage, 4);
//  Serial.print("  Bits: ");
//  Serial.println(bitCode);
//  Serial.println("-----------------------");
//  Serial.println("");
  bitCode = bitCode << 8;
  Serial.println(bitCode);
 
  digitalWrite(13, HIGH);
  digitalWrite(PIN_CS, LOW);
  shiftOut(PIN_DAT, PIN_CLK, MSBFIRST, 0x40);
  shiftOut(PIN_DAT, PIN_CLK, MSBFIRST, (bitCode & 0x00FF0000) >> 16);
  shiftOut(PIN_DAT, PIN_CLK, MSBFIRST, (bitCode & 0x0000FF00) >> 8);
  shiftOut(PIN_DAT, PIN_CLK, MSBFIRST, (bitCode & 0x000000FF));
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(13, LOW);
}
 
uint32_t voltageToBits(float voltage) {
  return (voltage / 50.0)* pow(2, 20);
}
 
void reset () {
//  digitalWrite(13, HIGH);
  digitalWrite(PIN_CS, LOW);
  //Reset DAC
  pinMode(PIN_CLK, OUTPUT);
  digitalWrite(PIN_CLK, LOW);
  delay(1);
  digitalWrite(PIN_CLK, HIGH);
  delayMicroseconds(240); //First high period (600 clocks)
  digitalWrite(PIN_CLK, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_CLK, HIGH);
  delayMicroseconds(480); //Second high period (1200 clocks)
  digitalWrite(PIN_CLK, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_CLK, HIGH);
  delayMicroseconds(960); //Second high period (2400 clocks)
  digitalWrite(PIN_CLK, LOW);
  delay(1);
//  digitalWrite(PIN_CS, HIGH);
  //Start Self-Calibration
  shiftOut(PIN_DAT, PIN_CLK, MSBFIRST, 0x05);
  //20-bit resolution
  shiftOut(PIN_DAT, PIN_CLK, MSBFIRST, 0xA1); //20-bit resolution
  digitalWrite(PIN_CS, HIGH);
//  delay(600);
//  digitalWrite(13, LOW);
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
