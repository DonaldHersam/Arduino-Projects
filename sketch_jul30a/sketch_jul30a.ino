
  
/*
**----------------------------------------------------------------------
**   Example of using functions
**----------------------------------------------------------------------
**    writeToDIR();
**      shiftOutMB(0,3); //Set DIR
**    delay(500);
**
**    writeToDIR();
**    shiftOutMB(0xFFFFF,3); //Set DIR
**    delay(500);
**    
**    sweep();
**
**
**- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
*/  


#define RESOLUTION_16BIT (0x01)
#define RESOLUTION_20BIT (0x02)
#define Resolution (0x02)
#define CSPin 10
#define CLOCK_PIN 13
#define MOSI_PIN 11




void setup(){

  
  pinMode(CSPin, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);


  digitalWrite(CSPin, LOW);
  DACSetup();
}


void loop(){
    writeToDIR();
    shiftOutMB(0x001000,3); //Set DIR
    delay(500);

    writeToDIR();
    shiftOutMB(0xAAAAAA,3); //Set DIR
    delay(500);
}





///////////////////////////////////////////
//            DAC Setup
///////////////////////////////////////////


void DACSetup(void)
{
    //DDRE=0b00110000; //Set digital pins 2&3 as outputs

    resetDAC();
    shiftOutMB(0b00100100,1);       //write to cmd register
    shiftOutMB(0b0010000010100000,2); //program cmd register
}

///////////////////////////////////////////
//            Shift out Multi Byte
///////////////////////////////////////////


void shiftOutMB(long data, byte byteCount) {
  byte bits;
  long andHelper;

  switch (byteCount){
    case 1:
    bits=8;
    andHelper=0x80; // 1000 0000
    break;

    case 2:
    bits=16;
    andHelper=0x8000;
    break;

    case 3:
    bits=24;
    andHelper=0x80000<<4; // bottom 4 bits are not used
    data<<=4; // bottom 4 bits are not used
    break;
  }

  for (byte bitCounter=0; bitCounter < bits; bitCounter++) {

    if(data & andHelper)
//    PORTE |=DAC_DAC_MOSI; //write DAC_DAC_MOSI HIGH
    digitalWrite(MOSI_PIN, HIGH);
    else
//    PORTE &= ~DAC_DAC_MOSI; //write DAC_DAC_MOSI LOW
    digitalWrite(MOSI_PIN, LOW);

    data<<=1; // data=data<<1;

    delayMicroseconds(1);
//    PORTE |= CLK; //clk HIGH
    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(1);
//    PORTE &= ~CLK; //clk LOW
    digitalWrite(CLOCK_PIN, LOW);
  }
  delayMicroseconds(17); // min time b/t register data & command

}


///////////////////////////////////////////
//            Shift in Multi Byte
///////////////////////////////////////////

long shiftInMB(byte byteCount) {

  byte bits;

  switch (byteCount){
    case 1:
    bits=8;
    break;

    case 2:
    bits=16;
    break;

    case 3:
    bits=24;
    break;
  }

  long dataIn=0;

  for (byte bitCounter=0; bitCounter<bits; bitCounter++){

//    PORTE |= CLK; //clk HIGH
    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(2);

    dataIn<<=1;
//    if(PING & (1<<5)) // read MISO, pin 4
//    dataIn |= 1;

//    PORTE &= ~CLK; // clk low
    digitalWrite(CLOCK_PIN, LOW);
    delayMicroseconds(2);
  }

  delayMicroseconds(17); // min time b/t register data & command

  return dataIn;
}

///////////////////////////////////////////
//            Write to DIR Command
///////////////////////////////////////////

void writeToDIR(void){
  shiftOutMB(0b01000000,1); //write to DIR register
}

///////////////////////////////////////////
//            Read from DIR Command
///////////////////////////////////////////

void readFromDIR(void){
  shiftOutMB(0b11000000,1); //read from DIR register
}


///////////////////////////////////////////
//            Reset
///////////////////////////////////////////

void resetDAC(void) {
//  PORTE &= ~CLK; // make sure clk starts low
  digitalWrite(CLOCK_PIN, LOW);
  delayMicroseconds(2); //min clk time

//  PORTE |= CLK; //clk HIGH
  digitalWrite(CLOCK_PIN, HIGH);
  delayMicroseconds(267); //t16
//  PORTE &= ~CLK; // clk low
  digitalWrite(CLOCK_PIN, LOW);
  delayMicroseconds(4); //t17

//  PORTE |= CLK; //clk HIGH
  digitalWrite(CLOCK_PIN, HIGH);
  delayMicroseconds(575); //t18
//  PORTE &= ~CLK; // clk low
  digitalWrite(CLOCK_PIN, LOW);
  delayMicroseconds(4); //t17

//  PORTE |= CLK; //clk HIGH
  digitalWrite(CLOCK_PIN, HIGH);
  delayMicroseconds(905); //t19
//  PORTE &= ~CLK; // clk low
  digitalWrite(CLOCK_PIN, LOW);
  delayMicroseconds(4); //t17

  delay(100); // give time to reset
}


///////////////////////////////////////////
//            Sweep
///////////////////////////////////////////

void sweep(void){
  byte delayTime=100;
  for(long counter=10485; counter<=1048575; counter=counter+100){
    writeToDIR();
    shiftOutMB(counter,3); //Set DIR
    //delay(delayTime);
  }

  for(long counter=1048575; counter>=10485; counter=counter-100){
    writeToDIR();
    shiftOutMB(counter,3); //Set DIR
    //delay(delayTime);
  }
}

///////////////////////////////////////////
//            Check Read Write
///////////////////////////////////////////


boolean checkReadWrite(void){

  long Match_dataOut=0xFAAAF;

  writeToDIR();
  shiftOutMB(Match_dataOut,3); //Set DIR

  readFromDIR();
  long Match_dataIn=shiftInMB(3);

  Match_dataIn>>=4;

  if (Match_dataOut==Match_dataIn){
    //Serial.println("Good match");
    return 1;
  }
  
  else{
    //Serial.println("Bad match");
    return 0;
  }

  //Serial.println(Match_dataOut, BIN);
  //Serial.println(Match_dataIn, BIN);
}



///////////////////////////////////////////
//            checkLEDDriver
///////////////////////////////////////////

boolean checkLEDDriver (void){
  
  int waitToCheck = 20;
  int offCurrent;
  int halfCurrent;
  int fullCurrent;
  boolean LEDDriverStatus=0; // default is zero (=bad)
  
  
  writeToDIR();
  shiftOutMB(0,3); //Set DIR
  delay(waitToCheck);
  offCurrent=analogRead(57);

  
  writeToDIR();
  shiftOutMB(0x7FFFF,3); //Set DIR
  delay(waitToCheck);
  halfCurrent=analogRead(57);

  
  writeToDIR();
  shiftOutMB(0xFFFFF,3); //Set DIR
  delay(waitToCheck);
  fullCurrent=analogRead(57);
  
  //////////Check if within limits/////////
  
  // Use nested ifs as large AND statement
  
  if (offCurrent<=1023 && offCurrent>=1000) //nominal=1023
  {
    if (halfCurrent<=650 && halfCurrent>=550) // nominal=500
    {
      if (fullCurrent<=200 && fullCurrent>=100)
      {
        LEDDriverStatus=1; // set status to 1 (=good)
        
      }
      
    }
    
  }
  
  return LEDDriverStatus;
}
