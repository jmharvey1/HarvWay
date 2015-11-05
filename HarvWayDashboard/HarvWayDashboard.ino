/*
  This code was origially written as part of the Harvway [DIY Segway] Project. 
  It programs a Leonardo W/ 16x2 LCD Display & Button set to function
  as a remote Diagnostic Display/Dashboard via a serial connection to a Mega 2560 

 It receives data on serial port 1, and sends it to a 16x2 LCD Display.
 
 
 The circuit: 
 * A serial device attached to Serial port 1
 
 created 25 July 2014
 
 This example code is in the public domain.
 
 */

// include the library code:
#include <LiquidCrystal.h>
#include <LCDKeypad.h>
#define LEDpin 2
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
LCDKeypad Keypad;
char MsgHdr[4];
char FltMsgHdr[4]= {"MAX"};
char SerBuf [20];
int LineLength = 0;
int DispPos = 0;
int LastButtonPressed =3;
int BlinkCnt = 0;
//unsigned long LastTime;
//int FlashPeriod = 50; //measured in milliSeconds
unsigned long timer1_counter; // governs LED flash period
boolean SndCmnd = false;
boolean BlnkLED = false;
//boolean LEDOn = false;
int CmndId;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// LCD message
char msgs[6][17] = {"NO KEY      ", 
                    "LEFT        ", 
                    "RIGHT       ", 
                    "UP          ", 
                    "DOWN        ",
                    "PRESS SELECT"};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//########################################################################

ISR(TIMER1_OVF_vect)        // interrupt service routine 
{
  TCNT1 = timer1_counter;   // preload timer
  if (BlnkLED) digitalWrite(LEDpin, digitalRead(LEDpin) ^ 1);
  if (BlinkCnt > 0) BlinkCnt -=1;
  return;
}

//########################################################################


void setup() {
    // initialize timer1 
  noInterrupts();           // disable all interrupts
  pinMode(LEDpin, OUTPUT);
  TCCR1A = 0;
  //TCCR1B = 0;

  //---------------------------------------------- Set PWM frequency for D11 & D12 -----------------------------
  
  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
  TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
  //  TCCR1B |= (1 << CS12);    // 256 prescaler
 
  // Set timer1_counter to the correct value for our interrupt interval
  //timer1_counter = 64886;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz
  timer1_counter = 62411 ;   // preload timer 65536-(16000000/256/20Hz)
  
  TCNT1 = timer1_counter;   // preload timer
 
  
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts

  Serial.begin(9600);   // Testing Only
  // initialize serial port 1:
  Serial1.begin(9600);
  lcd.begin(16, 2);
  lcd.clear();
  Keypad.noBlink();
  // 1/2 Second Blink Warning LED Demo
  BlnkLED = true;
  delay(1000); 
  BlnkLED = !BlnkLED;
  // End LED Demo
}



void loop() {
  // if needed send the current button status
         if (SndCmnd)
        {
          SndCmnd = !SndCmnd;
          //ShowMessage("", msgs[CmndId], 2, 0);
          for(int x=0; x < 20; x++) SerBuf[x]= 0; // reset/clear character memory array
          sprintf (SerBuf, "%s\n", msgs[CmndId]);
          Serial1.print(SerBuf);
          Serial.print(SerBuf);         // Testing Only  
          //Serial1.write(0); // Send Null Character Message Terminator
        } 

  // Attempt to read from port 1
  
  if (Serial1.available()>0)
  {
    //DispPos = Serial1.read();
    do 
     {
      if (Serial1.available()>0)
       {
        SerBuf[LineLength] = Serial1.read();
        LineLength +=1;
       }
      }
    while (SerBuf[LineLength-1] != 0 && LineLength < 17);// read serial data until you hit the Null character '0' 
    if (LineLength >0)
     {
       int x;
       DispPos = (int) SerBuf[0]; // get line assignment
       for( x=1; x < LineLength; x++) SerBuf[x-1]= SerBuf[x]; //shift character buffer one position left
       LineLength = x-1;
       if (LineLength >0){
         if (LineLength < 16){
          for( x=LineLength-1; x <16; x++) SerBuf[x]= 32;
         }
         for( x=16; x < 20; x++) SerBuf[x]= 0;
//         Serial.print(SerBuf);         // Testing Only 
//         Serial.print("  ");           // Testing Only
//         Serial.println(LineLength);   // Testing Only
         strncpy(MsgHdr, SerBuf, 3);
         int chrPos = 0;
        // Now Check to see if MEGA sent Warning Message [Starts w/ 'MAX']
        BlnkLED = true; // Assume that it did
        do
        {
          if (MsgHdr[chrPos] != FltMsgHdr[chrPos] && BlinkCnt==0) BlnkLED = false;
         chrPos +=1; 
        }
        while (chrPos < 4);
        if (!BlnkLED && ( digitalRead(LEDpin)==1)) digitalWrite(LEDpin,LOW) ; // If not needed make sure Warning LED is off
        if (LineLength <18)  ShowMessage("", SerBuf, DispPos, 0); 
        else ShowMessage("", "OVERFLOW", 1, 0);
        //ShowMessage1("Char Count:", LineLength, 1, 0);
       }
      // now clear char buffer in preperation to receive next remote serial message
      LineLength =0;
      for(int x=0; x < 20; x++) SerBuf[x]= 0; // reset/clear character memory array
     }
  }
 // Now Scan Keypad for input
  
  int buttonPressed = KEYPAD_NONE;
  int LastVal = KEYPAD_NONE;
  int LVCnt = 0;
  int TestButtonCnt = 0;
  do
   {
    buttonPressed=Keypad.button();
    if (buttonPressed != LastVal && buttonPressed != KEYPAD_NONE){
      LastVal = buttonPressed;
      LVCnt = 0;
    }
    else if (buttonPressed != KEYPAD_NONE) LVCnt +=1;
    TestButtonCnt += 1;
   }
  while ( TestButtonCnt < 25 && buttonPressed != KEYPAD_NONE);
  if (LastVal != LastButtonPressed && LVCnt > 20 && buttonPressed != KEYPAD_NONE ){
  //if (LastVal != KEYPAD_NONE && LVCnt > 7){
     LastButtonPressed = LastVal;


//  int buttonPressed = 255;
//  int LastVal = 255;
//  int LVCnt = 0;
//  int TestButtonCnt = 0;
//  do
//   {
//    buttonPressed=Keypad.button();
//    if (buttonPressed != LastVal){
//      LastVal = buttonPressed;
//      LVCnt = 0;
//    }
//    else LVCnt +=1;
//    TestButtonCnt += 1;
//   }
//  while ( TestButtonCnt < 10 && buttonPressed != KEYPAD_NONE);
//  //Keypad.noBlink();
//  if ((int)buttonPressed != (int)LastButtonPressed && LVCnt>7){
//     LastButtonPressed = buttonPressed;
     SndCmnd = true;
     BlnkLED = true;
     BlinkCnt =2;
     switch (LastButtonPressed) {
      case KEYPAD_NONE:
        CmndId = 0;
        break;
      case KEYPAD_LEFT:
        CmndId = 1;
        break;
      case KEYPAD_RIGHT:
        CmndId = 2;
        break;    
      case KEYPAD_UP:
        CmndId = 3;
        break;
      case KEYPAD_DOWN:
        CmndId = 4;
        break;
      case KEYPAD_SELECT:
        CmndId = 5;  
        break;
       //default: 
        // if nothing else matches, do the default
        // default is optional
    }
   } 
}



//########################################################################
void ShowMessage(char* text, char* data, int line, int pos)
{
  //char buf [16];
  //sprintf (buf, "%s %s        ", text, data); // Concatenate text message with interger speed value
 //sprintf (buf, "%s        ", data); 
//  lcd.cursorTo(line, pos);
//  lcd.printIn(buf);
  if (line>0) line -=1;
  lcd.setCursor(pos,line);
  //lcd.print(buf);
 lcd.print(data); 
} 
//########################################################################
void ShowMessage1(char* text, int data, int line, int pos)
{
  char buf [16];
  sprintf (buf, "%s %03i ", text, data); // Concatenate text message with interger speed value 
//  lcd.cursorTo(line, pos);
//  lcd.printIn(buf);
  lcd.setCursor(pos,line);
  lcd.print(buf); 
} 
//#####################################################################
//void FlashLED()
//{
// unsigned long now = millis();
//   int timeChange = (now - LastTime);
//   if(timeChange>=FlashPeriod) {
//     LastTime = now;
//     if (!LEDOn){
//       digitalWrite(LEDpin, HIGH);
//        LEDOn = true;
//     }
//     else{
//       digitalWrite(LEDpin, LOW);
//        LEDOn = false;
//     }
//   }
//  return;
//} 
