/*
This code was written to provide the core motion control for My DIY Segway [aka: HarvWay] 
It's based on the following hardware:
 A. Two SX7970 H Bridge Motor Control Boards'
 B. a 2560 Mega Arduino
 C. a 6050 MPU
 D. a 2nd Leonardo W/ 2x16 LCD display [to act as the HarvWay's Dashboard]
In its current configuration the HarvWay is powered by two 24V Lipo battery packs
and two 24V/350W brushed geared motors which in turn drives a pair sprocketed 12.5" wheels
through #35 chains. 
FWIW The gearing is such that given a motor rpm of 3,000, the HarvWay will be traveling
~3MPH
*/


//#include <LiquidCrystal.h>
//#include <LCDKeypad.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

//create object to control an LCD.  
//number of lines in display=1
//LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
//LCDKeypad Keypad;
#define LEFTWHEEL 1  //Left Wheel Handle
#define RIGHTWHEEL 0  //Right Wheel Handle
#define ALARM 4 // Digital Pin# that the Audible Alarm is tied to;  
#define FORWARD true
#define REVERSE false
#define LINE1 1
#define LINE2 2


byte error;

byte address = 0x68;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// PWM_DIR controller type Definitions
//int LDC1 = A0;//3;      // SX7970 Left Motor Dir Control LDC1 connected to Arduino digital pin 1
//int LDC2 = A1;//3;      // SX7970 Left Motor Dir Control LDC2 connected to Arduino Analog pin 2
//int RDC1 = A2; //1;      // SX7970 Right Motor Dir Control RDC1 connected to Arduino digital pin 0
//int RDC2 = A3;//3;      // SX7970 Right Motor Dir Control RDC2 connected to Arduino Analog pin 3
//
//int EnA = 13; //11;     // SX7970 Left Motor Control EnA connected to Digital PWM pin 11
//int EnB = 11; //13;     // SX7970 Right Motor Control EnB connected to Digital PWM pin 13; Note also controls On-Board LED

// PWM_PWM controller type Definitions
int LPWMR = 8;    // SX7970 Left Motor Dir Control LDC1 connected to Arduino digital pin 8 [Black Wire]
int LPWMF = 7;    // SX7970 Left Motor Dir Control LDC2 connected to Arduino Analog pin 7  [White Wire]
int RPWMF = 6;    // SX7970 Right Motor Dir Control RDC1 connected to Arduino digital pin 6
int RPWMR = 5;    // SX7970 Right Motor Dir Control RDC2 connected to Arduino Analog pin 5

int EnL = 13;     // SX7970 Left Motor Control Enable connected to Digital pin 13; Note also controls On-Board LED
int EnR = 11;     // SX7970 Right Motor Control Enable connected to Digital pin 11



int SpeedL = 0;         // Speed Value for Motor A (Left Wheel)
int SpeedR = 0;         // Speed Value for Motor B (Right Wheel)
int Speed;
double MtrLdFctr = 1.0; //=(0.03*AvgSpeed)-0.7
double AvgLdFctr = 0.0;
double AvgSpeed = 0;
double SlowStart = 0;
int Steer = 0;
double MaxTipAngle = 10.0; // stop motors if HarvWay is more than 15 degrees (0.2618 Radians) off normal [level]
double AvgPitch = 0.0;

float Iamps =0.0;

//double RunningSpd = 0.0;// Used as part of auto Speed based Balance Correction
//double LastRunningSpd = 0.0;// Used as part of auto Speed based Balance Correction
double BPtDif = 0.0;// Used as part of auto Speed based Balance Correction
int loopCnt = 0;
int loopCnt2 = 1;
int loopCnt1 = 0; // auto yaw zero
int loopCnt3 = 0; // auto pitch zero
int loopCnt4 = 0; // auto Speed based Balance Correction
boolean LoopComplete = true;
boolean Gud2Go = false;
boolean ReSetRdy = false;
boolean SteerTst = false;
boolean MPUTst = false;
boolean STOP = false;
boolean NrMaxAngle = false;
boolean NrMaxThrtl = false;
boolean ALARMON = false;
boolean SportMode = false; //user Selectable PID multiplecation constants flag; when set "true", both steering, and pitch, outputs are enhanced 
boolean HillComp = false; //user Selectable hill climp compensation flag
//boolean Reverse = false;
//boolean LCD_On = false;
boolean SkipUpdate = false;
boolean stringComplete = false;  // whether the string is complete
boolean LastLDir = REVERSE;
boolean LastRDir = REVERSE;
String RXstring = "";         // a string to hold incoming data
String TXstring = "";         // a string to hold OutGoing data
unsigned long LastShw = millis();
int val = 0; 
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// PID variables
/*working variables*/
unsigned long lastTime;
double InputP, Setpoint;
double ITerm, lastInput;
double kp, ki, kd;
/* these worked at 12v
double Kp = 0.32;//0.32;
double Ki = 5.4;//5.4;
double Kd = 0.040; //0.052;
*/
/* Try the following 24v
*/
double Kp = 0.16;//0.32;0.384;//
double Ki = 2.7;//5.4;
double Kd = 0.020; //0.052;

double OS = 18;
double BrkPt = 22;
double X = 0.0;
//double gain = 1.0;
double PIDOut = 0.0;
double DeadBnd = 02.0; //62;; DeadBnd = OffSet Lower Limit; used in AplyOffSet routine 
                         // to eliminate Motor deadband throttle values
unsigned long lastTimeS;
double InputY, SetpointS;
double ITermS, lastInputS;
double kpS, kiS, kdS;
double KpS = 1.0;//0.2;
double KiS = 0.3;//0.02;
double KdS = 0.022;//0.022;
double PIDOutS = 0.0;
double DeltaStPt = 10.0; //fixed steering incremental change 
int SampleTime = 25; //1 sec
int IdealSmplTm = SampleTime;
unsigned long SampleTimeP, SampleTimeY;
double Min = -254.0;
double Max = 254.0;
double outMin, outMax;
double PitchBalancePt = -3.2;//-0.98;// A larger positive number makes the car lean further forward
double StrOffset = +2.2;// angle in degrees needed to correct steering/tiller output
double PtchBalPtRef = PitchBalancePt;
double CarLvl = 2.0; //Degree range that the Car's pitch angle needs to be within to initiate startup; This value will be converted to radians in the "setup" code 
//double BackAngleRef = 0.1;
//double BackAngle = 0.0;
//boolean BackAway = false;
double YawHeading = 0.0;
char Ser1PBuf [18];// used to support sending Data Via Serial Port 1 to be displayed on the HarvWay Dashoard [Leonardo LCD Display]
char Ptch[6];
char StrCmnd[6];
char CurYAW[6];
char LclBuf [6];
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// message
char msgs[9][17] = {"FORWARD Spd", 
                    "IR Dect", 
                    "BAL PT     ", 
                    "MPU GOOD", 
                    "MPU FAILED    ",
                    "STRAIGHT LINE",
                    "PRESS SELECT",
                    "AUTO-BAL PT ON ",
                    "AUTO-BAL PT OFF"};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void setup()
{

//setup PWM frequency to ~60Hz for OutPut pins 13 & 11
// TCCR4B = TCCR4B & 0b11111000 | 0x06; //Set Digital pin 13 Right Wheel PWM period @ ~2ms
  //TCCR0B = TCCR0B & 0b11111000 | 0x05; //pin 11 can't tinker with this one because its linked to the master clock
// For Arduino Mega1280, Mega2560, MegaADK, Spider or any other board using ATmega1280 or ATmega2560

//---------------------------------------------- Set PWM frequency for D4 & D13 ------------------------------
  
//TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
//TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
//TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz
//TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
//TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz


//---------------------------------------------- Set PWM frequency for D11 & D12 -----------------------------
  
//TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

//---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
  
//TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
//  TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
//TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz


//---------------------------------------------- Set PWM frequency for D2, D3 & D5 ---------------------------
  
//TCCR3B = TCCR3B & B11111000 | B00000001;    // set timer 3 divisor to     1 for PWM frequency of 31372.55 Hz
TCCR3B = TCCR3B & B11111000 | B00000010;    // set timer 3 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR3B = TCCR3B & B11111000 | B00000011;    // set timer 3 divisor to    64 for PWM frequency of   490.20 Hz *Default
//TCCR3B = TCCR3B & B11111000 | B00000100;    // set timer 3 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR3B = TCCR3B & B11111000 | B00000101;    // set timer 3 divisor to  1024 for PWM frequency of    30.64 Hz

  
//---------------------------------------------- Set PWM frequency for D6, D7 & D8 ---------------------------
  
//TCCR4B = TCCR4B & B11111000 | B00000001;    // set timer 4 divisor to     1 for PWM frequency of 31372.55 Hz
TCCR4B = TCCR4B & B11111000 | B00000010;    // set timer 4 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR4B = TCCR4B & B11111000 | B00000011;    // set timer 4 divisor to    64 for PWM frequency of   490.20 Hz *Default
//TCCR4B = TCCR4B & B11111000 | B00000100;    // set timer 4 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR4B = TCCR4B & B11111000 | B00000101;    // set timer 4 divisor to  1024 for PWM frequency of    30.64 Hz


//---------------------------------------------- Set PWM frequency for D44, D45 & D46 ------------------------
  
//TCCR5B = TCCR5B & B11111000 | B00000001;    // set timer 5 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR5B = TCCR5B & B11111000 | B00000010;    // set timer 5 divisor to     8 for PWM frequency of  3921.16 Hz
//  TCCR5B = TCCR5B & B11111000 | B00000011;    // set timer 5 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR5B = TCCR5B & B11111000 | B00000100;    // set timer 5 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR5B = TCCR5B & B11111000 | B00000101;    // set timer 5 divisor to  1024 for PWM frequency of    30.64 Hz 
  
//  lcd.begin(16, 2);
//  Serial.begin(9600);
  Serial1.begin(9600);  
  // initialize the Arduino digital pins as an OutPuts.
//  pinMode(LDC1, OUTPUT); //Left Motor Dir Control1
//  pinMode(LDC2, OUTPUT); //Left Motor Dir Control2 
//  pinMode(RDC1, OUTPUT); //Right Motor Dir Control1
//  pinMode(RDC2, OUTPUT); //Right Motor Dir Control2

  pinMode(EnL, OUTPUT);   //Left Enable Motor Control
  pinMode(LPWMF, OUTPUT); //Left Motor Forward PWM Control
  pinMode(LPWMR, OUTPUT); //Left Motor Reverse PWM Control 
  pinMode(EnR, OUTPUT);   //Right Enable Motor Control
  pinMode(RPWMF, OUTPUT); //Right Motor Forward PWM Control
  pinMode(RPWMR, OUTPUT); //Right Motor Reverse PWM Control
  pinMode(ALARM, OUTPUT); //Provision Audible Alarm Pin as a Digital Output   
  Speed = 0;
  SetMotorSpeed(LEFTWHEEL, Speed);
  SetMotorSpeed(RIGHTWHEEL, Speed);
  
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  if(StartMPU())
  {
  Serial1.write(LINE1);  
  Serial1.print(msgs[3]);
  Serial1.write(0); // Send Null Character Message Terminator
  }
  else
  {
  Serial1.write(LINE1);  
  Serial1.print(msgs[4]);
  Serial1.write(0);
  }
    SndBuzr();
    ConfigRunMode(); // wait 4 seconds for MPU to stablize 
    //initialize PID
    X =  16.0/BrkPt+(OS/(Kp*16.0));
    Setpoint = 0.00; // Robot heading [0 = straight ahead]
    SetTunings(Kp, Ki, Kd);
    SetTuningsS(KpS, KiS, KdS);
    SampleTimeY = (unsigned long)SampleTime;
    SampleTimeP = (unsigned long)SampleTime;
    PIDOut = 0.0;
    PIDOutS = 0.0;
    // convert Angle References from degrees to Radians
    MaxTipAngle = (M_PI/180)*MaxTipAngle;
    PitchBalancePt = (M_PI/180)*PitchBalancePt;
    PtchBalPtRef = PitchBalancePt;
//    BackAngleRef = (M_PI/180)*BackAngleRef;
    CarLvl = (M_PI/180)*CarLvl;
//    BackAngle = BackAngleRef;
//    RXstring ="";

}
// End Setup
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
     
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    if (stringComplete) 
     {
//       Serial.print("*");
//       Serial.print(RXstring);
       TXstring = "NO KEY      \n";
       //Serial.print(RXstring);
      
//      if (RXstring == String("UP          \n")) TestMotors(); //Testing Only; Proves basic motor setup; 
//                                                                   //Starts Motors at zero throttle, ramps up to full forward throttle, 
//                                                                   //then ramps down to max rev, and then back to zero
                                                                   
      //if (RXstring == String("UP          \n")) MPUTst= true; // Balance Pointcheck 
                                                                 // [Kills motors, but lets everything else operate in normal mode]
      if (RXstring == String("UP          \n")){ //Turn Load Compensation On
        HillComp = true;
        if(SportMode) TXstring = "FIRM - Hill ON";
        else  TXstring = "SOFT - Hill ON";        
      }
      if (RXstring == String("DOWN        \n")){ //Turn Load Compensation Off
        HillComp = false;
        if(SportMode) TXstring = "FIRM - Hill OFF";
        else  TXstring = "SOFT - Hill OFF";        
      }
      //if (RXstring == String("DOWN        \n")) SteerTst = true;// Steering Control Test; Lets you verify that the tiller runs the motors correctly
                                                                 // [Kills Pitch PID result, but lets everything else operate in normal mode]
      
      if (RXstring == String("RIGHT       \n")){
        SportMode = true;
        if (HillComp) TXstring = "FIRM - Hill ON";
        else  TXstring = "FIRM - Hill OFF";
      }
      if (RXstring == String("LEFT        \n")){
        SportMode = false;
        if (HillComp) TXstring = "SOFT - Hill ON";
        else  TXstring = "SOFT - Hill OFF";
      }
      if (RXstring == String("PRESS SELECT\n")){
        STOP = true;// Normally pressed immediately after "Dismount" to shutdown motors; 
                    // Requires Power Off Power On to reset stop flag
        TXstring = "STOP";            
      }
      if (TXstring != String("NO KEY      \n")){      
        Serial1.write(LINE2);
        Serial1.print(TXstring); // echo back string recieved from remote Terminal [normally the Leonardo Dashboard]
        Serial1.write(0); // Send Null Character Message Terminator
      } 
      // clear the string:
      RXstring = "";
      stringComplete = false;
     }
//    do // test to see if the MPU answers back [with its address]
//    {
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();
//    }
//    while  (error != 0);
    fifoCount = 0;
    while (fifoCount < packetSize)
    {
      fifoCount = mpu.getFIFOCount();
    }
    // get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();
    //digitalWrite(LED_PIN, true);
    fifoCount = mpu.getFIFOCount();
    //Serial.print("mpuIntStatus: ");
    //Serial.println(mpuIntStatus);
    //digitalWrite(LED_PIN, false);
    // check for overflow (this should never happen unless our code is too inefficient)
    // sometimes quits before getting here
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) { //  || fifoCount > 2*packetSize
        // reset so we can continue cleanly
        //lcd.clear();
        //lcd.printIn("FIFO overflow!");
        //ShowMessage("FIFO!", fifoCount, 1, 0);
        mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) { // never gets here
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
        {
         fifoCount = mpu.getFIFOCount();
        }
        //Serial.print(fifoCount);
        //Serial.print("\t");
        //Serial.println(packetSize);
        
        
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        //digitalWrite(LED_PIN, false);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        if (fifoCount ==0)
        {
            
              // display Euler angles in degrees
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetGravity(&gravity, &q);
              mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
              // enable the following print lines for seting only
//              Serial.print("ypr\t");
//              Serial.print(ypr[0] * 180 / M_PI);
//              Serial.print("\t");
//              Serial.print(ypr[1] * 180 / M_PI);
//              Serial.print("\t");
//              Serial.println(ypr[2] * 180 / M_PI);
              int VisLAdc = analogRead(A1);
              int VisRAdc = analogRead(A2);
              double InstI = 0.19*(VisRAdc+VisLAdc);
              /* 
              Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V)
              & then to a current based the SX7970 Spec Sheet equation I = 19.5 x Vis:
              */
              Iamps += InstI/30; //((VisLAdc+VisRAdc)*2 * 0.003177);// ((5/1023)*19.5)/30 =0.0031769

              InputP = PitchBalancePt- ypr[2]; // Remember Pitch reading is in Radians, not Degrees; 
                                                       // And given th orientation of the 6050[upsidedown and sideways], we're using the "Roll" reading
                                                       // as the "pitch" reading
              AvgPitch += InputP;
              loopCnt4 += 1;
              MtrLdFctr = InstI/((0.03*Speed)-0.7);//(Iamps*(31-loopCnt4))/((0.03*AvgSpeed)-0.7);
              if (MtrLdFctr >0) AvgLdFctr += MtrLdFctr;
              //Apply limits to Motor Load Factor
              if (MtrLdFctr <1 || Iamps <2.0) MtrLdFctr = 1;
              //else MtrLdFctr = 1.2*MtrLdFctr;
              //if(Iamps <2.0) MtrLdFctr = 1;
              if (MtrLdFctr >3.0) MtrLdFctr = 3.0;
              //AvgLdFctr += MtrLdFctr-1;
              /* Read Tiller Steering Signal
                 Given the orientation of the 6050 [upsidedown and sideways], we're using the "Pitch" reading
                 as the "Steering" command
              */
             if(SportMode) YawHeading = YawHeading - ((ypr[1] * 360/M_PI)-StrOffset)/10;
             else YawHeading = YawHeading - ((ypr[1] * 180/M_PI)-StrOffset)/10; 
              
              if(YawHeading <= -180.0) YawHeading = 360.0+YawHeading;
              else if (YawHeading > 180.0) YawHeading = YawHeading-360.0;  

              
                           
              // if we're running in "normal" mode, build an average reading of the car's pitch speed 
              //using the next 30 reported values
             if(loopCnt4 >= 30) //Based on a sample period of 25ms, a loopCnt4 of 40 is ~1 second; note - loopCnt4 gets incremented in the Pitch PID 
              {
                AvgPitch = AvgPitch/loopCnt4;
                AvgSpeed = AvgSpeed/loopCnt4;
                AvgLdFctr = AvgLdFctr/loopCnt4;
//                MtrLdFctr = Iamps/((0.03*AvgSpeed)-0.7);
//                if (MtrLdFctr <1) MtrLdFctr = 1;
//                else if(Iamps <2.0) MtrLdFctr = 1;
//                if (MtrLdFctr >1.5) MtrLdFctr = 1.5;
                if (Gud2Go)
                {
                  
                  //Set Alarm flag if we're approaching Max Motor Speed Value [i.e. +/- 200]
                  if (AvgSpeed > 200 && !MPUTst) NrMaxThrtl = true;
                  else if (AvgSpeed < -200 && !MPUTst) NrMaxThrtl = true;
                  else NrMaxThrtl = false;
                  //Set Alarm flag if we're approaching Max Pitch Value [i.e. Within 2 Degrees/0.035 Radians]
                  if (AvgPitch > MaxTipAngle - 0.035) NrMaxAngle = true;
                  else if (AvgPitch < -MaxTipAngle + 0.035) NrMaxAngle = true;
                  else NrMaxAngle = false;   
                  // stop motors if Chariot is more than 10 degrees off normal [level]
                  if(AvgPitch > MaxTipAngle) Gud2Go = !Gud2Go;
                  if(AvgPitch < -MaxTipAngle) Gud2Go = !Gud2Go;
                }
                if (NrMaxThrtl || NrMaxAngle){
                  digitalWrite(ALARM, HIGH); // Sound Audible Alarm
                  ALARMON = true;
                  if (NrMaxAngle)      sprintf (Ser1PBuf, "MAX ANGLE      ");
                  else if (NrMaxThrtl) sprintf (Ser1PBuf, "MAX SPEED      ");
                  Serial1.write(LINE1);
                  Serial1.write(Ser1PBuf); 
                  Serial1.write(0); // Send Null Character Message Terminator
                  loopCnt2 =8;
                }
                else if(ALARMON && !NrMaxThrtl && !NrMaxAngle){
                  digitalWrite(ALARM, LOW);
                  ALARMON = false;
                } 
               loopCnt2 -=1;
               if (loopCnt2==0) // if true; One second has elapsed since the last display update
                 {
                       loopCnt2 +=3;
                    // now convert & format readings so that they can be sent Via the serial port to be displayed
                    // on the DashBoard LCD
                    if (HillComp){
                      strncpy(Ptch, Rad2String(AvgLdFctr, 1, false), 6); // AvgLdFctr (0.03*AvgSpeed)-0.7 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                      //strncpy(Ptch, Rad2String(AvgSpeed, 0, false), 6);
                      //strncpy(Ptch, Rad2String(ypr[1] , 1, true), 6); // for testing show Steering signal in place of load factor
                    }
                    else strncpy(Ptch, Rad2String(AvgPitch, 1, true), 6); // Pitch reading
                    
                    memset(Ser1PBuf,0,sizeof(Ser1PBuf));
                    if (Gud2Go && !SteerTst && !MPUTst){
                      if (SlowStart <=Max) sprintf (Ser1PBuf, "SOFT START     ", StrCmnd, Ptch);
                      else {
                        //strncpy(StrCmnd, Rad2String(YawHeading,0,false), 6); // steering command or relative YAW heading.
                        //sprintf(Ser1PBuf, "Str%s P%s", StrCmnd, Ptch);
                        char* AmpStrng = Flt2String(Iamps);
                        //strncpy(AmpStr, MyStrng, 5);
                        if (HillComp) sprintf(Ser1PBuf, "A%s LdF%s ",AmpStrng, Ptch);
                        else sprintf(Ser1PBuf, "Amps%s P%s",AmpStrng, Ptch);
                      }
                    }
                    else if (Gud2Go && SteerTst){
                      strncpy(CurYAW, Rad2String(InputY,0,false), 6); //
                      sprintf (Ser1PBuf, "YAW%s Str%s", CurYAW, StrCmnd);
                    }
                    else if (Gud2Go && MPUTst){
                      strncpy(CurYAW, Rad2String(ypr[2], 1, true), 6);
                      sprintf (Ser1PBuf, "AP%s CP%s", CurYAW, Ptch);
                    }
                    else if (STOP) sprintf (Ser1PBuf, "STOP          ");
                    else  sprintf (Ser1PBuf, "LOCKOUT P %s", Ptch);
                    Serial1.write(LINE1);
                    Serial1.write(Ser1PBuf); 
                    Serial1.write(0); // Send Null Character Message Terminator
                 }
                AvgPitch = 0.0;
                AvgSpeed = 0.0;
                AvgLdFctr = 0.0;
                Iamps = 0.0;
                //RunningSpd = RunningSpd/loopCnt4;//; note - RunningSpd gets accumulated in the Pitch PID
                //RunningSpd = 0.0;
                loopCnt4 = 0;
              }
              InputY= -(ypr[0] * 180/M_PI); // convert Yaw Radian value to Degrees
              double CurPtchNdegs = InputP* 180/M_PI;
              if ( InputP < CarLvl && InputP > -CarLvl)// If BalanceBot is within 1 degree of vertical, then recallibrate Yaw offset
               { 
                 loopCnt1 += 1;
                 if (loopCnt1 >= 450)// its vertical long enough re-zero the yaw heading 
                 {
                   loopCnt1 = 0;
                   if(!Gud2Go)
                    {
                     // lock in initial [current] heading 
                     YawHeading = InputY;
                     StrOffset = (ypr[1] * 180/M_PI);
                     Gud2Go = true; 
                     //Reset any residual integral PID values 
                     ITerm = 0.0;
                     ITermS = 0.0;
                     BPtDif = 0.0;
                     SlowStart = 0.0;
                     //LastRunningSpd = 0.0;
                     // Energize Motor Controllers
                     digitalWrite(EnL, HIGH);
                     digitalWrite(EnR, HIGH);
                    } 
                   
                 }
               }
              else
               {
                 loopCnt1 = 0; 
               }
//               unsigned long CurTm = millis();
//               int DeltaT = (CurTm - LastShw);
//               if(DeltaT>=250)
//               {
//               loopCnt2 -=1;
//               if (loopCnt2==0) // if true; One second has elapsed since the last diaplay update
//                 {
//                   loopCnt2 +=4;
//                   //ShowMessage("Yaw" ,(int) (InputY), 1, 8); 
//                   if(!SkipUpdate)
//                    { 
//                    //ShowMessage("P" ,(int) (InputP* 180/M_PI), 1, 0);
//                    //ShowMessage("Y" ,(int) (InputY), 1, 8);
//                    }
//                    
//                   else SkipUpdate = false;
//                   // now, if Speed Signal is small, then its probably safe to take the time to Update the Yaw Display reading 
//                   //Rpt_IR = true; //reset IR Flag to enable new report
//                   //Speed = -Speed; // flip the sign of the speed value
//                   //ShowMessage("Spd" ,Speed, 2, 0);
//                   //ShowMessage("Str" ,Steer, 2, 8);
//                 }
//                LastShw = CurTm;
//               }
              Speed = CalcPtchPID(InputP);// Calculate new PID Balancing value
              if (SteerTst) Speed = 0;
              AvgSpeed += Speed; 
              // the Yaw Output ranges from -180 deg to +180 deg.
              // Test for yaw corrections trying to negotiating this +/- crossing point, &
              // as needed, temporarily extend  the yaw range to bridge the gap
              if (InputY-YawHeading < -180.0) InputY = InputY + 360.0;
              if (InputY-YawHeading > +180.0) InputY = InputY - 360.0;        
              Steer = ComputePIDS(InputY-YawHeading);
              //SpeedL= AplyOffSet(Speed+ RCSpeed - Steer, SpeedL);
              //SpeedR= AplyOffSet(Speed+ RCSpeed + Steer, SpeedR);
              //Steer = 0; //Enable this line for testing only
              SpeedL= AplyOffSet(Speed - Steer, SpeedL);
              SpeedR= AplyOffSet(Speed + Steer, SpeedR);
              //ShowMessage("Lft" ,(int) (SpeedL), 0, 8);
//              Serial.print("\tSpeedL\t");
//              Serial.println(SpeedL);
//              Gud2Go = false;
              if (STOP) Gud2Go = false;
              if (Gud2Go)
               {
                if (SlowStart <=Max) SlowStart += 2.0;
                if (MPUTst){
                  SpeedR = 0;
                  SpeedL = 0;
                } 
                //SpeedR=100;
                SetMotorSpeed(RIGHTWHEEL, SpeedR);
                SetMotorSpeed(LEFTWHEEL, SpeedL);
                ReSetRdy = true;
               }
              else
              {
                
                SetMotorSpeed(RIGHTWHEEL, 0);
                SetMotorSpeed(LEFTWHEEL, 0);
                // wait 2 seconds & then shutdown Motor Controllers
                if (ReSetRdy) //if true, then Up until now, the motor controllers were energized; Shut them down 
                {
                  ReSetRdy = false;
                  NrMaxThrtl = false;
                  NrMaxAngle = false;
                  AvgPitch = 0.0;
                  AvgSpeed = 0.0;
                  delay(2000);
                  digitalWrite(EnL, LOW);
                  digitalWrite(EnR, LOW);
                  if(StartMPU())
                    {
                     Serial1.write(LINE1); 
                     Serial1.print(msgs[3]);
                     Serial1.write(0); // Send Null Character Message Terminator
                    }
                  else
                    {
                     Serial1.write(LINE1); 
                     Serial1.print(msgs[4]);
                     Serial1.write(0);
                    }
                }
               }
              SampleTime =IdealSmplTm;
              SampleTimeY = (unsigned long)SampleTime;
              SampleTimeP = (unsigned long)SampleTime; 
              SetTunings(Kp, Ki, Kd);
              SetTuningsS(KpS, KiS, KdS);
 
        }
        else
        {
          mpu.resetFIFO();
        }
    }
 
}

//#############  END MAIN LOOP ##################################################

// ############ Test Buzzer #####################################################
void SndBuzr()
{
  int lclcntr =0;
  do{
  digitalWrite(ALARM, HIGH);
  delay(200);
  digitalWrite(ALARM, LOW);
  delay(75);
  lclcntr +=1;
  }
  while(lclcntr <2);
  return;
}

boolean StartMPU()
{
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) 
   {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);    // (Gidi) if you use the interrupts, you can set this to interrupt 2 for the Teensy2.0 
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        mpu.setXAccelOffset(-2058);
        mpu.setYAccelOffset(1860);
        mpu.setZAccelOffset(1542);
        mpu.setXGyroOffset(38);
        mpu.setYGyroOffset(-5);
        mpu.setZGyroOffset(8);

        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        return true;

   } 
 else 
  {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial1.write("DMP Setup failed");
        //delay(5000);
        return false;
    }
}  

//############################################################################## 
//########### Convert radian or Degree value to Degree String
char * Rad2String ( double RadVal, int Dp, boolean IsRadVal)
{
  //char LclBuf [6];
  double DegAngle;
  char* FormatStr;
  if (IsRadVal) DegAngle = RadVal*180/M_PI;
  else DegAngle = RadVal;
  char* Sign = "+";
  if (DegAngle < 0) Sign = "-"; 
  //AvgPitch = 0.0;
  int WhlDeg = (int) DegAngle;
  int FracDeg = (DegAngle-WhlDeg)*10;
  WhlDeg = abs(WhlDeg);
  FracDeg = abs(FracDeg);
  if (Dp == 1)
  { 
    FormatStr = "%s%02i.%01i";
    sprintf(LclBuf, FormatStr, Sign, WhlDeg, FracDeg); 
  }
  else if(Dp == 0)
  {
    FormatStr = "%03i ";
    sprintf(LclBuf, FormatStr, (int)RadVal);
  }
  else
  { 
    FormatStr = "%s%03i";
    sprintf(LclBuf, FormatStr, Sign, WhlDeg); 
  }
  
  //strncpy(LclBuf, LclBuf, 6);
  //LclBuf[5] = '\0';
  //Serial.println(LclBuf); 
  return LclBuf;
}
//###########################################################################
char * Flt2String ( float FltVal)
{
   char* FormatStr;
   char* Sign = "+";
   memset(LclBuf,0,sizeof(LclBuf));
   if (FltVal < 0) Sign = "-";
   int IntFltVal = (int)FltVal;
   int FracVal = (int)((FltVal-IntFltVal)*10);
   FormatStr = "%s%02i.%01i";
   sprintf (LclBuf, FormatStr, Sign, IntFltVal, FracVal);
   strncpy(LclBuf, LclBuf, 5);
   return LclBuf;
   }
//###########################################################################
void ConfigRunMode()
{
  //char buf [16];
  int downCnt = 4;
  do
  {
  sprintf (Ser1PBuf, "STARTING IN %i   ",downCnt);
  //lcd.setCursor(0,0); 
  //lcd.print(buf);
  Serial1.write(LINE1);
  Serial1.print(Ser1PBuf); // send StartUp/DownCount to hardware serial port
  Serial1.write(0); 
  delay(1000);
  downCnt = downCnt-1;
  }
  while (downCnt>0);
  TXstring = "SOFT - Hill OFF";
  Serial1.write(LINE2);
  Serial1.print(TXstring); // echo back string recieved from remote Terminal [normally the Leonardo Dashboard]
  Serial1.write(0); // Send Null Character Message Terminator
  //RXstring = "NO KEY      \n";
}
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

void TestMotors() {
  /*
  This is just a simple routine that is intended to validate that the motor
  wiring has been properly configured.
  In its current form, this code will start both motors in their forward direction
  and at a near zero throttle [Speed] setting. It gradually incretments the speed to 
  full throttle forward direction [+255], and then reverses the ramping so that the motors
  will gradually slow to a stop, then reverse their direction and will increase in speed
  until they hit max reverse throttle [-255]. The ramping action will reverse again. The
  motors will slow and come to a stop at zero throttle [speed =0]. At this point, the Arduino
  will exit this routine. 
  The code also contains provisions to update the Dashboard dispaly
 */ 
  boolean RampUp = true;
  boolean Continue = true;
  boolean MtrFlt = false;
  Speed = 5;
  // Energize Motor Controllers
  digitalWrite(EnL, HIGH);
  digitalWrite(EnR, HIGH);
  do
  {
   
   SetMotorSpeed(LEFTWHEEL, Speed);
   SetMotorSpeed(RIGHTWHEEL, Speed);
   //delay(250);
   int LpCnt = 0;
   float Iamps =0.0;
   double InstI =0.0;
   do
     {
       delay(10);
       int VisLAdc = analogRead(A1);
       int VisRAdc = analogRead(A2);
       // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V)& then to a current based the SX7970 Spec Sheet equation I = 19.5 x Vis:
       //Iamps += ((VisLAdc+VisRAdc)*2 * 0.0038);// ((5/1023)*19.5)/25 =0.003812316715542522
       Iamps += ((VisLAdc+VisRAdc)*2 * 0.003177);// ((5/1023)*19.5)/30 =0.0031769
       LpCnt +=1;
       InstI = 0.19*(VisRAdc+VisLAdc);
       // check for Motor Fault
       if (InstI > 15.0){
         SetMotorSpeed(LEFTWHEEL, 0);
         SetMotorSpeed(RIGHTWHEEL, 0);
         // De-Energize Motor Controllers
         digitalWrite(EnL, LOW);
         digitalWrite(EnR, LOW);
         Continue = false;
         MtrFlt = true;
       }
     }
   while (LpCnt <30);
   
   char* Sign = "+";
   if (Speed < 0) Sign = "-";
   int IntAmps = (int)Iamps;
   int FracAmp = (Iamps-IntAmps)*10;
   if (!MtrFlt) sprintf (Ser1PBuf, "Spd%s%03d Amp %02i.%01i ",Sign, abs(Speed), IntAmps, FracAmp);
   else sprintf (Ser1PBuf, "MOTOR FAULT ");
   Serial1.write(LINE1);
   Serial1.write(Ser1PBuf); 
   Serial1.write(0); // Send Null Character Message Terminator
   Serial.print(Ser1PBuf);
   IntAmps = (int)InstI;
   FracAmp = (InstI-IntAmps)*10;
   double EstAmp = 0.03*abs(Speed)-0.7;
   Sign = "+";
   if (EstAmp < 0) Sign = "-";
   int IntEA = (int)EstAmp;
   int FracEA = (EstAmp-IntEA)*10;
   FracEA = abs(FracEA);
   sprintf (Ser1PBuf, " Ea %s%02i.%01i Ia %02i.%01i ",Sign, IntEA, FracEA, IntAmps, FracAmp);
   Serial.print(Ser1PBuf);
   MtrLdFctr = InstI/((0.03*abs(Speed))-0.7);
   strncpy(Ptch, Rad2String(MtrLdFctr, 1, false), 6);
   sprintf (Ser1PBuf, " LdFctr %s ",Ptch);
   Serial.println(Ser1PBuf);
   if (MtrFlt) delay(10000);
   //ShowMessage("SPEED", Speed, 0, 0);
   
   if (RampUp && Speed == 0) Continue = false;
   if (RampUp) Speed += 5;
   if (Speed >= 255) RampUp = !RampUp;
   if (!RampUp) Speed -= 5;
   if (Speed <= -250) RampUp = !RampUp;
   
   
  }
  while(Continue);
  // De-Energize Motor Controllers
  digitalWrite(EnL, LOW);
  digitalWrite(EnR, LOW);
}

//#######################################################################






// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
double SqrVal(double x)
{
  double sign = 1.0; 
  if(x<1) sign = -1.0;
  if (x!=0) x = x/20.0;
  x = sign*x*x;
  return x;
}
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//###########################################################################
// Begin PID computations
int CalcPtchPID(float Input)
{
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTimeP)
   {
      SetSampleTime(timeChange);
      /*Compute all the working error variables*/
//      if (BackAway)
//       {
//        Input = Input+ BackAngle;
//        BackAngle += 0.1*BackAngleRef;
//        if (BackAngle>0.08) BackAngle = 0.04; //limit reversing anlge to a Max of 2.5deg (note 0.08 Radians = ~5 deg
//       }
//      else BackAngle = BackAngleRef;
      Input = 1530.0*sin(Input);// 255.0*6*sin(Input);
      double error = Setpoint - Input;
      double dInput = (Input - lastInput);
      //AvgPtchDif += dInput;
      /*
      Angle: 0.1 Error: 2.6703523998244125
      Angle: 0.2 Error: 5.3406966652923105
      Angle: 0.3 Error: 8.011024662071957
      Angle: 0.4 Error: 10.681328255881175
      Angle: 0.5 Error: 13.35159931251212
      Angle: 0.6 Error: 16.021829697856063
      Angle: 0.7 Error: 18.692011277928167
      Angle: 0.8 Error: 21.362135918892267
      Angle: 0.9 Error: 24.032195487085634
      Angle: 1.0 Error: 26.70218184904377
      Angle: 1.1 Error: 29.372086871525195
      Angle: 1.2 Error: 32.04190242153614
      Angle: 1.3 Error: 34.711620366355476
      Angle: 1.4 Error: 37.38123257355932
      Angle: 1.5 Error: 40.050730911045925
      Angle: 1.6 Error: 42.72010724706039
      Angle: 1.7 Error: 45.38935345021946
      Angle: 1.8 Error: 48.05846138953628
      Angle: 1.9 Error: 50.7274229344452
      Angle: 2.0 Error: 53.39622995482648
      */
      
      
     double offset = 0.0; 
     if(abs((int) error)>16)
      {
        // Square the error term
        double sign = 1.0;
        offset = OS;
        //double BrkPt = 22;//6.3; 
        if(error<1)
        {
          sign = -1.0;
          offset = -OS;
        }
        if (error!=0) error = error/BrkPt;
        error = sign*BrkPt*error*error;
      }
     else
      {
        //kp= kp*2.4;
        kp= kp*2.977;
      }
     error= error+offset;
     double DtI = ki*error;
     //if(RCSpeed>0) DtI = 0.7*DtI;
     
     double Cap = 14.0;//12.0;//10.0;
     if (DtI >Cap) DtI = Cap;
     else if (DtI <-Cap) DtI = -Cap;
     ITerm+= DtI;//ITerm+= (0.75*ki*error);
     
      /*Compute PID PIDOut*/
      PIDOut = kp*error + ITerm - kd * dInput;
      if (HillComp){
        //user has hill climp compensation turned on; if needed apply correction to PID output  
        if (PIDOut > 110 && MtrLdFctr> 1.0){
          PIDOut -= ITerm ;
          ITerm -= DtI;
          ITerm += MtrLdFctr*DtI;
          PIDOut = PIDOut +( kd * dInput);
          PIDOut =(1+(MtrLdFctr/6))*PIDOut; // limit max proportional boost to 1.5 times the oridinary correction
          PIDOut += (ITerm- kd * dInput) ;
        }
      }
//      Serial.print("ITerm ");
//      Serial.println(ITerm);
//      Serial.print("\tPIDOut ");
//      Serial.println(PIDOut);
      SetPIDOutLimits(-Max, Max, SlowStart);
//      if(Gud2Go && !BackAway)
//      {
//        RunningSpd += PIDOut;
//        loopCnt4 += 1;
//      }
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
   return (int) PIDOut;
   
}   
 
void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   //kp = Kp;
   if(SportMode){ kp = 2.4*Kp;
   ki = 2.1*Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
   }
   else{ kp = Kp; 
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
   }
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTimeP = (unsigned long)NewSampleTime;
   }
}

void SetPIDOutLimits(double Min, double Max, double SoftStart)
{
   if(Min > Max) return;
   
   if(SoftStart < Max){
     if (PIDOut > SoftStart)
      {
       ITerm -= PIDOut - SoftStart;
       PIDOut = SoftStart;
      }
     else if (PIDOut < -SoftStart)
      {
       ITerm  -=(PIDOut + Min);//+= Min - PIDOut;// -=(PIDOut - Min)
       PIDOut = -SoftStart;
      }     
   return;
   }

   if (PIDOut > Max)
    {
      ITerm -= PIDOut - Max;
      PIDOut = Max;
    }
   else if (PIDOut < Min)
    {
      ITerm  -=(PIDOut - Min);//+= Min - PIDOut;// -=(PIDOut - Min)
      PIDOut = Min;
    }
}
 

// End PID computations
//#######################################################################

// Begin PID Steering computations
int ComputePIDS(float Input)
{
   unsigned long now = millis();
   int timeChange = (now - lastTimeS);
   if(timeChange>=SampleTimeY)
   {
      SetSampleTimeS(timeChange);
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      double dInput = (Input - lastInputS);
      double outMax = 255;
      double outMin = -255;
      ITermS+= (kiS * error);
      if(ITermS> outMax) ITermS= outMax;
      else if(ITermS< outMin) ITermS= outMin;
      
      if(ITermS> SlowStart) ITermS= SlowStart;
      else if(ITermS< -SlowStart) ITermS= -SlowStart;
      /*Compute PID PIDOut*/
      PIDOutS = kpS * error + ITermS - kdS * dInput;
      if(PIDOutS > outMax)
      {
        PIDOutS = outMax;
      }
      else if(PIDOutS < outMin)
      {
        PIDOutS = outMin;
      }
      if(SlowStart < Max){
        if (PIDOutS > SlowStart)
         {
          //ITerm -= PIDOut - SoftStart;
          PIDOutS = SlowStart;
         }
        else if (PIDOutS < -SlowStart)
         {
          //ITerm  -=(PIDOut + Min);//+= Min - PIDOut;// -=(PIDOut - Min)
          PIDOutS = -SlowStart;
         }     
      }
       /*Remember some variables for next time*/
      lastInputS = Input;
      lastTimeS = now;
//      Serial.print("PIDOutS\t");
//      Serial.print(PIDOutS);
   }
   
//   Serial.print("\tYAW");
//   Serial.println(Input);
   return (int) PIDOutS;
   
}   
 
void SetTuningsS(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kpS = Kp;
   kiS = Ki * SampleTimeInSec;
   kdS = Kd / SampleTimeInSec;
}
 
void SetSampleTimeS(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      kiS *= ratio;
      kdS /= ratio;
      SampleTimeY = (unsigned long)NewSampleTime;
   }
}
 


// End PID Steer computations
//#######################################################################

int AplyOffSet( int SpdSgnl, int LastSpeed)
{
 if (SpdSgnl <= LastSpeed && SpdSgnl >0) 
   {
    //LastSpeed = SpdSgnl; 
    return SpdSgnl;
   } 
 else if (SpdSgnl >= LastSpeed && SpdSgnl <0) 
   {
    //LastSpeed = SpdSgnl; 
    return SpdSgnl;
   }
 else if (SpdSgnl ==0)
   {
    //LastSpeed = SpdSgnl; 
    return SpdSgnl;
   }  
   
 if (SpdSgnl <0)
   {
    SpdSgnl = -DeadBnd + SpdSgnl;
    //return SpdSgnl;
   }
 else SpdSgnl = +DeadBnd + SpdSgnl;
 
 if(SpdSgnl > Max) SpdSgnl = Max;
   else if(SpdSgnl < -Max) SpdSgnl = -Max;
 //LastSpeed = SpdSgnl; 
 return SpdSgnl;   
}
//#######################################################################

//// normally called by the PWM_DIR SetMotorSpeed routine [see below] 
//void SetMotorDir(int Motor , boolean Dir)
//{
//  int InPin;
//  int InPin_;
//  if( Motor == LEFTWHEEL)
//   {
//    InPin = LDC2; // If the left wheel turns in the wrong Dir relative to the Right Wheel, reverse ldc1 & ldc2 relationship 
//    InPin_ = LDC1;// If the left wheel turns in the wrong Dir relative to the Right Wheel, reverse ldc1 & ldc2 relationship
//   }
//  else
//   {
//    InPin = RDC1; 
//    InPin_ = RDC2;
//   } 
//  if( Dir) // if true set up forward
//   {
//    digitalWrite(InPin, LOW);   // if the wheels turn the wrong way, flip the High/Low setting
//    digitalWrite(InPin_, HIGH); // if the wheels turn the wrong way, flip the High/Low setting
//    
//   }
//  else
//   {
//    digitalWrite(InPin, HIGH); // if the wheels turn the wrong way, flip the High/Low setting
//    digitalWrite(InPin_, LOW); // if the wheels turn the wrong way, flip the High/Low setting
//    }
//}

// //PWM_DIR Controller Type Interface
//void SetMotorSpeed(int Motor,int Speed)
//{
//  int PWMpin;
//  boolean Dir;
//  if (Speed > 0)
//   {
//     Dir = FORWARD;
//   }
//  else
//   {
//     Dir = REVERSE;
//     Speed = - Speed;
//   }
//  if (Motor == LEFTWHEEL)
//  {
//   PWMpin = EnA;
//  }
//  else
//  {
//   PWMpin = EnB;
//  }
//  analogWrite(PWMpin, 0);
//  SetMotorDir(Motor , Dir);
//  analogWrite(PWMpin, Speed);
//}  

//########################################################################

//PWM_PWM Controller Type Interface
void SetMotorSpeed(int Motor, int MtrSpd)
{
  boolean Dir;
  if (MtrSpd >= 0)
   {
     Dir = FORWARD;
   }
  else
   {
     Dir = REVERSE;
     MtrSpd = - MtrSpd;
   }
   
  
  
  if (Motor == LEFTWHEEL)
  {
    if (Dir != LastLDir)
    {
      LastLDir = Dir;
      analogWrite(LPWMF, 0);
      analogWrite(LPWMR, 0);
    } 
   if (Dir == FORWARD)
    {
      analogWrite(LPWMF, 0);  // Digital Pin 7 [White]
      analogWrite(LPWMR, MtrSpd); // Digital Pin 8 [Black]
    }
   else
    {
      analogWrite(LPWMR, 0); // Digital Pin 7 [White]
      analogWrite(LPWMF, MtrSpd); // Digital Pin 8 [Black]
    }
  }
  else// this call is to setup the Right side Motor Speed & Direction
  {
    if (Dir != LastRDir)
    {
      LastRDir = Dir;
      analogWrite(RPWMF, 0);
      analogWrite(RPWMR, 0);
    } 
   if (Dir == FORWARD)
    {
      analogWrite(RPWMF, 0); // Digital Pin 6  [Black]
      analogWrite(RPWMR, MtrSpd); // Digital Pin 5 [White]
      //Serial.println(MtrSpd);
    }
   else
    {
      analogWrite(RPWMR, 0); // Digital Pin 5  [White]
      analogWrite(RPWMF, MtrSpd); // Digital Pin 6  [Black]
      //Serial.println(MtrSpd);
    }
  }
}  
//########################################################################
//void ShowMessage(char* text, int value, int line, int pos)
//{
//  char buf [16];
//  sprintf (buf, "%s %03i ", text, value); // Concatenate text message with interger speed value 
////  lcd.cursorTo(line, pos);
////  lcd.printIn(buf);
//  lcd.setCursor(pos, line);
//  lcd.print(buf); 
//} 
//#######################################################################
//void SetMotorDir(int Motor , boolean Dir)
//{
//  if( Motor == LeftWheel)
//  {
//    if( Dir) // if true set up forward
//    {
//     digitalWrite(LDC1, HIGH);
//     //digitalWrite(LDC2, LOW);     
//    }
//    else
//    {
//     digitalWrite(LDC1, LOW);
//     //digitalWrite(LDC2, HIGH);
//    }
//  }
// else
//  {
//    if( Dir) // if true set up forward
//    {
//     digitalWrite(RDC1, HIGH);
//     //digitalWrite(RDC2, LOW);     
//    }
//    else
//    {
//     digitalWrite(RDC1, LOW);
//     //digitalWrite(RDC2, HIGH);
//    }
//
//  } 
//}
//
//void SetMotorSpeed(int Motor,int Speed)
//{
//  int PWMpin;
//  if (Motor == LeftWheel)
//  {
//    PWMpin = EnA;
//  }
//  else
//  {
//    PWMpin = EnB;
//  }
//  analogWrite(PWMpin, Speed);
//}  

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent1(){
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read(); 
    // add it to the RXstring:
    RXstring += inChar;
    //Serial.println(RXstring);
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    } 
  }
  //if (stringComplete) Serial.print(RXstring);
}



  
