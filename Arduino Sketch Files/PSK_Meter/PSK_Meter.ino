/* PSKmeter (KW4KD).
 *  As written, this Sketch uses these Principal Components
 *  Arduino Leonardo
 *  AD9850 DDS Generator
 *  SPI Serial 128X64 OLED Display
 *  Rotory Switch, Type KY-040
 *  MCP6002-I/P MCP6002 Dual Low Power Op Amp
 *  AA143 DIODE GERMANIUM 60mA/30V x2
 */

/* 

  ROTARY ENCODER KY-040 on Arduino Leonardo/Sept 2015
  WIRING INFORMATION
  ===================
  Connect CLK to Pin D2 on Arduino Board  (CLK is Data Output 1 of KY-040)
  Connect DT  to Pin A4 on Arduino Board  (DT is Data Output 2 of KY-040)
  Connect SW  to Pin A5 on Arduino Board  (Switch - goes LOW when pressed)
  Connect GND to ground
  Connect +   to +5V  (this will pull up CLK and DT with 10 KiloOhm resistors)
  ----------------------------------------------------------------------------
  Connect a 10 KiloOhm resistor from +5V to SW (no integrated pullup for SW !!)
  ----------------------------------------------------------------------------
  It is better NOT to use internal pull-up resistors on the Arduino, instead
  use the integrated pull-ups of KY-040 (this requires "+" to be connected to 5V).
  You can check if your version of the KY-040 has pull-up resistors on the bottom
  side ouf the printed circuit board.
  If not, use internal pull-ups from Arduino or external pull-ups.
  -----------------------------------------------------------------------------
 
 */

#include <U8glib.h>
#if defined(__AVR_ATmega32U4__)
  //Code in here will only be compiled if an Arduino Leonardo is used.
  #define InterruptId 1
#endif
//#if defined(__AVR_ATmega16U4__)
//  //Code in here will only be compiled if an Arduino Uno is used.
//  #define InterruptId 0
//#endif

#if defined(__AVR_ATmega328P__)
  //Code in here will only be compiled if an Arduino Uno is used.
  #define InterruptId 0
#endif

 
 #define W_CLK 8       // Pin 8 - connect to AD9850 module word load clock pin (CLK)
 #define FQ_UD 9       // Pin 9 - connect to freq update pin (FQ)
 #define DATA 10       // Pin 10 - connect to serial data load pin (DATA)
 #define RESET 11      // Pin 11 - connect to reset pin (RST).
 
 #define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }



#define PSK_Logo_width 50
#define PSK_Logo_height 28
const unsigned char PSK_Logo_bits[] PROGMEM = {
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03, 0xfc, 0xff, 0xff, 0xff, 0xff,
   0xff, 0x03, 0xfa, 0xcf, 0x3f, 0x90, 0xf8, 0x0f, 0x00, 0xf8, 0xc7, 0x3f,
   0x21, 0xf0, 0x07, 0x00, 0xf0, 0x83, 0x1f, 0x00, 0xf0, 0x27, 0x00, 0xf0,
   0x17, 0x1f, 0x01, 0xe2, 0x43, 0x00, 0xe0, 0x43, 0x1f, 0x00, 0xe0, 0x03,
   0x00, 0xe0, 0x0b, 0x0f, 0x00, 0xc8, 0x13, 0x00, 0xc0, 0x41, 0x4e, 0x00,
   0xe0, 0x41, 0x00, 0xc0, 0x01, 0x0e, 0x00, 0xc0, 0x01, 0x00, 0xc0, 0x00,
   0x06, 0x00, 0x80, 0x01, 0x00, 0x80, 0x00, 0x06, 0x00, 0x80, 0x00, 0x00,
   0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x06, 0x00, 0x80, 0x00, 0x00, 0x80,
   0x00, 0x06, 0x00, 0x80, 0x00, 0x00, 0xc0, 0x01, 0x06, 0x00, 0xc0, 0x01,
   0x00, 0xc0, 0x01, 0x0e, 0x00, 0xc0, 0x01, 0x00, 0xc0, 0x01, 0x0f, 0x00,
   0xc0, 0x01, 0x00, 0xe0, 0x01, 0x0f, 0x00, 0xe0, 0x03, 0x00, 0xe0, 0x83,
   0x1f, 0x00, 0xe0, 0x03, 0x00, 0xf0, 0x83, 0x1f, 0x42, 0xf0, 0x07, 0x00,
   0xf4, 0xc7, 0x3f, 0x00, 0xf2, 0x07, 0x00, 0xf8, 0xcf, 0x7f, 0x02, 0x78,
   0x07, 0x00, 0xff, 0xff, 0xff, 0x6e, 0x7f, 0xad, 0x01, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0x03 };
 
//PSK Meter Variables - Start 
 #define MAXSAMPLES 64 //exactly two psk31 cycles
float sample[MAXSAMPLES];
double theta[MAXSAMPLES];
double A[10];
double B[10];
int SampleWait = 876; //wait interval between in Microseconds between each sample; ideal value yields a "period" reading of 64000; (64 milliseconds),
int HalfWait;
unsigned long Start; 
unsigned long Stop;
unsigned long period;
bool UseSerPrt = false; // Set to 'true' if you want to capture/record data via the Arduino's IDE serial monitor
bool StandAlone = false; // Set to 'false' if you want to use this sketch with the PSKscope program [to actively manage the sound cards volume setting]
bool PSKMode =true;
String RXstring = "";         // a string to hold incoming data
String TXstring = "";         // a string to hold OutGoing data
bool PskDataRdy = false;
bool PScpLinked = false;
//PSK Meter Variables - Stop
U8GLIB_SH1106_128X64 u8g(3, 4, 7, 6 , 5); // SW SPI Com: CLK = 3, MOSI = 4, CS = 7, dc = 6, RES = 5
double Start_Freq; //13.4e6;//  20.5e6;//6.4e6; //start Freq; in Mhz  20.5e6;//  3.6e6;//
double End_Freq; //14.5e6;// 22.5e6;//7.8e6; //End Scan Freq; in Mhz 22.5e6;// 4.5e6;//
double Step_Freq; //= 10e3; //Step  Freq; in KHz 25000;// 25e3;// 
double VSWRLim = 1.7;
int FwdOffSet;
int RevOffSet;
// After build is complete first find the two following values
int FwdOpAmpGain = 92;// initialially set to 1; then set to FWD reading found when Cathodes of D1 and D2 are shorted together; (Op Amp Gain loop compensation)
int RevOpAmpGain = 88;// initialially set to 1; then set to REV reading found when Cathodes of D1 and D2 are shorted together; (Op Amp Gain loop compensation)
// Next determine these two values
int FwdSCVal = 30;// initialially set to 1; then set to reading found when antenna leg of bridge is shorted; Diode sensitivity compensation
int RevSCVal = 40;// initialially set to 1; then set to reading found when antenna leg of bridge is shorted; Diode sensitivity compensation
// Once the above 4 values have been established, then the antenna analyzer's accuracy can be evaluated.

double LastVSWR;
bool VSWRtrendUP;
//bool Incrmnt;
bool RunCurve;
char buf [32]; // used to concatenate strings & numerical data for display
double SweepData[8];

//// Rotory Sw Global Variables ////

bool TurnDetected;
bool SwRead = false;
bool StartSwp = false;
bool RunCal = true;
int IntCnt = 0;
int IncDecVal = 1;
int virtualPosition =1;
unsigned long WaitInterval = micros();
const int PinCLK=2;  // Used for generating interrupts using CLK signal
const int PinDT=A4;  // Used for reading DT signal
const int PinSW=A5;  // Used for the push button switch
//// End of Rotory Sw Global Variables ////


///******** Begin Rotory Sw Interrupt Service Routine *******///

void isr()  { 
   unsigned long ThisIntTime = micros();
   if (ThisIntTime < WaitInterval) return;
   WaitInterval = ThisIntTime+80000; //move the next valid interrupt out by 3 milliSeconds
   int cnt =0;
   int DtHIcnt =0;
   int ClkHIcnt =0;
   bool ClkLow = true;
   bool DtLow = true;
   detachInterrupt(InterruptId);
   int SmplCnt = 30;
   while(cnt <SmplCnt*2){
    if(analogRead(PinDT)>= 500) DtHIcnt++;
    if (digitalRead(PinCLK)!= 0) ClkHIcnt++;
    cnt++;
   }
   if(DtHIcnt>SmplCnt+1) DtLow = !DtLow;
   if(ClkHIcnt>SmplCnt+1) ClkLow =!ClkLow;
   if( ClkLow == DtLow) IncDecVal = -1;
   else IncDecVal = +1;
//   if (ClkLow) Serial.print ("cL/");
//   else Serial.print ("cH/");
//   if (DtLow) Serial.print ("dL; ");
//   else Serial.print ("dH; "); 
//  if (IncDecVal>0) Serial.print ("+ ");
//  else  Serial.print ("- ");
   virtualPosition = virtualPosition+ IncDecVal;
    if( IncDecVal > 0 && virtualPosition >=7 ) virtualPosition = 1;
    if( IncDecVal < 0 && virtualPosition <=0 ) virtualPosition = 6;
//    sprintf (buf,"Count = %d", virtualPosition);
//    Serial.println ("**");

   attachInterrupt (InterruptId,isr,CHANGE);
   TurnDetected = true;
} // Interrupt service routine End

///******** End of Rotory Sw Interrupt Service Routine *******///


 // transfers a byte, a bit at a time, LSB first to the 9850 via serial DATA line
void tfr_byte(byte data)
{
  for (int i=0; i<8; i++, data>>=1) {
    digitalWrite(DATA, data & 0x01);
    pulseHigh(W_CLK);   //after each bit sent, CLK is pulsed high
  }
}

 // frequency calc from datasheet page 8 = <sys clock> * <frequency tuning word>/2^32
void sendFrequency(double frequency) {
  int32_t freq = frequency * 4294967295/125000000;  // note 125 MHz clock on 9850
  for (int b=0; b<4; b++, freq>>=8) {
    tfr_byte(freq & 0xFF);
  }
  tfr_byte(0x000);   // Final control byte, all 0 for 9850 chip
  pulseHigh(FQ_UD);  // Done!  Should see output
}

void setup() {
   
// setup encoder
   pinMode(PinCLK,INPUT);
   pinMode(PinDT,INPUT);  
   pinMode(PinSW,INPUT);
   attachInterrupt (InterruptId,isr,CHANGE);   // Use interrupt 0 [Uno] or 1 [Leonardo]; It is always connected to pin 2 on Arduino UNO
   TurnDetected = false;
   StartSwp = false;
   Serial.begin (19200);
   virtualPosition = 1;
   SwRead = false;
// end encoder setup
    
// configure arduino data pins for output
    pinMode(FQ_UD, OUTPUT);
    pinMode(W_CLK, OUTPUT);
    pinMode(DATA, OUTPUT);
    pinMode(RESET, OUTPUT);

    pulseHigh(RESET);
    pulseHigh(W_CLK);
    pulseHigh(FQ_UD);  // this pulse enables serial mode - Datasheet page 12 figure 10
    for (int i=0; i<64; i++) {
    //theta[i]= (2*M_PI/32)*i;
    theta[i]= (M_PI/32)*i;
    if (theta[i] >= M_PI) theta[i] = theta[i]-M_PI;
    }
    HalfWait = SampleWait/2 ;
    RevOffSet = 0.0;
    FwdOffSet = 0.0;
    sendFrequency(1.0);  // set AD9850 output to 1 Hz
    Splash_Screen(1);
 }

void loop() {
  int incomingByte = 0;   // for incoming serial data
  
  delay(100);
  while(!StartSwp){
    ReadPushButton();
    if (TurnDetected){
      TurnDetected = !TurnDetected;
      Splash_Screen(virtualPosition);
     }
    if(PSKMode) StartSwp = true; 
  }
 
  detachInterrupt(InterruptId); // Disable Rotory Sw interrupts While running Antenna Sweep

  
  if(RunCal){
        // First, recalibrate "zero input" offsets
  int FOAccum = 0;
  int ROAccum = 0;      

  for (int i=0; i<60; i++) {
       ROAccum += analogRead(A0);
       FOAccum += analogRead(A1);
       delay(10);
     }
     if ( ROAccum/60.0 <10 ){
       RevOffSet = ROAccum/60.0;
       FwdOffSet = FOAccum/60.0;
     }
     RunCal = !RunCal;
     if(RevOffSet < 0) RevOffSet = 0; 
  // Calibration Complete
  }
   while (!PScpLinked && !UseSerPrt &&!StandAlone){
       while (Serial.available()) {
        // get the new byte:
        char inChar = (char)Serial.read(); 
        if (inChar == 'v') {
         Serial.print("KF6VSG");
         // add it to the RXstring:
         //RXstring += inChar;
         PScpLinked = true;
        } 
      }
   }

  if (PSKMode){ 
     // PSK Meter variables
      int i=0;
      int j;
      int cnt=0;
      int iOld=0;
      int minSig;
      int maxSig;
      bool sigFal = true;
      double ScaleFctr = 0.0;
      double error= 0.0;
      double rms =0.0;
      double mean =0.0;
      double E = 0.0;
      double H = 0.0;
      double F = 0.0;
      double IMD = 0.0;
      int REV = 0;
      double PskPeriod =  64000;
      PskDataRdy = false;
      //i = 0;
      //iOld =0;
      //cnt =0;
      //ScaleFctr = 0.0;
      // the following code syncs the sampling period to begin at the beginning of the 31Hz cycle
      // first wait for the sig to peak
      while((iOld <= i || sigFal) && cnt <80){
         iOld = i;
         delayMicroseconds(SampleWait); // wait .96 millisecond
         i = analogRead(A0)-RevOffSet;
         if (i > iOld) ++cnt ;
         if (i > iOld && cnt >3) sigFal = false;
         ++cnt;
      }
      maxSig = iOld;
      cnt = 0;
      //ScaleFctr = 1.00/maxSig;
      // now wait for sig to bottom out/ hit a minimum
      while( i >2&& cnt < 80){ //iOld >= i ||
          iOld = i;
         delayMicroseconds(HalfWait); // wait .96 millisecond
         i = analogRead(A0)-RevOffSet;
          ++cnt;
      }
      minSig = iOld;
       // now start the sampling routine & take MAXSAMPLES samples separated by 1 msec:
      Start = micros(); 
      for (i=0;i<MAXSAMPLES; i++){
          sample[i] = analogRead(A0)-RevOffSet;
          //sample[i] = 1024;//testing only
          delayMicroseconds(SampleWait); // wait .96 millisecond
        }
      period = micros()-Start;
      int adjVal =0;
      if(period > PskPeriod) adjVal = -1;
      if(period < PskPeriod) adjVal = +1;
      SampleWait = SampleWait+adjVal;
      // linearize readings
      for (i=0; i<MAXSAMPLES; i++){
        if(sample[i]>REV) REV=int(sample[i]); //store peak sig
        sample[i] =CorrectReading(sample[i]);// linearize raw Diode readings based on calibration equation
        ////////////////////////////////////////////////////////////////
        //     Fake -20dbm Signal                                     //
        ////////////////////////////////////////////////////////////////
        if(false){// include the following lines to simulate a signal with a -20db THD /IMD
            maxSig = 31;
            //ScaleFctr = 2.28/1000;
            double HE = (-0.1*sin(3*(2*M_PI/64)*i));
            if (i< 31) HE = -HE;
            sample[i] = (HE+sin(theta[i]))*250;// use for testing only; Simulates a perfect signal when HE is et to zero
        }//end -20db IMD simulation
      }

      // now now using FFT algorythm calc harmonic Amplitudes in the Freq Domain
    
//      if (maxSig>30){
         for (j=0; j < 10; j++){ // But first "zero out" old harmonic readings
          A[j] =0.0;
          B[j] =0.0;
         }
         double SmplVal = 0.0;
         for (j=0; j < 10; j++){
          for (i=0; i< 64; i++){ // now using the current data extract the odd and even harmonic energy
            SmplVal = sample[i];
            if (i>31) SmplVal = -SmplVal;
            A[j] = A[j] + (SmplVal*sin((j+1)*(2*M_PI/64)*i));//2*M_PI*
            B[j] = B[j] + (SmplVal*cos((j+1)*(2*M_PI/64)*i));
           
           }
         }
         E = 0.0;
         double Eold = 0.0;    
         for (j=0; j <10; j++){
           E = E + (A[j]*A[j])+ (B[j]*B[j]);
//          uncomment the following if you want to examine the harmonic energy distribution              
//          Serial.print("E");
//          Serial.print(j);
//          Serial.print(",EVal:,");
//          Serial.print(E-Eold);
//          Serial.println(" ");
//          Eold = E;
         }
         j = 0;
         H = E - ((A[j]*A[j])+ (B[j]*B[j])); //Subtract out the fundemental component
         F = H/E; //get the ratio of Harmonic energy to All energy
         if (F>0){
         IMD = 10*log10(F); // convert the ratio to DBs
         //IMD = 10*log(F) / log(10); //convert THD ratio to a DB value
         }
         /*
          * Now using the energy contained in the Fundemental signal (31hz) 
          * compute the scale factor needed to "Normalize" the samples just collected
          */
         ScaleFctr = 32/sqrt((A[j]*A[j])+ (B[j]*B[j]));
//       }//end if(maxSig>30)
      for (i=0; i<MAXSAMPLES; i++){
        error = (sample[i]*ScaleFctr) -  sin(theta[i]);
        rms = rms + (error*error);
        mean = mean +error;
        if(maxSig>30 && UseSerPrt){ // output / print to serial port only if there is useful data
          Serial.print("Smpl:,");
          Serial.print(i);
          Serial.print(",Val:,");
          Serial.print(sample[i]);//*ScaleFctr
          Serial.print(",Sin:,");
          Serial.print(sin(theta[i])/ScaleFctr);
          Serial.println("");
        }
      }
      rms = sqrt(rms)/(MAXSAMPLES-1);
      mean = mean/MAXSAMPLES;
       if(maxSig>30 && UseSerPrt){ // output / print only if there is useful data
          Serial.print("PERIOD:,");
          Serial.print(period); // Ideal value is 64000,
           Serial.print(", MaxSig:,");
          Serial.print(maxSig);
          Serial.print(", RMS:,");
          Serial.print(rms);
          Serial.print(", MEAN:,");
          Serial.print(mean);
          Serial.print(", IMD:,");
          Serial.print(IMD);
          Serial.print(", Scale Factor:,");
          Serial.print(1000*ScaleFctr);
          Serial.println("");
       }
      

     u8g.firstPage();
     do{;
     int row = 11; //first OLED row to print on
      u8g.setPrintPos(8, row);  
      int x0 = 0;
      int y0 = 64;
      int x1 = 0;
      int y1 = 0;
      //if(CorrectReading(REV) > 1/ScaleFctr) 
      //ScaleFctr = 1/CorrectReading(REV);
      float maxval= 0.0;
      for (i=0; i<MAXSAMPLES; i++){
        if(sample[i]>maxval) maxval = sample[i]; 
      }
      for (int i=0; i<64; i++) {
        x1 = 2*i;
        if (IMD >-1.0) y1 = 64;
        //if(sample[i]>REV) REV=sample[i];
        else y1 = 64-(int)(62*sample[i]/maxval);
        if(y1 <0) y1 = 0;
        u8g.drawLine(x0, y0, x1, y1);
        x0 = x1;
        y0 = y1;
      }
      row = 50;
      u8g.setPrintPos(17, row);
      u8g.print("IMD: ");
      u8g.setPrintPos(80, row);
      u8g.print("PkV: ");
      row = NuRow(row);
      u8g.setPrintPos(17, row);
      u8g.print(IMD);// 
      u8g.setPrintPos(80, row);
      u8g.print((float)(5.0*CorrectReading(REV)/1024.0));//correted PeakV
      } while( u8g.nextPage() );
      //&&&&&&&&&&&&&& for Testing Only &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
      if(false){
        GenTestData(); // Generate Perfect Data Steam
        maxSig = 31;
      }
      //&&&&&&&&&&&&&& End For Testing Only &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
      // now if PSKscope is connected send captured data via serial link
      if(maxSig>30) PskDataRdy = true;
      else RunCal = true;
      while (Serial.available() && PScpLinked && PskDataRdy) { //check for send request ('s') from PSKscope
        // get the new byte:
        char inChar = (char)Serial.read();
        if (inChar == 's') { // if true, we have a valid request
          for (int i=0; i<64; i++) { // send data
            //int y1 = int(fabs(sample[i]*0.63));//old single byte method
            //Serial.write(y1);//old single byte method
            //send each data sample as two bytes
            float absVal = fabs(sample[i]);
            int y1 = (int)(absVal/256);
            Serial.write(y1);// write the HIGH Byte
            y1 = (int)(absVal-(256*y1));
            Serial.write(y1);// write the LOW Byte
           // get access to the float as a byte-array:
//           float absVal = fabs(sample[i]);
//           byte * data = (byte *) &absVal;
           // write the data to the serial
//           Serial.write (data, sizeof (absVal));
          }
           PskDataRdy = false;
        } 
      }
      return; 
  }//end PSK mode
}  // End Main Loop Code


//*************************************************************************
double CorrectReading(double ReadVal){
double CalcVal;//22.0417;
 if( ReadVal<=167){
  CalcVal = -(4.61597*ReadVal*ReadVal*ReadVal*ReadVal/10000000)+(0.000194461*ReadVal*ReadVal*ReadVal)-(0.0294468*ReadVal*ReadVal)+(2.42394*ReadVal);
 }
 else{
  CalcVal =0.44324138* ReadVal+(78.242-24.841);
 }
  CalcVal = 2.01* CalcVal;// this bumps the slope back to where a count of 1024 in gives a count of ~1024 out
 return CalcVal;
}

int NuRow(int oldrow){
   {
    int pixcnt = 13;
    oldrow += pixcnt;
    return oldrow; 
   }
}

// Splash Screen
void Splash_Screen(int option){
   char str_Band[4];
   char buf2 [32];
   char buf3 [32];
   char buf4 [32];
   char str_StrtFq[6];
   char str_EndFq[6];
   char str_SWRLim[6];
   char str_Step[6];
   
   switch (option) {
     case 1: //Set Freq Limits for 80_MTRS 
      Start_Freq = 3.0e6; // Freq; in Mhz
      End_Freq =  4.5e6; // Freq; in Mhz
      Step_Freq = 15e3; //Step  Freq; in KHz
      sprintf (str_Band, "80"); //str_Band = "40"
      break;
    case 2: //Set Freq Limits for 40_MTRS 
      Start_Freq = 6.4e6; // Freq; in Mhz
      End_Freq =  7.8e6; // Freq; in Mhz
      Step_Freq = 10e3; //Step  Freq; in KHz
      sprintf (str_Band, "40"); //str_Band = "40"
      break;
    case 3: //Set Freq Limits for 30_MTRS 
      Start_Freq = 9.6e6; // Freq; in Mhz  20.5e6;//  3.6e6;//
      End_Freq =  10.25e6; // Freq; in Mhz
      Step_Freq = 15e3; //Step  Freq; in KHz
      sprintf (str_Band, "30");
      break;
    case 4: //Set Freq Limits for 20_MTRS 
      Start_Freq = 13.4e6; // Freq; in Mhz  20.5e6;//  3.6e6;//
      End_Freq =  14.8e6; // Freq; in Mhz
      Step_Freq = 20e3; //Step  Freq; in KHz
      sprintf (str_Band, "20");
      break;
    case 5: //Set Freq Limits for 15_MTRS 
      Start_Freq = 20.5e6; // Freq; in Mhz
      End_Freq =  22.5e6; // Freq; in Mhz
      Step_Freq = 30e3; //Step  Freq; in KHz
      sprintf (str_Band, "15"); //str_Band = "15"
      break;
    case 6: //Set Freq Limits for 10_MTRS 
      Start_Freq = 25.5e6; // Freq; in Mhz 
      End_Freq =  31.5e6; // Freq; in Mhz
      Step_Freq = 40e3; //Step  Freq; in KHz
      sprintf (str_Band, "10");
      break;  
   }
    
//   double SFqMhz = Start_Freq/1e6;
//   double EFqMhz = End_Freq/1e6; 
    
//   dtostrf(SFqMhz, 4, 2, str_StrtFq);// char *dtostrf(double val, signed char width, unsigned char prec, char *s)
//   dtostrf(EFqMhz, 4, 2, str_EndFq);
//   dtostrf(VSWRLim, 4, 2, str_SWRLim);
//   dtostrf(Step_Freq/1000, 4, 1, str_Step);
//   sprintf (buf,"%s to %s Mhz", str_StrtFq,str_EndFq);
//   sprintf (buf2,"MAX VSWR: %s", str_SWRLim);
//   sprintf (buf3,"Step: %s Khz", str_Step);
//   sprintf (buf4,"%sM Ant Analyzer", str_Band);
   u8g.firstPage();
   do {
    int row = 11; //first OLED row to print on
    u8g.setFont(u8g_font_unifont);
    u8g.setPrintPos(0, row);
    u8g.print(" << PSK Meter >>");
    row = NuRow(row);
    u8g.drawXBMP( 15, row-4, PSK_Logo_width, PSK_Logo_height, PSK_Logo_bits);
    u8g.drawXBMP( 65, row-4, PSK_Logo_width, PSK_Logo_height, PSK_Logo_bits);
    row = NuRow(row);
    row = NuRow(row);
    u8g.setFont(u8g_font_7x13);
    row = NuRow(row);
    u8g.setPrintPos(20, row);
    u8g.print("(( -KW4KD- ))");
  } while( u8g.nextPage() );
}
 ////////////////////////////////////////////////////////////////////////////////////
void ReadPushButton(){   //Begin DeBounce & Read Rotory Push Button
  int cnt =0;
  if(analogRead(PinSW)>= 700 && SwRead){
    while (analogRead(PinSW)>= 700 && cnt < 5) {
      delay(20);
      if(analogRead(PinSW)>= 700) cnt++;
      else cnt =0;
    }
    if (cnt == 5){
      SwRead = !SwRead;
      cnt = 0;
    }
  }
    
  while (analogRead(PinSW)<= 200 && cnt < 5) {
    delay(20);
    if(analogRead(PinSW)<= 200) cnt++;
    else cnt =0;
  }
  if (!SwRead && cnt == 5){
    SwRead = !SwRead;
    StartSwp = true;  
//    virtualPosition=0;              // if cnt == 5, then reset counter to ZERO
//    Serial.print ("Reset = ");      // Using the word RESET instead of COUNT here to find out a buggy encoder
//    Serial.println (virtualPosition);
//    sprintf (buf,"Reset = %d", virtualPosition);
  }
  //End of DeBounce & Read Rotory Push Button code 

 }
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void GenTestData() {
  //uchar vals[64];
  float limit = (1.05*255.0)/(5.0);
  float p, v, offset;
  int n,shift;
  p = 255.0/5 ;//p = 255.0*1/5;
  offset = 2*M_PI*2/64;
  //if (testmode == 1)
    shift = 0;
  //else
  //  shift = (int) ((float)rand()*256/RAND_MAX);
  for (int i = 0; i < 64; i++) {
    n = shift + i;
   // if ( 144 <= n && n < 240) 
   //   v = p;
   // else
      v = p*fabs(cos((2*M_PI*n/64)-offset));//p * fabs(sin(M_PI*(n%256)/64));// applying sin or cos plus or adding phase shifting did NOT effect the results as displayed on the PSKscope 
    if (v > limit) v = limit;
    sample[i] = int(v);
  }
 // pskdata->data(vals);
}
////******************************************************************//////

 

