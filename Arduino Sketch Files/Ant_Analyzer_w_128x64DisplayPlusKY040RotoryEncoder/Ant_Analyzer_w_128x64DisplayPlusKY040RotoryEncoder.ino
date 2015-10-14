/* DIY Antenna Analyzer W/ Simple Band Menu Select Menu.
 *  As written, this Sketch uses these Principal Components
 *  Arduino Leonardo
 *  AD9850 DDS Generator
 *  SPI Serial 128X64 OLED Display
 *  Rotory Switch, Type KY-040
 *  MCP6002-I/P MCP6002 Dual Low Power Op Amp
 *  AA143 DIODE GERMANIUM 60mA/30V x2
 */

/* 
 * A simple single freq AD9850 Arduino test script
 * Original AD9851 DDS sketch by Andrew Smallbone at www.rocketnumbernine.com
 * Modified for testing the inexpensive AD9850 ebay DDS modules
 * Pictures and pinouts at nr8o.dhlpilotcentral.com
 * 9850 datasheet at http://www.analog.com/static/imported-files/data_sheets/AD9850.pdf
 * Use freely
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
// Small signal correction table [array] {No longer used, but left in place as a possible option}
int CrtdVal[] = {
  0, 0, 2, 4, 7, 9, 12, 14, 16, 19, 22, 25, 28, 31, 33, 35,36, 37, 39, 40, 41, 42, 43, 44, 44, 46, 47, 49, 50, 52, 53, 54, 55, 57, 58, 
  59, 60, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 79, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97,
  98, 99, 100, 101, 102, 103, 104, 105, 105, 106, 107, 108, 109, 110, 111, 111, 112, 113, 114, 114, 115, 116, 117, 117, 118, 119, 120, 120, 121, 122, 122,
  123, 124, 124, 125, 126, 126, 127, 128, 129, 130, 131, 132, 133, 133, 134, 135, 136, 137, 138, 139, 139, 140, 141, 142, 143, 143, 144, 145, 146, 147, 147,
  148, 149, 150, 151, 151, 152, 153, 154, 154, 155, 156, 157, 157, 158, 159, 160, 160, 161, 162, 162, 163, 164, 164, 165, 166, 166, 167, 168, 168, 169, 170,
  171, 172, 172, 173, 174, 175, 176, 176, 177, 178, 179, 179, 180, 181, 182, 182, 183, 184, 185, 185, 186, 187, 187, 188, 189, 190, 190, 191, 192, 193, 193,
  194, 195, 196, 196, 197, 198, 198, 199, 200, 200, 201, 202, 203, 203, 204, 205, 205, 206, 207, 207, 208, 209, 209, 210, 211, 211, 212, 213, 213, 214, 215,
  215, 216, 216, 217, 218, 218, 219, 220, 220, 221, 221, 222, 223, 223, 224, 225, 225, 226, 226, 227, 228, 228, 229, 229, 230, 230, 231, 232, 232, 233, 233,
  234, 234, 235, 235
  };

//// Rotory Sw Global Variables ////

bool TurnDetected;
bool SwRead = false;
bool StartSwp = false;
//bool ClkLow = false;
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
    if( IncDecVal > 0 && virtualPosition >=6 ) virtualPosition = 1;
    if( IncDecVal < 0 && virtualPosition <=0 ) virtualPosition = 5;
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
   Serial.begin (9600);
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
    Serial.begin(9600);
    Splash_Screen(1);
 }

void loop() {
  int incomingByte = 0;   // for incoming serial data
  sendFrequency(1.0);  // set AD9850 output to 1 Hz
  delay(100);
  while(!StartSwp){
    ReadPushButton();
  
    if (TurnDetected){
      TurnDetected = !TurnDetected;
      Splash_Screen(virtualPosition);
    }
//    // Upate OLED display with current Counter or Reset value   
//    u8g.firstPage();
//    do {
//      int row = 11; //first OLED row to print on
//      u8g.setFont(u8g_font_unifont);
//      u8g.setPrintPos(0, row);
//      u8g.print(buf);
//   
//     } while( u8g.nextPage() ); //// End Encoder Loop Instructions
  }
  //StartSwp = !StartSwp;
  detachInterrupt(InterruptId); // Disable Rotory Sw interrupts While running Antenna Sweep


// Continue original Ant Analyzer loop  
  // First, recalibrate "zero input" offsets
  RevOffSet = 0.0;
  FwdOffSet = 0.0;
  for (int i=0; i<20; i++) {
       RevOffSet += analogRead(A0);
       FwdOffSet += analogRead(A1);
       delay(10);
     }
     RevOffSet =RevOffSet/20.0;
     FwdOffSet = FwdOffSet/20.0;
  // Calibration Complete   
  while(StartSwp){
       StartSwp = PrintNextPoint(StartSwp);
  }

  // Run Complete; Show Summary Report
    sendFrequency(1.0);  // set AD9850 output to 1 Hz
    char buf2 [32];
    char buf3 [32];
    double BestSWR = SweepData[1];
    char str_SWR[6];
    char str_Ohms[6];
    char str_LowFreq[6];
    char str_HiFreq[6];
    dtostrf(BestSWR, 3, 1, str_SWR);
    dtostrf(SweepData[4], 3, 0, str_Ohms);
    sprintf (buf2,"SWR:%s/%s Ohm", str_SWR, str_Ohms);

    dtostrf(SweepData[6]/1000000, 4, 2, str_LowFreq);
    dtostrf(SweepData[5]/1000000, 4, 2, str_HiFreq);
    sprintf (buf3,"L/H %s/%s Mhz", str_LowFreq, str_HiFreq);

    if(SweepData[2]> 0 && SweepData[3] >0){
      double centerFreq = (SweepData[2]+(SweepData[3]-SweepData[2])/2);
      char str_CFq[6];
      double CFqMhz = centerFreq/1000000;
      dtostrf(CFqMhz, 5, 3, str_CFq);// char *dtostrf(double val, signed char width, unsigned char prec, char *s)
      sprintf (buf,"Freq: %sMhz", str_CFq);
    }
    else sprintf (buf,"%s", "NO DATA AVAILABLE!");
    
    u8g.firstPage();
   do {
     int row = 11; //first OLED row to print on
     u8g.setFont(u8g_font_unifont);
     u8g.setPrintPos(0, row);
     u8g.print(buf);
     u8g.setFont(u8g_font_7x13);
     row = NuRow(row);
     u8g.setPrintPos(0, row);
     u8g.print(buf3);
     u8g.setFont(u8g_font_unifont);
     row = NuRow(row);
     u8g.setPrintPos(0, row);
     u8g.print(buf2);
     row = NuRow(row);
     u8g.setPrintPos(0, row);
     u8g.print("Hit 'Reset' Btn");
     row = NuRow(row);
     u8g.setPrintPos(0, row);
     u8g.print("to Scan Again");
    } while( u8g.nextPage() );

  attachInterrupt (InterruptId,isr,CHANGE);   //Re-Enable Rotory Sw  Use interrupt 0 [Uno] or 1 [Leonardo];

//  }
}  // End Main Loop Code

//*******************************************************//
bool PrintNextPoint(bool RunCurve){
  double FWD;
  double REV;
  double VSWR;
  double current_freq;
  double EffOhms;
  bool GudRun = false;
  bool Incrmnt = true;
  LastVSWR = 999;
  VSWRtrendUP = false;
  

//  Serial.println("StartRun");
  current_freq = Start_Freq;
// Prepare Array for new Run
    SweepData[1] = 999; //Lowest Recorded VSWR
    SweepData[2] = 0;
    SweepData[3] = 0;
    SweepData[4] = 9999; 
    SweepData[5] = 0;
    SweepData[6] = 0;
  
  while (RunCurve){
    
    sendFrequency(current_freq);  // freq
    delay(100);
     // Read the forward and reverse voltages
     REV = 0.0;
     FWD = 0.0;
     for (int i=0; i<70; i++) {
       REV += (analogRead(A0)-RevOffSet);
       FWD += (analogRead(A1)-FwdOffSet);
       
     }
     REV = REV/70.0;
     FWD = FWD/70.0;
    //REV = analogRead(A0)-RevOffSet;
    REV = (FwdOpAmpGain*REV)/RevOpAmpGain; // apply Op Amp Gain loop compensation
    REV = CorrectReading(REV);// now using table apply Small Signal correction value
    
    //FWD = analogRead(A1)-FwdOffSet;
    FWD = (RevSCVal*FWD)/FwdSCVal;// apply "Short Circuit" offset
    FWD = CorrectReading(FWD);// now using table apply Small Signal correction value
    if(REV>=FWD){
      // To avoid a divide by zero or negative VSWR then set to max 999
      VSWR = 999;
    }else{
      // Calculate VSWR
      
      VSWR = ((FWD+REV)/(FWD-REV));
    }
    if(FWD>=116) EffOhms = VSWR*47.0;//FWD>=94
    else  EffOhms = 47.0/VSWR;
    u8g.firstPage();
    // Post results
    do {
      int row = 11; //first OLED row to print on
      u8g.setFont(u8g_font_7x13);
      u8g.setPrintPos(0, row);
      u8g.print("Freq: ");
      u8g.print(int(current_freq/1000));
      u8g.print(" KHz");
      if (Incrmnt) u8g.print(" +");
      else u8g.print(" -");
      u8g.setFont(u8g_font_unifont);
      row = NuRow(row);
      u8g.setPrintPos(0, row);
      u8g.print("VSWR: ");
      u8g.print(VSWR);//LastVSWR
      if (VSWRtrendUP) u8g.print(" +");
      else u8g.print(" -");
      if( VSWR > VSWRLim)u8g.print("!!");
       else u8g.print("  ");
      row = NuRow(row);
      u8g.setPrintPos(0, row);
      u8g.print("Ohms: ");
      u8g.print(EffOhms);
      row = NuRow(row);
      u8g.setPrintPos(8, row);  
      u8g.print("Fwd: ");
      u8g.print(FWD);
      row = NuRow(row);
      u8g.setPrintPos(8, row);
      u8g.print("Rev: ");
      u8g.print(REV);
    } while( u8g.nextPage() );
    // Check for & Save interesting data
    if (VSWR < SweepData[1]) {
      if (Incrmnt){
        SweepData[1] = VSWR;
        SweepData[2] = current_freq;
        SweepData[4] = EffOhms;
      }
    }
    if (VSWR == SweepData[1]) {
      if (Incrmnt){
        SweepData[3] = current_freq;
      }
    }
    if (VSWR < LastVSWR-0.06){
      VSWRtrendUP = false;
       LastVSWR = VSWR;
    }

    if (VSWR > LastVSWR+0.06){
      VSWRtrendUP = true;
      LastVSWR = VSWR;
    }
    if( VSWR < VSWRLim) GudRun = true;
    if (Incrmnt && VSWR < VSWRLim ) SweepData[5] = current_freq; // capture Highest Freq that is below the VSWR limit
    if (!Incrmnt && VSWR < VSWRLim ) SweepData[6] = current_freq; // capture Lowest Freq that is below the VSWR limit 
    if (GudRun){
//      Serial.println("GudRun");
      if (Incrmnt && VSWRtrendUP && VSWR > VSWRLim )Incrmnt = false;
      else if (!Incrmnt && VSWRtrendUP && VSWR > VSWRLim ){
        Incrmnt = true;
        if (GudRun) RunCurve = false;
      }
    }
    if (current_freq >  End_Freq) Incrmnt = false;
    if (current_freq <  Start_Freq) RunCurve = false; // Exit The freqs of interest have been examined; But aparently a usable SWR was not found 
    if (Incrmnt) current_freq += Step_Freq;
    else current_freq -= Step_Freq;
    }// end While Loop
    return RunCurve;
}

double CorrectReading(double ReadVal){
//  Serial.println(ReadVal);
//  if (ReadVal > 263)return ReadVal;// if (ReadVal > 0)return ReadVal;//
//  return CrtdVal[ReadVal];
 if (ReadVal > 70)return 0.8*ReadVal+57;// if (ReadVal > 0)return ReadVal;//
 if (ReadVal < 13)return 3.6*(ReadVal*ReadVal)/15;
 float CalcVal =1.1*( 8+(2.1*ReadVal)-((ReadVal*ReadVal)*10.7/1000));
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
    case 1: //Set Freq Limits for 40_MTRS 
      Start_Freq = 6.4e6; // Freq; in Mhz
      End_Freq =  7.8e6; // Freq; in Mhz
      Step_Freq = 10e3; //Step  Freq; in KHz
      sprintf (str_Band, "40"); //str_Band = "40"
      break;
    case 2: //Set Freq Limits for 30_MTRS 
      Start_Freq = 9.6e6; // Freq; in Mhz  20.5e6;//  3.6e6;//
      End_Freq =  10.25e6; // Freq; in Mhz
      Step_Freq = 15e3; //Step  Freq; in KHz
      sprintf (str_Band, "30");
      break;
    case 3: //Set Freq Limits for 20_MTRS 
      Start_Freq = 13.4e6; // Freq; in Mhz  20.5e6;//  3.6e6;//
      End_Freq =  14.8e6; // Freq; in Mhz
      Step_Freq = 20e3; //Step  Freq; in KHz
      sprintf (str_Band, "20");
      break;
    case 4: //Set Freq Limits for 15_MTRS 
      Start_Freq = 20.5e6; // Freq; in Mhz
      End_Freq =  22.5e6; // Freq; in Mhz
      Step_Freq = 30e3; //Step  Freq; in KHz
      sprintf (str_Band, "15"); //str_Band = "15"
      break;
    case 5: //Set Freq Limits for 20_MTRS 
      Start_Freq = 27.4e6; // Freq; in Mhz  20.5e6;//  3.6e6;//
      End_Freq =  31.5e6; // Freq; in Mhz
      Step_Freq = 40e3; //Step  Freq; in KHz
      sprintf (str_Band, "10");
      break;  
//    default: 
//      // if nothing else matches, do the default
//      // default is optional
//    break;
   }
    
   double SFqMhz = Start_Freq/1e6;
   double EFqMhz = End_Freq/1e6; 
    
   dtostrf(SFqMhz, 4, 2, str_StrtFq);// char *dtostrf(double val, signed char width, unsigned char prec, char *s)
   dtostrf(EFqMhz, 4, 2, str_EndFq);
   dtostrf(VSWRLim, 4, 2, str_SWRLim);
   dtostrf(Step_Freq/1000, 4, 1, str_Step);
   sprintf (buf,"%s to %s Mhz", str_StrtFq,str_EndFq);
   sprintf (buf2,"MAX VSWR: %s", str_SWRLim);
   sprintf (buf3,"Step: %s Khz", str_Step);
   sprintf (buf4,"%sM Ant Analyzer", str_Band);
   u8g.firstPage();
   do {
    int row = 11; //first OLED row to print on
    u8g.setFont(u8g_font_unifont);
    u8g.setPrintPos(0, row);
    u8g.print(buf4);
    row = NuRow(row);
    u8g.setPrintPos(0, row);
    u8g.print(buf);
    row = NuRow(row);
    u8g.setPrintPos(0, row);
    u8g.print(buf2);
    row = NuRow(row);
    u8g.setPrintPos(0, row);
    u8g.print(buf3);
    u8g.setFont(u8g_font_7x13);
    row = NuRow(row);
    u8g.setPrintPos(0, row);
    u8g.print("AD9850; 128x64Disp");
  } while( u8g.nextPage() );
  //delay(5500);

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


