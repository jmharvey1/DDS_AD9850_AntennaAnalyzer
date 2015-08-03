/* 
 * A simple single freq AD9850 Arduino test script
 * Original AD9851 DDS sketch by Andrew Smallbone at www.rocketnumbernine.com
 * Modified for testing the inexpensive AD9850 ebay DDS modules
 * Pictures and pinouts at nr8o.dhlpilotcentral.com
 * 9850 datasheet at http://www.analog.com/static/imported-files/data_sheets/AD9850.pdf
 * Use freely
 */
 
 #define W_CLK 8       // Pin 8 - connect to AD9850 module word load clock pin (CLK)
 #define FQ_UD 9       // Pin 9 - connect to freq update pin (FQ)
 #define DATA 10       // Pin 10 - connect to serial data load pin (DATA)
 #define RESET 11      // Pin 11 - connect to reset pin (RST).
 
 #define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }

double Start_Freq =  2.0e6;// 6.8e6; //start Freq; in Mhz
double End_Freq = 23.0e6;//7.425e6; //End Scan Freq; in Mhz
double Step_Freq =  25000; //Step  Freq; in Hz
double current_freq;
int FwdOffSet;
int RevOffSet;
int FwdSCVal = 30;// initialially set to 1; then set to reading found when antenna leg of bridge is shorted; Diode sensitivity compensation
int RevSCVal = 40;// initialially set to 1; then set to reading found when antenna leg of bridge is shorted; Diode sensitivity compensation
int FwdOpAmpGain = 92;// initialially set to 1; then set to FWD reading found when Cathodes of D1 and D2 are shorted together; (Op Amp Gain loop compensation)
int RevOpAmpGain = 88;// initialially set to 1; then set to REV reading found when Cathodes of D1 and D2 are shorted together; (Op Amp Gain loop compensation)

// Small signal correction table [array] 
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
 // configure arduino data pins for output
  pinMode(FQ_UD, OUTPUT);
  pinMode(W_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(RESET, OUTPUT);
   
  pulseHigh(RESET);
  pulseHigh(W_CLK);
  pulseHigh(FQ_UD);  // this pulse enables serial mode - Datasheet page 12 figure 10
  current_freq = Start_Freq;
  Serial.begin(9600);
 }

void loop() {
  int incomingByte = 0;   // for incoming serial data
  bool RunCurve = false;
  
  if (Serial.available() > 0) {
    // read the incoming byte:
    while (Serial.available()){
      incomingByte = Serial.read();
      Serial.println(incomingByte);
    }
    RunCurve = !RunCurve;
  }
  else{
    Serial.println("Hit 'enter' Key to Start Scan");
    sendFrequency(1.0);  // set AD9850 output to 1 Hz
    delay(200);
    RevOffSet = analogRead(A0);
    FwdOffSet = analogRead(A1);
 
    delay(2000);
  }
  
  while (RunCurve){
     RunCurve = PrintNextPoint(RunCurve);
  }
}

//*******************************************************//
bool PrintNextPoint(bool RunCurve){
  double FWD = 0.0;
  double REV = 0.0;
  double VSWR;
  double EffOhms;
  if (current_freq > End_Freq){
      current_freq = Start_Freq;
      RunCurve = !RunCurve;
      Serial.println("Scan Complete");
     return RunCurve; 
    }
    sendFrequency(current_freq);  // freq
    delay(100);
     // Read the forawrd and reverse voltages
     for (int i=0; i<70; i++) {
       REV += (analogRead(A0)-RevOffSet);
       FWD += (analogRead(A1)-FwdOffSet);
       
     }
     REV = REV/70.0;
     FWD = FWD/70.0;    
    REV = (FwdOpAmpGain*REV)/RevOpAmpGain; // apply Op Amp Gain loop compensation
    REV = CorrectReading(REV);// now using table apply Small Signal correction value
    
    FWD = (RevSCVal*FWD)/FwdSCVal;// apply "Short Circuit" offset
    FWD = CorrectReading(FWD);// now using table apply Small Signal correction value
    if(REV>=FWD){
      // To avoid a divide by zero or negative VSWR then set to max 999
      VSWR = 999;
    }else{
      // Calculate VSWR
      
      VSWR = ((FWD+REV)/(FWD-REV));
    }
    if(FWD>=115
    
    ) EffOhms = VSWR*47.0;//FWD>=94
    else  EffOhms = 47.0/VSWR;
    // Send current line back to PC over serial bus
    Serial.print(int(current_freq/1000));
    Serial.print(",KHz VSWR:, ");
    Serial.print(VSWR); //Serial.print(int(VSWR*1000));
    Serial.print(", FWD: ");
    Serial.print(FWD);
    Serial.print(" REV: ");
    Serial.print(REV);
    Serial.print(" RevOffSet: ");
    Serial.print(RevOffSet);
    Serial.print(" FwdOffSet: ");
    Serial.print(FwdOffSet);
    Serial.print("; Ohms: ");
    Serial.print(EffOhms);
    Serial.println("");
    current_freq += Step_Freq;
    return RunCurve;
}

double CorrectReading(float ReadVal){
//  Serial.println(ReadVal);
//  if (ReadVal > 263)return ReadVal;// if (ReadVal > 0)return ReadVal;//
//  return CrtdVal[ReadVal];
 if (ReadVal > 70)return 0.8*ReadVal+57;// if (ReadVal > 0)return ReadVal;//
 if (ReadVal < 13)return 3.6*(ReadVal*ReadVal)/15;
 float CalcVal =1.1*( 8+(2.1*ReadVal)-((ReadVal*ReadVal)*10.7/1000));
 return CalcVal;
//if (ReadVal < 15) return 2.5*ReadVal;// if (ReadVal > 0)return ReadVal;//
// float CalcVal = (2.5*15)-(15*0.9)+(0.9*ReadVal);
//  return CalcVal;

  
}


