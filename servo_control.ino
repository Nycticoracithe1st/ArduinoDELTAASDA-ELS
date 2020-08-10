#include "ModbusXT.h"

#define TIMEOUT 500   //Timeout for a failed packet. Timeout need to larger than polling
#define POLLING 2     //Wait time to next request

#define BAUD        57600  
#define RETRIES     10    //How many time to re-request packet frome slave if request is failed
#define BYTE_FORMAT SERIAL_8E1
#define TxEnablePin 2   //Arduino pin to enable transmission

#define print(x)  Serial.print(x)
#define println(x) Serial.println(x)


//Name for register in regs[]
enum {
 //READ PACKET 1
 PPR1,//W40101 DWLOW R1 0
 PPR2,//W40102 DWHIGH R2 1
 LEFTMAX1,//W40103 DWLOW R3 2
 LEFTMAX2,//W40104 DWHIGH R4 3
 RIGHTMAX1,//W40105 DWLOW R5 4
 RIGHTMAX2,//W40106 DWHIGH R6 5
 TOOLLEAD,//W40107 W R7 6
 PULSEWIDTH,//W40108 W R8 7
 HOMESETZERO,//W40109 W R9 8
 BACKLASH,//W40110 W R10 9
 SETSPINDLEZERO,//W40111 W R11 10
 SETUPCOMPLETE,//W40112 W R12 11
 READYFORPASS,//W40113 W R13 12
 PASSDIRECTION,//W40114 W R14 13 0=left pass 1=right pass
 GOTOSTART,//W40115 R15 14 0=not ready 1=ready to pass left 2=ready to pass right
 //WRITE PACKET 2
 LEADSCREWPOS1,//W40121 DWLOW R16 15
 LEADSCREWPOS2,//W40122 DWHIGH R17 16
 SPINDLE_DEG1,//W40123 W R18 17
 SPINDLE_DEG2,//W40124 W R19 18 
 SPINDLE_RPM1,//W40125 W R20 19
 SPINDLE_RPM2,//W40126 W R21 20
 PASSCOMPLETE,//W40127 W R22 21
 TOOLPOSITION1,//W40128 DWLOW R23 22
 TOOLPOSITION2,//W40129 DWHIGH R24 23
 LEFTMAXWRITE1,//W40130 DWLOW R25 24
 LEFTMAXWRITE2,//W40131 DWHIGH R26 25
 RIGHTMAXWRITE1,//W40132 DWLOW R27 26
 RIGHTMAXWRITE2,//W40133 DWHIGH R28 27
 INPOSITION, // W40134 W R29 28 0 = not in position 1 in position left 2 in pos right 3 backlash complete
 PASSNUMBER,//W40135 w R30 29
 //WRITE PACKET 3
 total_packets,//W40141 W R31 30
 total_failed,//W40142 W R32 31
 total_requests,//W40143 W R33 32
 transfer_rate,//W40144 W R34 33
 transfer_delay,//W40145 W R35 34
 DRIVEJOG,//W40146 W R36 35
 ALARM,//W40147 W R37 36
  TOTAL_REGS //=35 (double words covers two addresses)
};

unsigned int holding1, holding2, holding3, holding4, holding5, holding6, holding7, holding8, holding9, holding10, holding11, holding12, holding13, holding14, holding15;
unsigned int holding1l, holding2l, holding3l, holding4l, holding5l, holding6l, holding7l, holding8l, holding9l, holding10l, holding11l, holding12l, holding13l, holding14l, holding15l;
unsigned int holding16, holding17, holding18, holding19, holding20, holding21, holding22, holding23, holding24, holding25, holding26, holding27, holding28, holding29, holding30,holding31;

// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.
enum {
  PACKET1,
  PACKET2,
  PACKET3,
  NO_OF_PACKET  //=3
};

// Masters register array
uint16_t regs[TOTAL_REGS];

//Modbus packet
Packet packets[NO_OF_PACKET];

// Access individual packet parameter. Uncomment it if you know what you're doing
// packetPointer packet1 = &packets[PACKET1];
// packetPointer packet2 = &packets[PACKET2];

long sm,em,dm;
uint16_t temp;
const uint8_t hmiID = 1;  //ID of HMI. The ID need to match, unless program will not work

//Modbus Master class define
Modbus master;  

//Motion control data
// Notes: Encoder wave form is square wave, A wave leads B wave in cw dirrection. B wave lags by 90degrees on wave form.
//Step & Drirection output   
  const int StepPulse = 52; //200K pps = 3000rpm
  const int DirPulse = 53; //Set drive direction, High-FWD, Low-RVS
  int PulseState;
 
//SP-Spindle input
  const int SPEncoderA = 21;//Spindle encoder A pulse (1024ppr)
  const int SPEncoderB = 51;//Spindle encoder B Pulse (1024ppr)
  const int SPEncoderZ = 20;//Spindle encoder Z pulse (1ppr)
  
//DR-Drive Encoder Input
  const int DREncoderA = 19;//Servo Drive feedback encoder (423ppr) Feed back resolution of 0.00500mm
  const int DREncoderB = 50;//Servo Drive feedback encoder (423ppr)

//Control-Box input
  const int DriveLeft = 48; //Jog left,run pass in feed mode, start threading in threading mode/or if reverse threadng return to start.
  const int DriveRight = 49; //Jog right, run pass in feed mode, return to start in threading mode/or if reverse threading start threading.
  int DriveLeftState = LOW;
  int DriveRightState = LOW;
  
//Spindle positioning and timing
  unsigned long SpindleRPM;
  int SPEncoderAstate;
  int SPEncoderBstate;
  int SPEncoderZstate;
  int SpindlePulseMicros; // time from A to A rising
  unsigned long SpindleMicros; //time for spindle to complete one full rotation. spindle pulse microseconds mul 1024
  long SPETimeA; //micros at A state change
  long SPETimeB; //micros at B state change
  unsigned int SpindlePositionABS = 0;
  long SpindlePositionABS100;

// Lead screw Drive positioning
  int DREncoderAstate;
  int DREncoderBstate;
  long DriveCurrentPos;
  int BackLash;
  int BackLash2;
  long LeadScrewCurrentPos;
  long LeadScrewLeftMAX;
  long LeadScrewRightMAX;
  long LeadScrewReturnToLead;
  int SquareWaveHILO; //Time for square wave of step
  int HalfSquareWave; //Half square wave time
  long PreviousMicros;
  long CurrentMicros;


//HMI data
  int PassNumber; //passes done
  long sppsr; //step pulses per spindle revolution

//prog ints
  int RUN_SERVO = 0;
  int RUN_SERVOL = 0;
  int RUN_SERVOR = 0;
  int RUN_COMS = 1;
  int PassComplete = 0;
  long Millis, Millis1, Millis2, Millis3;
  int Positioning1 = 0;
  int Positioning2 = 0;
  int INPOSITIONL = 0;
  int INPOSITIONR = 0;
  int RUNTOSTART = 0;
  int READY_FOR_PASSR = 0;
  int READY_FOR_PASSL = 0;
  int BACKLASHTESTING, BACKLASHTEST, BACKLASHRETURN;
  int SpindleDir = 0;
  unsigned long SpindleDeg;
  int run1;
  int ComsStartmillis = 0;
  int ComsStartmillis1 = 0;
  int COMS_StartUp = 0;
 

void setup() {

// Servo control set up  
  pinMode (StepPulse, OUTPUT);
  pinMode (DirPulse, OUTPUT);
  pinMode (SPEncoderA, INPUT);
  pinMode (SPEncoderB, INPUT);
  pinMode (SPEncoderZ, INPUT);
  pinMode (DREncoderA, INPUT);
  pinMode (DREncoderB, INPUT);
  pinMode (DriveLeft, INPUT);
  pinMode (DriveRight, INPUT);

  attachInterrupt(digitalPinToInterrupt(SPEncoderA), SPINDLE_TIME_POSITION, RISING);
  attachInterrupt(digitalPinToInterrupt(DREncoderA), DRIVE_POS, RISING);

// RS 485 SetUp ALL MOVED TO VOID COMS_BEGIN

 //Config packets and register
 // master.configure(packets, NO_OF_PACKET, regs);

  //Config individual packet: (packet, ID, Function, Address, Number of register or data, start register in master register array)
  //master.construct(&packets[PACKET1], hmiID, READ_HOLDING_REGISTERS, 100, 15, 0);
 // master.construct(&packets[PACKET2], hmiID, PRESET_MULTIPLE_REGISTERS, 120, 15, 15);
 // master.construct(&packets[PACKET3], hmiID, PRESET_MULTIPLE_REGISTERS, 140, 7, 30);

  //Start Modbus
  //master.begin(&Serial3, BAUD, BYTE_FORMAT, TIMEOUT, POLLING, RETRIES, TxEnablePin); moved to "void COMS_BEGIN()"


  RUN_COMS = 1;
  PassComplete = 0;
  RUN_SERVOR = 0;
  RUN_SERVOL = 0;
  holding31 = 0;

  Serial.begin (57600);

}

void loop() {
  SquareWaveHILO = SpindleMicros/sppsr;//SPPSR-Servo Pulse Per spindle Revolution. Source HMI
  HalfSquareWave = SquareWaveHILO/2;

   //RUN ServoLEFT
 if (RUN_SERVOL == 1){
  SERVOPULSEFWD();
 }
  //RUN ServoRIGHT
 if (RUN_SERVOR == 1){
  SERVOPULSERVS();
 }

 if((RUN_SERVOL == 0) && (RUN_SERVOR == 0)){
  PROG_SEL();
 }

 if(PassComplete==1){
  RESET();
 }

}

void RESET(){
Millis2 = millis();
if( run1 == 0){
 RUN_SERVOL = 0;
 RUN_SERVOR = 0;
 READY_FOR_PASSR = 0;
 READY_FOR_PASSL = 0;
 INPOSITIONR = 0;
 INPOSITIONL = 0;
 holding13 = 0;
 holding21 = 0;
 holding29 = 0;
 detachInterrupt(digitalPinToInterrupt(SPEncoderA));//detach SPINDLE_TIME interrupt
 attachInterrupt(digitalPinToInterrupt(SPEncoderA), SPINDLE_TIME_POSITION, RISING);//attach SPINDLE_TIME_POSITION interrupt
 COMS_StartUp = 0; //Restart Serial
 RUN_COMS = 1;
 holding30++;
 Millis3 = Millis2;
 run1 = 1;
 }
 if(Millis2 - Millis3 > 1000){
 PassComplete = 0;
 run1 = 0; 
 }  
}

void PROG_SEL(){
  SpindleRPM = 60000000 / SpindleMicros;
  SpindlePositionABS100 = SpindlePositionABS * 100;//Improve degree accuracy
  BackLash2 = BackLash * 2;
  SpindleDeg = SpindlePositionABS100 / 284;//1 degree = 2.84 pulses from the spindle
  //JOG/RUN button inputs
  DriveLeftState = digitalRead(DriveLeft);
  DriveRightState = digitalRead(DriveRight);

  //SET HOME FROM HMI
  if(holding9==1){DriveCurrentPos = 0;}
  //SET LEFT MAX FROM HMI using drives current position
  if(holding9==2){LeadScrewLeftMAX = DriveCurrentPos;}
  //SET LEFT MAX FROM HMI using drives current position
  if(holding9==3){LeadScrewRightMAX = DriveCurrentPos;}
  //RUN BACKLASH TEST
  if((holding9==4)&&(BACKLASHTEST == 0)){BACKLASHTESTING = 1;}
  //Stop backlash testing from HMI
  if(holding9==5){BACKLASHTESTING = 0;BACKLASHRETURN = 0;BACKLASHTEST = 0;}
  //Reset after backlash testing
  if(BACKLASHTEST == 1){BACKLASHTEST = 0;}
  //ZERO SPINDLE POSITION FROM HMI
  if(holding9==6){SpindlePositionABS = 0;}
  //Further QOL to be implemented here



  //JOG L/R/stop
  if((holding12==0)&&(DriveLeftState==HIGH)){holding31=1;}//HMI TO RUN DRIVE LEFT RW405=4999
  if((DriveRightState==HIGH)&&(holding12==0)){holding31=2;}//HMI TO RUN DRIVE RIGHT RW405=4998
  if((DriveRightState==LOW)&&(DriveLeftState==LOW)){holding31=0;}//HMI TO STOP DRIVE RW405=5000
  
//Go to Start pos RIGHT
  if (holding15==2){
    if(DriveLeftState == HIGH){
      RUNTOSTART = 1;
    }
// Start RIGHT pass   
    if(holding13==2){
    if(DriveRightState == HIGH){
    if(INPOSITIONR == 1){
     RUN_COMS = 0;
     //Shut down Modbus protocol here
     digitalWrite(DirPulse, LOW);//Set direction of drive for pass
     READY_FOR_PASSR = 1;
     detachInterrupt(digitalPinToInterrupt(SPEncoderA));//Detach SPINDLE_TIME_POSITION interrupt
     attachInterrupt(digitalPinToInterrupt(SPEncoderA), SPINDLE_TIME, RISING);//attach SPINDLE_TIME interrupt
     attachInterrupt(digitalPinToInterrupt(SPEncoderZ), SPINDLE_ZERO, RISING);
      }else{holding22=2;} // NOT IN RIGHT POSTION ALARM
     }
    }
   }
//GO to Start pos LEFT  
  if (holding15==1){
    if(DriveRightState == HIGH){
      RUNTOSTART = 1;
    }
//Start LEFT pass    
    if(holding13==1){
    if(DriveLeftState == HIGH){
    if(INPOSITIONL == 1){
     RUN_COMS = 0;
    //Shut down Modbus protocal here
     digitalWrite(DirPulse, HIGH);// Set Dricetion of drive for pass
     READY_FOR_PASSL = 1;
     detachInterrupt(digitalPinToInterrupt(SPEncoderA));//Detach SPINDLE_TIME_POSITION interrupt
     attachInterrupt(digitalPinToInterrupt(SPEncoderA), SPINDLE_TIME, RISING);//attach SPINDLE_TIME interrupt
     attachInterrupt(digitalPinToInterrupt(SPEncoderZ), SPINDLE_ZERO, RISING);
      }else{holding22=1;} // NOT IN LEFT POSTION ALARM
     }
    }
   }
  //RUN BACKLASH TESTING
  if(BACKLASHTESTING == 1){BACKLASHTESTER();}
  //RUN RS485
  if (RUN_COMS==1){COMS();}
  //SET START POSITION
  if(RUNTOSTART== 1){GO_TO_START();}
}
//INTERRUPT FUNCTIONS

void SPINDLE_TIME_POSITION(){ 
      SPETimeA = SPETimeB;
      SPETimeB = micros();
      SPEncoderBstate = digitalRead(SPEncoderB);
      if (SPEncoderBstate == HIGH){
        SpindlePositionABS++;
        SpindlePositionABS = SpindlePositionABS & 0x3FF;//bitmask to allow max value 1023
      }else{
       SpindlePositionABS--;
       SpindlePositionABS = SpindlePositionABS & 0x3FF;
      }
  SpindlePulseMicros = SPETimeB - SPETimeA;
  SpindleMicros = SpindlePulseMicros * 1024;
}
void SPINDLE_TIME(){ 
  SPETimeA = SPETimeB;
  SPETimeB = micros();
  SpindlePulseMicros = SPETimeB - SPETimeA;
  SpindleMicros = SpindlePulseMicros * 1024;
}
void SPINDLE_ZERO(){
 if(READY_FOR_PASSR == HIGH){
  digitalWrite(RUN_SERVOR, HIGH);  
 }
 if(READY_FOR_PASSL == HIGH){
  digitalWrite(RUN_SERVOL, HIGH);  
 }
 detachInterrupt(digitalPinToInterrupt(SPEncoderZ));
}
void DRIVE_POS(){
  DREncoderBstate = digitalRead(DREncoderB);
    if (DREncoderBstate != HIGH){
        DriveCurrentPos++;
      }else{
        DriveCurrentPos--;
      }  
}
    
void SERVOPULSEFWD(){
  CurrentMicros = micros();
 if(LeadScrewLeftMAX > LeadScrewCurrentPos){
 if(PassComplete == 0){
  if(CurrentMicros - PreviousMicros >= HalfSquareWave) {
    PreviousMicros = micros();
    if (PulseState == LOW){
      PulseState = HIGH;
    } else{
      PulseState = LOW;
    }
  }
 }
}
digitalWrite(StepPulse, PulseState);
if(LeadScrewLeftMAX <= LeadScrewCurrentPos){
  PassComplete = 1;
 }
}
void SERVOPULSERVS(){
 CurrentMicros = micros();
 if(LeadScrewRightMAX < LeadScrewCurrentPos){
 if(PassComplete == 1){
  if(CurrentMicros - PreviousMicros >= HalfSquareWave) {
    PreviousMicros = micros();
    if (PulseState == LOW){
      PulseState = HIGH;
    } else{
      PulseState = LOW;
    }
  }
 }
}
digitalWrite(StepPulse, PulseState);
if(LeadScrewRightMAX >= LeadScrewCurrentPos){
  PassComplete = 1;
 }
}

void GO_TO_START(){
  
 if(holding14==0){//left pass
   if(LeadScrewCurrentPos>=0){
    holding31 = 1; 
   }else{
   holding31 = 0;
   SLOW_STEPL();
   }
  }
 if(holding14==1){//right pass
   if(LeadScrewCurrentPos<=0){
    holding31 = 2;
   }else{
   holding31 = 0;
   SLOW_STEPR();
   }
  }
}

void SLOW_STEPL(){//removes backlash and prepares for start possition for a right to left run
  if(DriveCurrentPos>=0+LeadScrewReturnToLead){
  if(Positioning2== HIGH){
    INPOSITIONL = 1;
    holding29= 1;
    RUNTOSTART = 0;
    Positioning1 = 0;
    Positioning2= 0;    
    } 
  }
  if(DriveCurrentPos>= LeadScrewReturnToLead-BackLash2){
  Positioning1 = 1;}
  if(DriveCurrentPos<= LeadScrewReturnToLead-BackLash2){
  Positioning2 = 1;
  Positioning1 = 0;}

  if(Positioning1==1){
    digitalWrite(DirPulse, LOW);}
  if(Positioning2==1){
    digitalWrite(DirPulse, HIGH);}
    
 if(INPOSITIONL==0){
  CurrentMicros = micros();     
  if(CurrentMicros - PreviousMicros >= 1600) {//servo running at 24rpm
    PreviousMicros = micros();
    if (PulseState == LOW){
      PulseState = HIGH;
    } else{
      PulseState = LOW;
    }
   }
  }
digitalWrite(StepPulse, PulseState);
}
void SLOW_STEPR(){//removes backlash and prepares for start possition for a left to right run
  if(DriveCurrentPos<=0+LeadScrewReturnToLead){
  if(Positioning2== 1){
    INPOSITIONR = 1;
    holding29= 2;
    RUNTOSTART = 1;
    Positioning1 = 0;
    Positioning2 = 0;
    } 
  }
  if(DriveCurrentPos<= LeadScrewReturnToLead+BackLash2){
  Positioning1 = 1;}
  if(DriveCurrentPos>= LeadScrewReturnToLead+BackLash2){
  Positioning2= 1;
  Positioning1 = 0;}

  if(Positioning1==HIGH){
    digitalWrite(DirPulse, HIGH);}
  if(Positioning2==HIGH){
    digitalWrite(DirPulse, LOW);}
    
 if(INPOSITIONR==0){
  CurrentMicros = micros();     
  if(CurrentMicros - PreviousMicros >= 1600) {
    PreviousMicros = micros();
    if (PulseState == LOW){
      PulseState = HIGH;
    } else{
      PulseState = LOW;
    }
   }
  }
digitalWrite(StepPulse, PulseState);
}
void BACKLASHTESTER(){//BACKLASH TEST to be run from drive position 0
  CurrentMicros = micros();
  if((DriveCurrentPos< 4000)&&(BACKLASHTEST == 0)){//run forward 20mm
   digitalWrite(DirPulse, HIGH);
  }
  if((DriveCurrentPos>= 4000)&&(BACKLASHTEST == 0)){//reverse 20mm
   BACKLASHRETURN = 1; 
  }
  if(BACKLASHRETURN == 1){
    digitalWrite(DirPulse, LOW);
  }
  if((BACKLASHRETURN == 1) && (DriveCurrentPos <= 0)){
   digitalWrite(DirPulse, HIGH);
   BACKLASHRETURN = 2;
  }
  if((BACKLASHRETURN == 2) && (DriveCurrentPos >= 4000)){
   BACKLASHTEST = 1;
   BACKLASHRETURN = 0;
   BACKLASHTESTING = 0;
  }
   if(BACKLASHTESTING==1){     
    if(CurrentMicros - PreviousMicros >= holding8) {
     PreviousMicros = CurrentMicros;
      if (PulseState == LOW){
       PulseState = HIGH;
       }else{
        PulseState = LOW;
    }
   }
  }
digitalWrite(StepPulse, PulseState);
}
void COMS_BEGIN(){
  //Config packets and register
 master.configure(packets, NO_OF_PACKET, regs);

  //Config individual packet: (packet, ID, Function, Address, Number of register or data, start register in master register array)
  master.construct(&packets[PACKET1], hmiID, READ_HOLDING_REGISTERS, 100, 15, 0);
  master.construct(&packets[PACKET2], hmiID, PRESET_MULTIPLE_REGISTERS, 120, 15, 15);
  master.construct(&packets[PACKET3], hmiID, PRESET_MULTIPLE_REGISTERS, 140, 7, 30);
  master.begin(&Serial3, BAUD, BYTE_FORMAT, TIMEOUT, POLLING, RETRIES, TxEnablePin);
  COMS_StartUp = 2;
  
}
void COMS(){
  if(COMS_StartUp < 2){
  ComsStartmillis = millis();
   if(COMS_StartUp == 0){
    ComsStartmillis1 = millis();
    COMS_StartUp = 1;
    }
   if((COMS_StartUp == 1) && (ComsStartmillis - ComsStartmillis1 > 30000)){//Allow HMI time to boot after power up before enabling coms
    COMS_BEGIN();
    }
  }
     
   master.update();  //polling
   sm = millis();
   //READS
   holding1 = regs[PPR1];
   holding2 = regs[PPR2];
   holding3 = regs[LEFTMAX1];
   holding4 = regs[LEFTMAX2];
   holding5 = regs[RIGHTMAX1];
   holding6 = regs[RIGHTMAX2];
   holding7 = regs[TOOLLEAD];
   holding8 = regs[PULSEWIDTH];
   holding9 = regs[HOMESETZERO];
   holding10 = regs[BACKLASH];
   holding11 = regs[SETSPINDLEZERO];
   holding12 = regs[SETUPCOMPLETE];
   holding13 = regs[READYFORPASS];
   holding14 = regs[PASSDIRECTION];
   holding15 = regs[GOTOSTART];
   sppsr = holding2;
   sppsr = (sppsr << 16) | holding1;
   BackLash = holding10;
   LeadScrewReturnToLead = holding7;
   LeadScrewLeftMAX = holding4;
   LeadScrewLeftMAX = (LeadScrewLeftMAX << 16) | holding3;
   LeadScrewRightMAX = holding6;
   LeadScrewRightMAX = (LeadScrewRightMAX << 16) | holding5;
   //WRITES
   holding16 = (int)DriveCurrentPos;
   holding17 = (int)(DriveCurrentPos >> 16);
   holding18 = (int)SpindleDeg;
   holding19 = (int)(SpindleDeg >> 16);
   holding20 = (int)SpindleRPM;
   holding21 = (int)(SpindleRPM >> 16);
   holding22 = PassComplete;
   holding23 = (int)DriveCurrentPos;
   holding24 = (int)(DriveCurrentPos >> 16);
   holding25 = (int)LeadScrewLeftMAX;
   holding26 = (int)(LeadScrewLeftMAX >> 16);
   holding27 = (int)LeadScrewRightMAX;
   holding28 = (int)(LeadScrewRightMAX >> 16);
   regs[LEADSCREWPOS1] = holding16;
   regs[LEADSCREWPOS2] = holding17;
   regs[SPINDLE_DEG1] = holding18;
   regs[SPINDLE_DEG2] = holding19;
   regs[SPINDLE_RPM1] = holding20;
   regs[SPINDLE_RPM2] = holding21;
   regs[PASSCOMPLETE] = holding22;
   regs[TOOLPOSITION1] = holding23;
   regs[TOOLPOSITION2] = holding24;
   regs[LEFTMAXWRITE1] = holding25;
   regs[LEFTMAXWRITE2] = holding26;
   regs[RIGHTMAXWRITE1] = holding27;
   regs[RIGHTMAXWRITE2] = holding28;
   regs[INPOSITION] = holding29;
   regs[PASSNUMBER] = holding30;
   regs[total_packets] = NO_OF_PACKET;            
   regs[total_requests] = master.total_requests();
   regs[total_failed] = master.total_failed(); 
   regs[DRIVEJOG] = holding31;    
  if ( (sm-dm) > 1000) //update 1s
   {
     dm = sm;
     regs[transfer_rate] = regs[total_requests] - temp;
     temp = regs[total_requests];
     regs[transfer_delay] = (unsigned int) ((NO_OF_PACKET*100000UL)/regs[transfer_rate]);
   }
}
