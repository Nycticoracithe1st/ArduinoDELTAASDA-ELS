#include "ModbusXT.h"

#define TIMEOUT 500   //Timeout for a failed packet. Timeout need to larger than polling
#define POLLING 2     //Wait time to next request

#define BAUD        57600  
#define RETRIES     10    //How many time to re-request packet frome slave if request is failed
#define BYTE_FORMAT SERIAL_8E1
#define TxEnablePin 2   //Arduino pin to enable transmission

#define print(x)  Serial.print(x)
#define println(x) Serial.println(x)

// Information I want to input from HMI

//Name for register in regs[]
enum {
 //READ PACKET 1
 PPR,//W40001 DW R1
 //W40002
 LEFTMAX,//W40003 DW R2
 //W40004
 RIGHTMAX,//W40005 DW R3
 //W40006
 TOOLLEAD,//W40007 W R4
 SPINDLESTARTPOS,//W40008 W R5
 HOMESETZERO,//W40009 W R6
 BACKLASH,//W40010 W R7
 SETSPINDLEZERO,//W40011 W R8
 SETUPCOMPLETE,//W40012 W R9
 READYFORPASS,//W40013 W R10
 JOGMODE,//W40014 W R11 0= Manual mode speed 1= Inching Speed 2= Rapids
 DOUBLEWORDTEST1,//W40015 DW R12
 //W40016
 PASSDIRECTION,//W40017 W R13 0=left pass 1=right pass
 GOTOSTART,
 //WRITE PACKET 2
 LEADSCREWPOS,//W40101 DW R14
 //W40102
 SPINDLE_DEG,//W40103 W R15
 SPINDLE_RPM,//W40104 W R16
 DRIVEJOG,//W40105 W R17 (0=speed setting 1)(1=speed setting 2)(2=speed setting 3) (3=Speed setting 4)(4= RUN left)(5= RUN right)(6=STOP)(7=HMI CONTROL)
 total_packets,//W40106 W R18
 total_failed,//W40107 W R19
 total_requests,//W40108 W R20
 transfer_rate,//W40109 W R21
 transfer_delay,//W40110 W R22
 DOUBLEWORDTEST2,//W40111 DW R23
 PASSCOMPLETE,//W40112 W R24
 ALARM,//W40113
  TOTAL_REGS //=25 (double words covers two addresses)
};

// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.
enum {
  PACKET1,
  PACKET2,
  NO_OF_PACKET  //=2
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
  const int StepPulse = 53; //Increment encoder drive one pulse (1 pulse = 0.00021167mm)
  const int DirPulse = 52; //Set drive direction, High-RVS, Low-FWD
  int PulseState;
 
//SP-Spindle input
  const int SPEncoderA = 21;//Spindle encoder A pulse (1024ppr)
  const int SPEncoderB = 51;//Spindle encoder B Pulse (1024ppr)
  const int SPEncoderZ = 20;//Spindle encoder Z pulse (1ppr)
  
//DR-Drive Encoder Input
  const int DREncoderA = 19;//Servo Drive feedback encoder (2500ppr)
  const int DREncoderB = 47;//Servo Drive feedback encoder (2500ppr)

//Control-Box input
  const int DriveLeft = 46; //Jog left,run pass in feed mode, start threading in threading mode/or if reverse threadng return to start.
  const int DriveRight = 45; //Jog right, run pass in feed mode, return to start in threading mode/or if reverse threading start threading.
  int DriveLeftState = LOW;
  int DriveRightState = LOW;
  
//Spindle positioning and timing
  int SpindleRPM;
  int SPEncoderAstate;
  int SPEncoderBstate;
  int SPEncoderZstate;
  int SPEncoderAlaststate = LOW;
  int SPEncoderBlaststate;
  int SpindlePulseMicros; // time from A pulse state change to B pulse state change
  unsigned long SpindleMicros; //time for spindle to complete one full rotation. spindle pulse microseconds mul 4096
  long SPETimeA; //micros at A state change
  long SPETimeB; //micros at B state change
  int SpindlePositionABS;

// Lead screw Drive positioning
  int DREncoderAstate;
  int DREncoderBstate;
  int DREncoderAlaststate;
  int DREncoderBlaststate;
  long DriveCurrentPos;
  int BackLash;
  long LeadScrewCurrentPos;
  long LeadScrewLeft;
  long LeadScrewLeftMAX;
  long LeadScrewRight;
  long LeadScrewRightMAX;
  long LeadScrewReturnToLead; // new postion to allow for correct tool pressure during threading, = leadscrewreturnto + toolanglelead
  int SquareWaveHILO; //Time for square wave of step
  int SquareWave; //Half square wave time
  long PreviousMicros;
  long CurrentMicros;

//Pulse calculation
  long spm; // step pulse micros = spindlemicros / step pulses per spindle revolution (sppsr, source:HMI) Time for each pulse for the servo step. SPM/2 for square wave

//HMI data
  int RunDirection; //Run Direction of operation0=forward 1=reverse, this will allow for rapid return to start pos
  int PassNumber; //passes done
  int FeedMode; //0-threads 1-feed
  int ToolAngleLead; //Amount to advance tool before each cut to keep cutting on front of tool (only applies to threading)
  long sppsr; //step pulses per spindle revolution

//prog ints
  int RUN_SERVO = LOW;
  int RUN_SERVOL = LOW;
  int RUN_SERVOR = LOW;
  int RUN_COMS = HIGH;
  long DoubleWordTestHolder;
  int PassComplete = LOW;
  int START_PASS = LOW;
  int JOG_L = LOW;
  int JOG_R = LOW;
  int JTSRJOG =LOW;
  int JTSLJOG = LOW;
  int LSDIR = LOW;
  int DRIVEJOG1 = LOW;
  int DRIVEJOG2 = LOW;
  int DRIVEJOG3 = LOW;
  int DRIVEJOG4 = LOW;
  int DRIVEJOG5 = LOW;
  int DRIVEJOG6 = LOW;
  long Millis;
  long Millis1;
  int Positioning1 = LOW;
  int Positioning2 = LOW;
  int INPOSITIONL = LOW;
  int INPOSITIONR = LOW;
  int RUNTOSTART = LOW;
  int READY_FOR_PASSR = LOW;
  int READY_FOR_PASSL = LOW;

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

  attachInterrupt(digitalPinToInterrupt(SPEncoderAstate), SPINDLE_TIME, HIGH);
  attachInterrupt(digitalPinToInterrupt(SPEncoderZstate), SPINDLE_ZERO, HIGH);
  attachInterrupt(digitalPinToInterrupt(DREncoderAstate), DRIVE_POS, HIGH);

// RS 485 SetUp

 //Config packets and register
  master.configure(packets, NO_OF_PACKET, regs);

  //Config individual packet: (packet, ID, Function, Address, Number of register or data, start register in master register array)
  master.construct(&packets[PACKET1], hmiID, READ_HOLDING_REGISTERS, 0, 14, 0);

  master.construct(&packets[PACKET2], hmiID, PRESET_MULTIPLE_REGISTERS, 100, 12, 14);

  //Start Modbus
  master.begin(&Serial, BAUD, BYTE_FORMAT, TIMEOUT, POLLING, RETRIES, TxEnablePin);



}

void loop() {
  SquareWaveHILO = SpindleMicros/sppsr;
  SquareWave = SquareWaveHILO/2;
//Backlash correction
  if(LSDIR==HIGH){
   LeadScrewCurrentPos = DriveCurrentPos + BackLash;  
  }else{LeadScrewCurrentPos = DriveCurrentPos - BackLash;}
    

   //RUN ServoLEFT
 if (RUN_SERVOL == HIGH){
  SERVOPULSEFWD();
 }
  //RUN ServoRIGHT
 if (RUN_SERVOR == HIGH){
  SERVOPULSERVS();
 }

 if(RUN_SERVOL == LOW && RUN_SERVOR == LOW){
  PROG_SEL();
 }

 if(PassComplete==HIGH){
  RESET();
 }
}

void RESET(){
digitalWrite(RUN_SERVOL, LOW);
digitalWrite(RUN_SERVOR, LOW);
digitalWrite(READY_FOR_PASSR, LOW);
digitalWrite(READY_FOR_PASSL, LOW);
digitalWrite(INPOSITIONR, LOW);
digitalWrite(INPOSITIONL, LOW);
regs[READYFORPASS] = 0;
regs[PASSCOMPLETE] = 0;
digitalWrite(JOG_L, HIGH);
digitalWrite(JOG_R, HIGH);
digitalWrite(RUN_COMS, HIGH);
digitalWrite(PassComplete, LOW);   
}

void PROG_SEL(){
  DoubleWordTestHolder = regs[DOUBLEWORDTEST1];
  regs[DOUBLEWORDTEST2] = DoubleWordTestHolder + 5;
  SpindleRPM = 60000000 / SpindleMicros;
  regs[SPINDLE_RPM] = SpindleRPM;
  //JOG/RUN button inputs
  DriveLeftState = digitalRead(DriveLeft);
  DriveRightState = digitalRead(DriveRight);

  if(regs[SETUPCOMPLETE]==0){
    if(DriveLeftState==HIGH){
      JOG_LEFT();
    }else{regs[DRIVEJOG]=7;
      digitalWrite(DRIVEJOG1, LOW);
          }
    if(DriveRightState==HIGH){
     JOG_RIGHT();
    }else{regs[DRIVEJOG]=7;
     digitalWrite(DRIVEJOG1, LOW);}
  }

  if (regs[GOTOSTART]==2){
    if(DriveLeftState == HIGH){
      digitalWrite(RUNTOSTART, HIGH);
    }
   if(DriveRightState == HIGH){
    if(INPOSITIONR == HIGH){
   digitalWrite(RUN_COMS, LOW);
   digitalWrite(JOG_L, LOW);
   digitalWrite(JOG_R, LOW);
   digitalWrite(READY_FOR_PASSR, HIGH);
     }else{regs[ALARM]=2;} // NOT IN POSTION ALARM
    }
   }
   
  if (regs[GOTOSTART]==1){
    if(DriveRightState == HIGH){
      digitalWrite(RUNTOSTART, HIGH);
    }
   if(DriveLeftState == HIGH){
    if(INPOSITIONL == HIGH){
   digitalWrite(RUN_COMS, LOW);
   digitalWrite(JOG_L, LOW);
   digitalWrite(JOG_R, LOW);
   digitalWrite(READY_FOR_PASSL, HIGH);
     }else{regs[ALARM]=1;} // NOT IN POSTION ALARM
    }
   }
    
  //RUN RS485
 if (RUN_COMS==HIGH){
  COMS();
 }
 //RUN JOG LEFT
 if (JOG_L == HIGH){
  JOG_LEFT();
 } 
 //RUN JOG RIGHT
 if (JOG_R == HIGH){
  JOG_RIGHT();
 }

 if(RUNTOSTART== HIGH){
  GO_TO_START();
  }
 }

//INTERRUPT FUNCTIONS
void SPINDLE_TIME(){ 
      SPETimeA = SPETimeB;
      SPETimeB = micros();
      SPEncoderAstate = digitalRead(SPEncoderA);
      SPEncoderBstate = digitalRead(SPEncoderB);
      if (SPEncoderAstate != SPEncoderBstate){
        SpindlePositionABS++;
      }else{
       SpindlePositionABS--;
      }
  SpindlePulseMicros = SPETimeB - SPETimeA;
  SpindleMicros = SpindlePulseMicros * 1024;
     }

void SPINDLE_ZERO(){
 SpindlePositionABS = 0;
 if(READY_FOR_PASSR == HIGH){
  digitalWrite(RUN_SERVOR, HIGH);  
 }
 if(READY_FOR_PASSL == HIGH){
  digitalWrite(RUN_SERVOL, HIGH);  
 }
}
  
void DRIVE_POS(){
  DREncoderAstate = digitalRead(DREncoderA);
  DREncoderBstate = digitalRead(DREncoderB);
    if (DREncoderAstate != DREncoderBstate){
        DriveCurrentPos++;
        digitalWrite(LSDIR, LOW);//Store Lead Screw DIRection for backlash
      }else{
        DriveCurrentPos--;
        digitalWrite(LSDIR, HIGH);//Store Lead Screw DIRection for backlash
      }  
}
    
void SERVOPULSEFWD(){
digitalWrite(DirPulse, HIGH);
 if(LeadScrewLeftMAX >= LeadScrewCurrentPos){
 if(PassComplete == LOW){
  if(micros() - PreviousMicros >= SquareWave) {
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
  PassComplete = HIGH;
 }
}
void SERVOPULSERVS(){
digitalWrite(DirPulse, LOW);
 if(LeadScrewRightMAX <= LeadScrewCurrentPos){
 if(PassComplete == LOW){
  if(micros() - PreviousMicros >= SquareWave) {
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
  PassComplete = HIGH;
 }
}
//Run jog function dependant on state from HMI
void JOG_LEFT(){
 Millis = millis();
  if(DRIVEJOG1 == LOW){
  Millis1 = millis();
  }
    if(Millis - Millis1 < 800){
    regs[DRIVEJOG] = 3;//select speed
    digitalWrite(DRIVEJOG1, HIGH);
     if(Millis - Millis1 > 800){
     regs[DRIVEJOG] = 4; 
     }
    }
   }
void JOG_RIGHT(){
 Millis = millis();
  if(DRIVEJOG1 == LOW){
  Millis1 = millis();
  }
    if(Millis - Millis1 < 800){
    regs[DRIVEJOG] = 3;//select speed
    digitalWrite(DRIVEJOG1, HIGH);
     if(Millis - Millis1 > 800){
     regs[DRIVEJOG] = 5; 
     }
    }
}

void GO_TO_START(){
  Millis = millis();
  if(DRIVEJOG3 == LOW){
  Millis1 = millis();
  }
 if(regs[PASSDIRECTION]==0){
   if(LeadScrewCurrentPos>=0){
    if(Millis - Millis1 < 800){
    regs[DRIVEJOG] = 3;
    digitalWrite(DRIVEJOG3, HIGH);
     if(Millis - Millis1 > 800){
     regs[DRIVEJOG] = 5; 
     }
    }
   }else{
   regs[DRIVEJOG] = 6;
   SLOW_STEPL();
   }
  }
 if(regs[PASSDIRECTION]==1){
   if(LeadScrewCurrentPos<=0){
    if(Millis - Millis1 < 800){
    regs[DRIVEJOG] = 3;
    digitalWrite(DRIVEJOG3, HIGH);
     if(Millis - Millis1 > 800){
     regs[DRIVEJOG] = 4; 
     }
    }
   }else{
   regs[DRIVEJOG] = 6;
   SLOW_STEPR();
   }
  }
}

void SLOW_STEPL(){//removes backlash and prepares for start possition for a right to left run
  if(LeadScrewCurrentPos>=0+LeadScrewReturnToLead){
  if(Positioning2== HIGH){
    digitalWrite(INPOSITIONL, HIGH);
    digitalWrite(RUNTOSTART, LOW);
    digitalWrite(Positioning1, LOW);
    digitalWrite(Positioning2, LOW);    
    } 
  }
  if(LeadScrewCurrentPos>= 0-BackLash){
  digitalWrite(Positioning1,HIGH);}
  if(LeadScrewCurrentPos<= 0-BackLash){
  digitalWrite(Positioning2,HIGH);
  digitalWrite(Positioning1,LOW);}

  if(Positioning1==HIGH){
    digitalWrite(DirPulse, LOW);}
  if(Positioning2==HIGH){
    digitalWrite(DirPulse, HIGH);}
    
 if(INPOSITIONL==LOW){     
  if(micros() - PreviousMicros >= 1600) {
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
  if(LeadScrewCurrentPos<=0-LeadScrewReturnToLead){
  if(Positioning2== HIGH){
    digitalWrite(INPOSITIONR, HIGH);
    digitalWrite(RUNTOSTART, LOW);
    digitalWrite(Positioning1, LOW);
    digitalWrite(Positioning2, LOW);
    } 
  }
  if(LeadScrewCurrentPos<= 0+BackLash){
  digitalWrite(Positioning1,HIGH);}
  if(LeadScrewCurrentPos>= 0+BackLash){
  digitalWrite(Positioning2,HIGH);
  digitalWrite(Positioning1,LOW);}

  if(Positioning1==HIGH){
    digitalWrite(DirPulse, HIGH);}
  if(Positioning2==HIGH){
    digitalWrite(DirPulse, LOW);}
    
 if(INPOSITIONR==LOW){     
  if(micros() - PreviousMicros >= 1600) {
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


void COMS(){
   master.update();  //polling

   sm = millis();
   //READS
   sppsr = regs[PPR];
   BackLash = regs[BACKLASH];
   //WRITES
   regs[SPINDLE_RPM] = SpindleRPM;
   regs[total_packets] = NO_OF_PACKET;             //Total number of packet, here is 2
   regs[total_requests] = master.total_requests(); //Update all requested packets. Take a look on ModbusXT.h
   regs[total_failed] = master.total_failed();     //Update all failed packet

  //update transfer rate and transfer delay
  if ( (sm-dm) > 1000) //update 1s
   {
     dm = sm;
     regs[transfer_rate] = regs[total_requests] - temp;
     temp = regs[total_requests];
     regs[transfer_delay] = (unsigned int) ((NO_OF_PACKET*100000UL)/regs[transfer_rate]);
   }
}
  
  
