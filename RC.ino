
//------------------------------------------------------------------

#if defined SERVO

#define Servo1 14 // 5
#define Servo2 12 // 6

#include <Servo.h> 
Servo myservo1,myservo2; 

void init_RC()
{
    myservo1.attach(Servo2);
    myservo2.attach(Servo2);
}

void writeServo()
{
   myservo1.writeMicroseconds(rcValue[0]);
   myservo2.writeMicroseconds(rcValue[1]);
}

#elif defined PWM

#define PWM1 14 // 5
#define PWM2 12 // 6
#define PWM3 13 // 7
#define PWM4 15 // 8

#define THROCHAN 2
#define DIFFCHAN 0
#define DIRCHAN  4
#define LOWRC 1100
#define MIDRC 1500
#define HIRC  1900

int16_t gasraw,dirraw,gas,dir,lmot,rmot;
uint8_t armed = 10;
uint16_t  motorA1,motorA2,motorB1,motorB2;

void init_RC()
{
    //analogWriteRange(new_range)
    //analogWriteFreq(new_frequency)
    //analogWrite(pin, value)
  
    pinMode(PWM1,OUTPUT);
    digitalWrite(PWM1,LOW);
    pinMode(PWM2,OUTPUT);
    digitalWrite(PWM2,LOW);
    pinMode(PWM3,OUTPUT);
    digitalWrite(PWM3,LOW);
    pinMode(PWM3,OUTPUT);
    digitalWrite(PWM3,LOW);
}

void writeServo()
{
    gasraw = rcValue[THROCHAN] - LOWRC; 
    gas = constrain(gasraw,0,HIRC); 
    dirraw = rcValue[DIFFCHAN] - MIDRC;
    dir = constrain(dirraw,-500,500);

    if (armed != 0) 
    { 
      if (gas < 10) armed--; 
      gas = 0; dir = 0; 
    }  // wait for gas = 0 to arm
    
    if (rcValue[DIRCHAN] < 1600) gas = -gas; // -1000 to +1000, dir -500 to +500
    
    rmot = gas+dir; // -1500 to +1500
    lmot = gas-dir; // -1500 to +1500

    if (rmot > 10)
    {
      motorA1 = constrain(+rmot, 0, 1020); 
      motorA2 = 0;
    }
    else if (rmot < -10)
    {
      motorA1 = 0;
      motorA2 = constrain(-rmot, 0, 1020);
    }
    else 
    {
      motorA1 = 0;
      motorA2 = 0;
    }
    
    if (lmot > 10)
    {
      motorB1 = constrain(+lmot, 0, 1020); 
      motorB2 = 0;
    }
    else if (lmot < -10)
    {
      motorB1 = 0;
      motorB2 = constrain(-lmot, 0, 1020); 
    }
    else
    {
      motorB1 = 0;
      motorB2 = 0;
    }
    
    analogWrite(PWM1, motorA1); // 14 D5
    analogWrite(PWM2, motorB1); // 12 D6
    analogWrite(PWM3, motorA2); // 13 D7
    analogWrite(PWM4, motorB2); // 15 D8
}

#else

#define ppmpin 15
#define RC_CHANNELS 6
#define GAPTIME 500*80
#define CYCLETIME 20000*80

// clockCyclesPerMicrosecond() = 80
uint8_t  ppmCounter = 0;
uint8_t  ppmPhase = 0;
uint32_t ppmTotal = 0;
uint32_t ppmServo[RC_CHANNELS];
uint32_t next;
    
void inline PPM_ISR(void)
{
  uint32_t ppmOut;
  if (ppmPhase == 0)
  {
    next += GAPTIME; // 0.5 ms
    timer0_write(next);
    digitalWrite(ppmpin,LOW);
    ppmTotal += 40000;
    ppmPhase = 1;
  }
  else
  {
    if (ppmCounter >= RC_CHANNELS) 
    {
      next += CYCLETIME - ppmTotal; // 20ms total 
      timer0_write(next);
      digitalWrite(ppmpin,HIGH);
      ppmCounter = 0;
      ppmTotal = 0;
    } 
    else 
    {  
      ppmOut = ppmServo[ppmCounter];
      next += ppmOut;
      timer0_write(next);
      digitalWrite(ppmpin,HIGH);
      ppmTotal += ppmOut;
      ppmCounter++;
    }
    ppmPhase = 0;
  }
}

void writeServo()
{
  for (int i=0;i<RC_CHANNELS;i++) ppmServo[i] = constrain(rcValue[i],900,2100)*80 - GAPTIME;
}

void init_RC()
{
  for (int i=0;i<RC_CHANNELS;i++) ppmServo[i] = 1500*80 - GAPTIME;
  pinMode(ppmpin, OUTPUT); 
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(PPM_ISR);
  next=ESP.getCycleCount()+100000;
  timer0_write(next);
  interrupts();
}

#endif

//------------------------------------------------------------------

void rc_to_buf()
{
  RCdata.chans.Ch1 = rcValue[0];   
  RCdata.chans.Ch2 = rcValue[1];   
  RCdata.chans.Ch3 = rcValue[2];   
  RCdata.chans.Ch4 = rcValue[3];   
  RCdata.chans.Ch5 = rcValue[4];   
  RCdata.chans.Ch6 = rcValue[5];   
  RCdata.chans.Ch7 = rcValue[6];   
  RCdata.chans.Ch8 = rcValue[7];
  RCdata.chans.spare = seqno;       
}

void buf_to_rc()
{
  uint8_t seq;
  rcValue[0] = RCdata.chans.Ch1;
  rcValue[1] = RCdata.chans.Ch2;
  rcValue[2] = RCdata.chans.Ch3;
  rcValue[3] = RCdata.chans.Ch4;
  rcValue[4] = RCdata.chans.Ch5;
  rcValue[5] = RCdata.chans.Ch6;
  rcValue[6] = RCdata.chans.Ch7;
  rcValue[7] = RCdata.chans.Ch8;
  seqno = RCdata.chans.spare;
}

