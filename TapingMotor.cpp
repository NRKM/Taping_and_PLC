#include "Common.h"

int M_pinA              = 0;
int M_pinB              = 1;
int M_pinPWM            = 2;
int M_pinFW             = 3;
int M_pinBR             = 24;
int M_pinRE             = 25;

double P                = 30;
double I                = 0;
double D                = 30;
//double PWM_MAX          = 65535;
double PWM_MAX          = 60000;
double PWM_MIN          = 12000;
int PID_On              = 0;

void Init_TapingMotor(void);
void enc_changedPinA(void);
void enc_changedPinB(void);
void Forward(void);
void Break(void);
void Reverse(void);
void PID_Start(void);
void PID_Control(int Target_Count, int phase);
void PWM_OutPutValue_Check(int Output);
void COrigin_Check(int p_deviation,int phase,int output);

void Init_TapingMotor(void){

    M_pinA						= Motor_pinA	;
    M_pinB						= Motor_pinB	;
    M_pinPWM					= Motor_pinPWM	;
    M_pinFW						= Motor_pinFW	;
    M_pinBR						= Motor_pinBR	;
    M_pinRE						= Motor_pinRE	;

	pinMode(M_pinA              , INPUT)		;
    pinMode(M_pinB              , INPUT)		;
    pinMode(M_pinFW             , OUTPUT)		;
    pinMode(M_pinBR             , OUTPUT)		;
    pinMode(M_pinRE             , OUTPUT)		;
    pinMode(M_pinPWM            , PWM)			;

	attachInterrupt(M_pinA      , enc_changedPinA, CHANGE);
    attachInterrupt(M_pinB      , enc_changedPinB, CHANGE);  

}

void enc_changedPinA(){
  
    if(digitalRead(M_pinA)){
      if(digitalRead(M_pinB)) --enc_count;
      else ++enc_count;
     }else{
      if(digitalRead(M_pinB)) ++enc_count;
      else --enc_count;
     }
     
}
  
void enc_changedPinB(){
  
    if(digitalRead(M_pinB)){
      if(digitalRead(M_pinA)) ++enc_count;
      else --enc_count;
     }else{
      if(digitalRead(M_pinA)) --enc_count;
      else ++enc_count;
     }
     
}
    
void Forward(){
    
	SerialUSB.println("Forward");
    digitalWrite(M_pinFW, HIGH);
    digitalWrite(M_pinBR, LOW);
    digitalWrite(M_pinRE, LOW);
  
}
  
void Break(){
      
	SerialUSB.println("Break");
    digitalWrite(M_pinFW, LOW);
    digitalWrite(M_pinBR, HIGH);
    digitalWrite(M_pinRE, LOW);
      
}
  
void Reverse(){
  
	SerialUSB.println("Reverse");
    digitalWrite(M_pinFW, LOW);
    digitalWrite(M_pinBR, LOW);
    digitalWrite(M_pinRE, HIGH);
  
}

void PID_Start(){
    //P = (PWM_MAX - PWM_MIN) / Target_Count;
    PID_On = 1;
    //
    Reverse();
}
  
  
void PID_Control(int Target_Count, int phase){
  static   int i              = 0;
  static volatile int P_Control      = 0;
  static volatile int I_Control      = 0;
  static volatile int D_Control      = 0;
  float dt, preTime;
  
  static volatile int p_dev          = 0;
  static volatile int i_sum          = 0;
  static volatile int d_val          = 0;
  static volatile int pre_p_dev      = 0;
    
  static int Output                  = 0;

  if(phase == 1){
	  P = 40;
	  //PWM_MIN = 30000;
	  PWM_MIN = 16000;
	  SerialUSB.println("Target_Count");
	  SerialUSB.println(Target_Count);
  }
  else if(phase == 2 || phase == 0){
	  P = 7;
	  //PWM_MIN = 30500;
  	  PWM_MIN = 16000;
	  SerialUSB.println("Target_Count");
	  SerialUSB.println(Target_Count);
  }
  if(Debug_On ==1){
    SerialUSB.println("P_Value");
    SerialUSB.println(P);
  }
  i++;
   //
   if(i == MicroSec_To_MilliSec * Debug_Time && Debug_On ==1){
      SerialUSB.println("enc_count");
      SerialUSB.println(enc_count);
      SerialUSB.println("p_dev");
      SerialUSB.println(p_dev);
      SerialUSB.println("i_sum");
      SerialUSB.println(i_sum);
      SerialUSB.println("P_control");
      SerialUSB.println(P_Control);
      i = 0;
        
   }
   dt = (micros() - preTime) / (MilliSec_To_Sec * MicroSec_To_MilliSec);
   preTime = micros();
   if(enc_count < 0){
      //Break();
      p_dev              = Target_Count - (-1) * enc_count;
      i_sum             += p_dev * dt;
      d_val              = (p_dev - pre_p_dev) / dt;
      if(p_dev >= 0){
        P_Control   = P            * p_dev + PWM_MIN;
        I_Control   = I            * i_sum;
        D_Control   = D            * d_val;
        Output      = P_Control    + I_Control - D_Control;
        Reverse();
      }else if(p_dev < 0){
		      //Break();
              P_Control   = P            * (-1) * p_dev  + PWM_MIN;
              I_Control   = I            * (-1) * i_sum;
              D_Control   = D            * (-1) * d_val;
              Output      = P_Control    + I_Control - D_Control;
              Forward();
      }
   }else if(enc_count >= 0){
      p_dev      = Target_Count - enc_count;
      i_sum             += p_dev * dt;
      d_val              = (p_dev - pre_p_dev) / dt;
        if(p_dev >= 0){
	      //Break();
          P_Control   = P            * p_dev + PWM_MIN;
          I_Control   = I            * i_sum;
          D_Control   = D            * d_val;
          Output      = P_Control    + I_Control - D_Control;
          Reverse();
        }else if(p_dev < 0){
		        //Break();
                P_Control   = P            * (-1) * p_dev  + PWM_MIN;
                I_Control   = I            * (-1) * i_sum;
                D_Control   = D            * (-1) * d_val;
                Output      = P_Control    + I_Control - D_Control;
                Forward();
        }
   }
   pre_p_dev = p_dev;
   SerialUSB.println("output");
   SerialUSB.println(Output);
   PWM_OutPutValue_Check(Output);
   COrigin_Check(p_dev,phase,Output);
     
}
  
void PWM_OutPutValue_Check(int Output){
   if(PWM_MIN < Output || Output < PWM_MAX){
     analogWrite(M_pinPWM, Output);
   }else if(Output < PWM_MIN){
       Output = PWM_MIN;
       analogWrite(M_pinPWM, Output);
   }else if(PWM_MAX < Output){
       Output = PWM_MAX;
       analogWrite(M_pinPWM, Output);
   }
}
  
void COrigin_Check(int p_deviation,int phase,int output){
  #define Origin_Min -40
  #define Origin_Max 40
  #define Origin_Check_Time 20
  static int i=0;
  static int j=0;
    
  if(PID_On == 1 && Origin_Min <= p_deviation && p_deviation <= Origin_Max){
    i++;
	output = output - i * 250;

    if(i >= Origin_Check_Time){
	  //while(j<100){
		//analogWrite(M_pinPWM, output);
		//j++;
	  //}
	  j=0;
      if(phase ==1){
        Running_Status = LEVER_CLOSE;
      }else if(phase == 2){
        Running_Status = OKURI_START2;
      }else {
        Running_Status = WAIT_INPUT;
      }
	  while(j<10){
	      analogWrite(M_pinPWM, output);
		  j++;
	  }
	  Break();
	  j=0;
	  i=0;
    }
  }
}