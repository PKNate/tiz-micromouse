#include <18f4550.h>
#device adc=10
#fuses HSPLL, PLL2, CPUDIV1, NOPROTECT, WDT128, NOMCLR, NOLVP
#use delay (clock=48M)
#use rs232(rcv=pin_c7,xmit=pin_c6,baud=9600,bits=8,parity=n)
//#use I2C(master, sda=PIN_B0, scl=PIN_B1, fast)
//#include <MPU6050.c>
#include <string.h>
#include <stdlib.h>

#define right 1
#define left 0
#define on 1
#define off 0

#bit CREN=0xFAB.4
#bit OERR=0xFAB.1
#byte PORTA = 0xF80
#byte PORTB = 0xF81
#byte PORTC = 0xF82
#byte PORTD = 0xF83
#byte PORTE = 0xF84
#byte TRISA = 0xF92
#byte TRISB = 0xF93
#byte TRISC = 0xF94
#byte TRISD = 0xF95
#byte TRISE = 0xF96
#bit PWMA = 0xF82.2
#bit PWMB = 0xF82.1
#bit BI1 = 0xF83.0
#bit BI2 = 0xF83.1
#bit AI1 = 0xF84.0
#bit AI2 = 0xF84.1
#bit STBY = 0xF84.2

//Movement variables
int16 L, FL, F, FR, R; 
float pwmL, pwmR;
static float k=0.5;                        //Proportional gain
static int16 minV=150;                     //PWM limits
static int16 nomV=200;
static int16 maxV=300;
static int16 turnV=260;                    //Why did I make this a variable? Used for turns
static int16 closeWall=350;                //Used for emergency shifts in straight(); and reverses in turn180();
static int16 turnWall=300;                 //Maximum allowed distance before attempting to turn
static int16 limWall=240;                  //Avg distance between both L and R sensors with two walls
static int16 noWall=100;                   //Minimum distance that indicates that no wall is nearby
static signed int16 pulses90= 660;         //Pulse counts (90º turn)
static signed int16 pulses180= 900;       //Pulse counts (180º turn)
static signed int16 pulsesD9= 600;                //Delay for 90 turn
static signed int16 pulsesDS= 550;               //Delay for forcing straight

//Encoder variables
volatile int8 aux;      
volatile int8 encoderM1=0;
volatile int8 encoderM1_prev=0;
volatile signed int16 pulsesM1=0;
volatile int8 encoderM2=0;
volatile int8 encoderM2_prev=0;
volatile signed int16 pulsesM2=0;

#int_rb
void rb_isr(){
   encoderM1=(PORTB&0b00110000)>>4;
   aux=encoderM1^encoderM1_prev;
   if(aux!=0&&aux!=0b00000011)
      if(((encoderM1_prev<<1)^encoderM1)&0b00000010)
         pulsesM1--;
      else
         pulsesM1++;
   encoderM1_prev=encoderM1;

   encoderM2=(PORTB&0b11000000)>>6;
   aux=encoderM2^encoderM2_prev;
   if(aux!=0&&aux!=0b00000011)
         if(((encoderM2_prev<<1)^encoderM2)&0b00000010)
            pulsesM2++;
         else
            pulsesM2--;
   encoderM2_prev=encoderM2;
   
   if((pulsesM1>30000)||(pulsesM1<-30000))
   pulsesM1=0;
   
   if((pulsesM2>30000)||(pulsesM2<-30000))
   pulsesM2=0;
}

void setup(short status);                                            //Setup instructions
void debug(int type);                                                //Encoder and SHARP test
void motor(char M1, signed int16 pwm1, char M2, signed int16 pwm2);   //Basic function for motor driving
void straight();                                                     //Tries to drive straight, based on (ALL) SHARP readings (All 5 of them)
void forceStraight(signed int16 delay, short reference);                                                //Used exclusively after a 90 turn.
void turn90(short direction);                                        //Short delay, 90 turn. Based on encoders, and (FL, FR) SHARP readings. 
void turn180();                                                      //180 turn, based on encoders and (ALL) SHARP readings
void readSensor(int sensor);                                         //ADC read
void update(int16 pwm1, int16 pwm2);                                 //Restores PWM value, reads (ALL) SHARP.

void main()
{  
   setup(on);
   
   while(true)
   {
      update(nomV, nomV);
      
      if(R<noWall)
      {
         forceStraight(pulsesD9,0);
         turn90(right);
         forceStraight(pulsesDS,1);
      }
      
      if(F>turnWall)
      {
         if(L<noWall)
         {
            turn90(left);
         }
         
         else
         {
            turn180();
         }
      }

      else
      {
         straight();
      }
   }
}

void setup(short status)
{
   STBY = 0;
   setup_wdt(WDT_ON);
   enable_interrupts(int_rb);
   enable_interrupts(int_rda);
   enable_interrupts(global);
   
   setup_adc(adc_clock_div_16);
   setup_adc_ports(AN0_TO_AN4, VSS_VDD);
   set_adc_channel(0);
   delay_us(20);
   
   //mpu6050_init();
   
   TRISA=0b11111111;
   TRISB=0b11111101; //SCL
   TRISC=0b10111001; //PWMA, PWMB, TX
   TRISD=0b11111100; //BI1, BI2
   TRISE=0b01111000; //STBY, AI1, AI2
   
   setup_timer_2(T2_DIV_BY_1,255,1);
   setup_ccp1(CCP_PWM);
   setup_ccp2(CCP_PWM);
   
   if(status)
   {
      motor('D',(int16)nomV,'D',(int16)nomV);
      STBY = 1;
   }
   
   restart_wdt();
   return;
}
void debug(int type)
{
   switch(type)
   {
      case 0:
      {
        readSensor(5);   
        printf("M1: %Ld    M2: %Ld \r\n",pulsesM1, pulsesM2);
        restart_wdt();
        delay_ms(100);
        break;
      }
      
      case 1:
      {
        readSensor(5);   
        printf("L:%Ld\tFL:%Ld\tF:%Ld\tFR:%Ld\tR:%Ld\n\r",L,FL,F,FR,R);
        restart_wdt();
        delay_ms(100);
        break;
      }
   }
   return;
}
void motor(char M1, signed int16 pwm1, char M2, signed int16 pwm2)
{
   switch (M1)
   {
      case 'D': {BI1=1; BI2=0; break;}
      case 'R': {BI1=0; BI2=1; break;}
      case 'N': {BI1=0; BI2=0; break;}
   }
   
   switch (M2)
   {
      case 'D': {AI1=1; AI2=0; break;}
      case 'R': {AI1=0; AI2=1; break;}
      case 'N': {AI1=0; AI2=0; break;}
   }
   
   if(pwm1>maxV) pwm1=maxV;
   else if(pwm1<minV) pwm1=minV;
   if(pwm2>maxV) pwm2=maxV;
   else if(pwm2<minV) pwm2=minV;
   
   set_pwm1_duty(pwm1);
   set_pwm2_duty(pwm2);
   
   restart_wdt();
   return;
}
void straight()
{ 
   pulsesM1=0;
   pulsesM2=0;
   
   if(FL>closeWall)     //Prioritizes not going straight into walls
   {
      pwmL=pwmL+(k*(FL-limWall));
      pwmR=pwmR-(k*(FL-limWall));
      motor('D',(signed int16)pwmL,'R',(signed int16)pwmR);
      return;
   }
   
   else if(FR>closeWall)
   {
      pwmL=pwmL-(k*(FR-limWall));
      pwmR=pwmR+(k*(FR-limWall));
      motor('R',(signed int16)pwmL,'D',(signed int16)pwmR);
      return;
   }
   
   else if(L>noWall && R>noWall)    //calibration
   {
      if(L>R)
      {
         pwmL=pwmL+(k*(L-R));
         pwmR=pwmR-(k*(L-R));
      }
      
      else if(R>L)
      {
         pwmL=pwmL-(k*(R-L));
         pwmR=pwmR+(k*(R-L));
      }
   }
   
   else if(pulsesM1>pulsesM2)       //no walls to calibrate
      {
         pwmL=pwmL-(k*(pulsesM1-pulsesM2));
         pwmR=pwmR+(k*(pulsesM1-pulsesM2));
         motor('D',(signed int16)pwmL,'D',(signed int16)pwmR);
      }
   
   else if(pulsesM1<pulsesM2)
   {
      pwmL=pwmL+(k*(pulsesM2-pulsesM1));
      pwmR=pwmR-(k*(pulsesM2-pulsesM1));
      motor('D',(signed int16)pwmL,'D',(signed int16)pwmR);
   }
   
   /*
   else if(R>noWall && L<noWall) //no left wall
   {
      if(R>limWall)
      {
         pwmL=pwmL-(k*(R-limWall));
         pwmR=pwmR+(k*(R-limWall));
      }
      
      if(R<limWall)
      {
         pwmL=pwmL+(k*(R-limWall));
         pwmR=pwmR-(k*(R-limWall));
      }
   }
   
   else if(L>noWall && R<noWall) //no right wall
   {
      if(L>limWall)
      {
         pwmL=pwmL+(k*(L-limWall));
         pwmR=pwmR-(k*(L-limWall));
      }
      
      if(L<limWall)
      {
         pwmL=pwmL-(k*(L-limWall));
         pwmR=pwmR+(k*(L-limWall));
      }
   }
   */
   motor('D',(signed int16)pwmL,'D',(signed int16)pwmR);
   return;
}
void forceStraight(signed int16 delay, short reference)
{
   pulsesM1=0;
   pulsesM2=0;
   
   while(pulsesM1<delay || pulsesM2<delay)        //Straight without references
   {
      update(nomV,nomV);
      
      if(F>turnWall)
      return;
      
      else if(FL>turnWall)     //Prioritizes not going straight into walls
      {
         pwmL=pwmL+(k*(FL-limWall));
         pwmR=pwmR-(k*(FL-limWall));
         motor('D',(signed int16)pwmL,'R',(signed int16)pwmR);
      }
   
      else if(FR>turnWall)
      {
         pwmL=pwmL-(k*(FR-limWall));
         pwmR=pwmR+(k*(FR-limWall));
         motor('R',(signed int16)pwmL,'D',(signed int16)pwmR);
      }

      /*
      else if(reference)
      {
         if(R>noWall && L>noWall)
         {
            return;
         }
      }
      */
      /*
      else if(pulsesM1>pulsesM2)
      {
         pwmL=pwmL-(k*(pulsesM1-pulsesM2));
         pwmR=pwmR+(k*(pulsesM1-pulsesM2));
         motor('D',(signed int16)pwmL,'D',(signed int16)pwmR);
      }
   
      else if(pulsesM1<pulsesM2)
      {
         pwmL=pwmL+(k*(pulsesM2-pulsesM1));
         pwmR=pwmR-(k*(pulsesM2-pulsesM1));
         motor('D',(signed int16)pwmL,'D',(signed int16)pwmR);
      }
      */
      else
         motor('D',(signed int16)pwmL,'D',(signed int16)pwmR);
   }
   return;
}
void turn90(short direction)
{
   pulsesM1=0;
   pulsesM2=0;

   if(direction==right)
   {
      while( (pulsesM1<pulses90) && (pulsesM2>(-1*pulses90)))
      {
         update(turnV, turnV);
         motor('D',(signed int16)pwmL,'R',(signed int16)pwmR);
         
         if(FL>turnWall)     //Prioritizes not going straight into walls
         {
            motor('R',(signed int16)pwmL,'R',(signed int16)pwmR);
         }
      
         else if(FR>turnWall)
         {
            motor('R',(signed int16)pwmL,'R',(signed int16)pwmR);
         }
      }
   }
        
   else
   {
      while((pulsesM2<pulses90) && (pulsesM1>(-1*pulses90)) )
      {
         update(turnV, turnV);
         motor('R',(signed int16)pwmL,'D',(signed int16)pwmR);
         
         if(FL>turnWall)     //Prioritizes not going straight into walls
         {
            motor('R',(signed int16)pwmL,'R',(signed int16)pwmR);
         }
      
         else if(FR>turnWall)
         {
            motor('R',(signed int16)pwmL,'R',(signed int16)pwmR);
         }
         
      }
   }      
    
}
void turn180()
{
   signed int16 tempM1, tempM2;
   pulsesM1=0;
   pulsesM2=0;
  
   while((pulsesM1<pulses180) || (pulsesM2>(-1*pulses180)) || (F>limWall) || (FL>limWall) || (FR>limWall))
   {      
      update(turnV, turnV);
      if(FR>closeWall || FR>closeWall || F>closeWall)
      {
         tempM1=pulsesM1;
         tempM2=pulsesM2;
         while(FR>closeWall || FR>closeWall || F>closeWall)
         {
            update(turnV, turnV);
            motor('R',(signed int16)pwmL,'R',(signed int16)pwmR);
         }
         pulsesM1=tempM1;
         pulsesM2=tempM2;
      }   
      else 
         motor('D',(signed int16)pwmL,'R',(signed int16)pwmR);
   }
   return;
}
void readSensor(int sensor)
{
   switch (sensor)
   {
      case 0: {set_adc_channel(1); delay_ms(1); L=read_adc(); break;}
      case 1: {set_adc_channel(4); delay_ms(1); FL=read_adc(); break;}
      case 2: {set_adc_channel(3); delay_ms(1); F=read_adc(); break;}
      case 3: {set_adc_channel(2); delay_ms(1); FR=read_adc(); break;}
      case 4: {set_adc_channel(0); delay_ms(1); R=read_adc(); break;}
      case 5: {set_adc_channel(1); delay_ms(1); L=read_adc();
               set_adc_channel(4); delay_ms(1); FL=read_adc();
               set_adc_channel(3); delay_ms(1); F=read_adc();
               set_adc_channel(2); delay_ms(1); FR=read_adc();
               set_adc_channel(0); delay_ms(1); R=read_adc(); break;}
   }
   
   return;
}
void update(int16 pwm1, int16 pwm2)
{
   readSensor(5);
   pwmL= (float) pwm1;
   pwmR= (float) pwm2;
   restart_wdt();
   return;
}
