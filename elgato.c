#include <18f4550.h>
#device adc=10
#fuses HSPLL, PLL2, CPUDIV1, NOPROTECT, NOWDT, NOMCLR, NOLVP
#use delay (clock=48M)
#use rs232(rcv=pin_c7,xmit=pin_c6,baud=9600,bits=8,parity=n)
//#use I2C(master, sda=PIN_B0, scl=PIN_B1, fast)
//#include <MPU6050.c>
#include <string.h>
#include <stdlib.h>

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

int16 L, FL, F, FR, R; 
//int32 temp, x, y, z;
volatile int8 aux;
volatile int8 encoderM1=0;
volatile int8 encoderM1_anterior=0;
volatile signed int16 cuentasM1=0;
volatile int8 encoderM2=0;
volatile int8 encoderM2_anterior=0;
volatile signed int16 cuentasM2=0;

#int_rb
void rb_isr(){
   encoderM1=(PORTB&0b00110000)>>4;
   aux=encoderM1^encoderM1_anterior;
   if(aux!=0&&aux!=0b00000011)
      if(((encoderM1_anterior<<1)^encoderM1)&0b00000010)
         cuentasM1--;
      else
         cuentasM1++;
   encoderM1_anterior=encoderM1;

   encoderM2=(PORTB&0b11000000)>>6;
   aux=encoderM2^encoderM2_anterior;
   if(aux!=0&&aux!=0b00000011)
         if(((encoderM2_anterior<<1)^encoderM2)&0b00000010)
            cuentasM2++;
         else
            cuentasM2--;
   encoderM2_anterior=encoderM2;
   
   if((cuentasM1>30000)||(cuentasM1<-30000))
   cuentasM1=0;
   
   if((cuentasM2>30000)||(cuentasM2<-30000))
   cuentasM2=0;
}

void setup(short on);
void fflush();
void motor(char M1, char M2, int pwm1, int pwm2);
void readSensor(int sensor);
void debug(int type);

void main()
{
   delay_ms(1000);
   setup(0);
   
   while(true)
   {
      debug(1);
      /*
      readSensor(5);
      if(F>210)
      {
         if(adc5<100)
         {
            motor('D',50,'R',50);
            cuentasM1=0;
            while(cuentasM1<300);
         }
         
         else if(L<100)
         {
            motor('R',50,'D',50);
            cuentasM2=0;
            while(cuentasM2<300);
         }
         
         else
         {
            motor('R',50,'R',50);
         }
      }
      
      else if(L>350)
      {
         motor('D',60,'R',50);
      }
      
      else if(FL>350)
      {
         motor('D',60,'R',50);
      }
      
      else if(FR>350)
      {
         motor('R',50,'D',60);
      }
      
      else if(R>350)
      {
         motor('R',50,'D',60);
      }
      
      else
      {
         motor('D',50,'D',50);
      }
      */
   }
}

void setup(short on)
{
   STBY = 0;
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
   
   if(on)
   {
      motor('D',50,'D',50);
      STBY = 1;
   }
   
   return;
}

void fflush()
{
   if(OERR)
   {
      getc();     //Clear buffer
      getc();
      CREN=0;      //Clear CREN bit
      CREN=1;           
   }
}

void motor(int M1, int pwm1, char M2, int pwm2)
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
   
   set_pwm1_duty(pwm1);
   set_pwm2_duty(pwm2);
}

void readSensor(int sensor)
{
   switch (sensor)
   {
      case 0: {set_adc_channel(1); delay_us(20); L=read_adc(); break;}
      case 1: {set_adc_channel(4); delay_us(20); FL=read_adc(); break;}
      case 2: {set_adc_channel(3); delay_us(20); F=read_adc(); break;}
      case 3: {set_adc_channel(2); delay_us(20); FR=read_adc(); break;}
      case 4: {set_adc_channel(0); delay_us(20); R=read_adc(); break;}
      case 5: {set_adc_channel(1); delay_us(20); L=read_adc();
               set_adc_channel(4); delay_us(20); FL=read_adc();
               set_adc_channel(3); delay_us(20); F=read_adc();
               set_adc_channel(2); delay_us(20); FR=read_adc();
               set_adc_channel(0); delay_us(20); R=read_adc(); break;}
   } 
}

void debug(int type)
{
   switch(type)
   {
      case 0:
      {
        readSensor(5);   
        printf("M1: %Ld    M2: %Ld \r\n",cuentasM1, cuentasM2);
        delay_ms(100);
        break;
      }
      
      case 1:
      {
        readSensor(5);   
        printf("L:%Ld\tFL:%Ld\tF:%Ld\tFR:%Ld\tR:%Ld\n\r",L,FL,F,FR,R);
        delay_ms(100);
        break;
      }
   }
   return;
}
