#include <18f4550.h>
#fuses HSPLL, PLL2, CPUDIV1, NOPROTECT, NOWDT, NOMCLR, NOLVP
#device adc=10
#use delay (clock=48M)
#use rs232(rcv=pin_c7,xmit=pin_c6,baud=9600,bits=8,parity=n)

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

#bit LEFT = 0xF80.0
#bit LFFR = 0xF80.1
#bit FRNT = 0xF80.2
#bit RGFR = 0xF80.3
#bit RGHT = 0xF80.5

#bit PWMA = 0xF82.2
#bit PWMB = 0xF82.1
#bit BI1 = 0xF83.0
#bit BI2 = 0xF83.1
#bit AI1 = 0xF84.0
#bit AI2 = 0xF84.1

#bit STBY = 0xF84.2

void setup();
void motorDirection(char M1, char M2, int pwm1, int pwm2);
void readSensor(int sensor);

int16 adc1, adc2, adc3, adc4, adc5; 

void main()
{
   setup();
   printf("Inicio\n\r");
   while(true)
   {
     /*
     motorDirection('D',50,'D',50);
     delay_ms(2000);
     motorDirection('D',50,'R',50);
     delay_ms(2000);
     motorDirection('R',50,'R',50);
     delay_ms(2000);
     motorDirection('R',50,'D',50);
     delay_ms(2000);
     */
     
     readSensor(5);

     printf("S1: %Ld\n\r",adc1);
     printf("S2: %Ld\n\r",adc2);
     printf("S3: %Ld\n\r",adc3);
     printf("S4: %Ld\n\r",adc4);
     printf("S5: %Ld\n\n\n\r",adc5);
     delay_ms(1000);
     
   }
}

void setup()
{
   STBY = 0;
   printf("Setup\n\r");
   
   setup_adc(adc_clock_div_16);
   setup_adc_ports(AN0_TO_AN5, VSS_VDD);
   set_adc_channel(0);
   delay_us(20);
   
   TRISA=0b11111111;
   TRISB=0b11111101; //SCL
   TRISC=0b10111001; //PWMA, PWMB, TX
   TRISD=0b11111100; //BI1, BI2
   TRISE=0b01111000; //STBY, AI1, AI2
   
   setup_timer_2(T2_DIV_BY_1,255,1);
   setup_ccp1(CCP_PWM);
   setup_ccp2(CCP_PWM);
   delay_ms(2000);
   //STBY = 1;
}

void motorDirection(char M1, int pwm1, char M2, int pwm2)
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
      case 0: {set_adc_channel(0); delay_us(20); adc1=read_adc(); break;}
      case 1: {set_adc_channel(1); delay_us(20); adc2=read_adc(); break;}
      case 2: {set_adc_channel(2); delay_us(20); adc3=read_adc(); break;}
      case 3: {set_adc_channel(3); delay_us(20); adc4=read_adc(); break;}
      case 4: {set_adc_channel(4); delay_us(20); adc5=read_adc(); break;}
      case 5: {set_adc_channel(0); delay_us(20); adc1=read_adc();
               set_adc_channel(1); delay_us(20); adc2=read_adc();
               set_adc_channel(2); delay_us(20); adc3=read_adc();
               set_adc_channel(3); delay_us(20); adc4=read_adc();
               set_adc_channel(4); delay_us(20); adc5=read_adc(); break;}
   } 
}
