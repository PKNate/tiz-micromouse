#include <18f4550.h>
#fuses HSPLL, PLL2, CPUDIV1, NOPROTECT, NOWDT, NOMCLR, NOLVP
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

void main()
{
   setup();
   printf("Inicio\n\r");
   while(true)
   {
     printf("Dentro\n\r");
     delay_ms(500);
   }
}

void setup()
{
   printf("Setup\n\r");
   
   TRISA=0b11111111;
   TRISB=0b11111101; //SCL
   TRISC=0b10111001; //PWMA, PWMB, TX
   TRISD=0b11111100; //BI1, BI2
   TRISE=0b01111000; //STBY, AI1, AI2
   
   setup_timer_2(T2_DIV_BY_1,255,1);
   setup_ccp1(CCP_PWM);
   setup_ccp2(CCP_PWM);
   
   motorDirection('D',50,'D',50);
   STBY = 1;
   
   delay_ms(2000);
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
