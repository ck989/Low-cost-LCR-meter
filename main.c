//LCR METER
//KUMAR ANANDA CHAYA
//1001671413

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "tm4c123gh6pm.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define MEASURE_LR   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))
#define MEASURE_C    (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))
#define HIGHSIDE_R   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define INTEGRATE    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define LOWSIDE_R    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
#define MAX_CHAR      80

char str[MAX_CHAR], a, type[10],pos_arg[10],s[], strverb[10];
uint8_t i = 0;
uint8_t m =0, arg_no =0;
uint8_t argc;
bool valid_cmd = false;
float V1,V2,diff_V,time = 0, resistance, capacitance,inductance,Rl;
char arg1[10],arg2[10], strun[10],strun1[10],strun2[10],stop[10],resis[10],ir[10];
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Blocking function that returns only when SW1 is pressed
void waitPbPress()
{
    while(PUSH_BUTTON);
}

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOB |SYSCTL_RCGC2_GPIOC;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0A;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTE_DIR_R = 0x20;
    GPIO_PORTB_DIR_R = 0x10;
    GPIO_PORTA_DIR_R = 0xE0;
    GPIO_PORTF_DR2R_R = 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DR2R_R = 0x20;
    GPIO_PORTB_DR2R_R = 0x10;
    GPIO_PORTA_DR2R_R = 0xE0;
    GPIO_PORTF_DEN_R = 0x1A;  // enable LEDs and pushbuttons
    GPIO_PORTE_DEN_R = 0x20;
    GPIO_PORTB_DEN_R = 0x10;
    GPIO_PORTA_DEN_R = 0xE0;
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // timer configuration
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;     // turn-on timer
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER5_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER5_TAV_R = 0;                               // zero counter for first period
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter

    // Configure AN0(DUT2) as an analog input
            SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
            GPIO_PORTE_AFSEL_R |= 0x08;                      // select alternative functions for AN0 (PE3)
            GPIO_PORTE_DEN_R &= ~0x08;                       // turn off digital operation on pin PE3
            GPIO_PORTE_AMSEL_R |= 0x08;                      // turn on analog operation on pin PE3
            ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
            ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
            ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
            ADC0_SSMUX3_R = 0;                               // set first sample to AN0
            ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
            ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

            // Configure AN1 as an analog input
                   SYSCTL_RCGCADC_R |= 2;                           // turn on ADC module 1 clocking
                   GPIO_PORTE_AFSEL_R |= 0x04;                      // select alternative functions for AN0 (PE2)
                   GPIO_PORTE_DEN_R &= ~0x04;                       // turn off digital operation on pin PE2
                   GPIO_PORTE_AMSEL_R |= 0x04;                      // turn on analog operation on pin PE2
                   ADC1_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
                   ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
                   ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
                   ADC1_SSMUX3_R = 1;                               // set first sample to AN1
                   ADC1_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
                   ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation


     // configuration for comparator
           GPIO_PORTC_DEN_R &= ~0x80;
           GPIO_PORTC_AFSEL_R |= 0x80;
           GPIO_PORTC_AMSEL_R |= 0x80;
           GPIO_PORTC_DIR_R &= ~0x80;
           SYSCTL_RCGCACMP_R |= 0x01;
           COMP_ACREFCTL_R |= COMP_ACREFCTL_VREF_M | COMP_ACREFCTL_EN;
           COMP_ACCTL0_R |= COMP_ACCTL0_ASRCP_REF | COMP_ACCTL0_ISEN_RISE;
           COMP_ACCTL0_R |= COMP_ACCTL0_CINV;
           COMP_ACRIS_R |= COMP_ACRIS_IN0;
           COMP_ACINTEN_R |= 0x01;
           NVIC_EN0_R &= ~(1 << INT_COMP0 - 16);



           SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;     // turn-on timer
           WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
           WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
           WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
           WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
           WTIMER5_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
           WTIMER5_TAV_R = 0;                               // zero counter for first period
           WTIMER5_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter

}



void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #3");
    __asm("WMS_LOOP1:   SUB  R1, #1");
    __asm("             CBZ  R1, WMS_DONE1");
    __asm("             NOP");
    __asm("             NOP");
    __asm("             B    WMS_LOOP1");
    __asm("WMS_DONE1:   SUB  R0, #1");
    __asm("             CBZ  R0, WMS_DONE0");
    __asm("             NOP");
    __asm("             B    WMS_LOOP0");
    __asm("WMS_DONE0:");

}

void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}

void itostring(int n)
{
    int j, sign;

    if ((sign = n) < 0)  /* record sign */
        n = -n;          /* make n positive */
    j = 0;
    do {       /* generate digits in reverse order */
        s[j++] = n % 10 + '0';   /* get next digit */
    } while ((n /= 10) > 0);     /* delete it */
    if (sign < 0)
        s[j++] = '-';
    s[j] = '\0';
}

int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit(DUT2)
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

int16_t readAdc1Ss3()
{
    ADC1_PSSI_R |= ADC_PSSI_SS3;                     // set start bit(DUT1)
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC1_SSFIFO3_R;                           // get single result from the FIFO
}


void analogcomp()
{
    time = WTIMER5_TAV_R;
    WTIMER5_TAV_R = 0;
    COMP_ACMIS_R |= 0x01;
}

void step5(char* arg1, char* arg2)
{

    if((strcmp(arg1,"greenled") == 0) && (strcmp(arg2,"on") == 0))
    {
        GREEN_LED = 1;
    }
    else if((strcmp(arg1,"greenled") == 0) && (strcmp(arg2,"off") == 0))
    {
        GREEN_LED = 0;
    }



    if((strcmp(arg1,"measurelr") == 0) && (strcmp(arg2,"on") == 0))
        {
            MEASURE_LR = 1;
        }
        else if((strcmp(arg1,"measurelr") == 0) && (strcmp(arg2,"off") == 0))
        {
            MEASURE_LR = 0;
        }

    if((strcmp(arg1,"measurec") == 0) && (strcmp(arg2,"on") == 0))
        {
            MEASURE_C = 1;
        }
        else if((strcmp(arg1,"measurec") == 0) && (strcmp(arg2,"off") == 0))
        {
            MEASURE_C = 0;
        }


    if((strcmp(arg1,"highsider") == 0) && (strcmp(arg2,"on") == 0))
        {
            HIGHSIDE_R = 1;
        }
        else if((strcmp(arg1,"highsider") == 0) && (strcmp(arg2,"off") == 0))
        {
            HIGHSIDE_R = 0;
        }

    if((strcmp(arg1,"integrate") == 0) && (strcmp(arg2,"on") == 0))
        {
            INTEGRATE = 1;
        }
        else if((strcmp(arg1,"integrate") == 0) && (strcmp(arg2,"off") == 0))
        {
            INTEGRATE = 0;
        }


    if((strcmp(arg1,"highsider") == 0) && (strcmp(arg2,"on") == 0))
        {
            HIGHSIDE_R = 1;
        }
        else if((strcmp(arg1,"highsider") == 0) && (strcmp(arg2,"off") == 0))
        {
            HIGHSIDE_R = 0;
        }

    if((strcmp(arg1,"lowsider") == 0) && (strcmp(arg2,"on") == 0))
         {
             LOWSIDE_R = 1;
         }
         else if((strcmp(arg1,"lowsider") == 0) && (strcmp(arg2,"off") == 0))
         {
             LOWSIDE_R = 0;
         }

}

void resetglobalvariables()
{
    uint8_t rs=0;
    for(rs=0;rs<15;rs++)
    {
        str[rs] = '\0';
        type[rs] = '\0';
        pos_arg[rs] = '\0';
        argc = 0;
        m=0;
    }
}


    bool iscommand(char* str_verb, uint8_t min_arg)
    {
      if(strcmp(str_verb,&str[pos_arg[0]]) == 0 && argc>=min_arg)
      {
          return true;
      }
         return false;
    }


void step4()
{
    uint8_t x = 0, y = 0, p = pos_arg[1], q = pos_arg[2];
if(iscommand("set",2))
{
    if(strcmp(&str[pos_arg[0]],"set")==0)
    {
        if(type[1] == 'a')
        {
          while(str[p] != '\0')
            {
              arg1[x] = str[p];
              x++;
              p++;
              arg1[x] = '\0';
            }
        }
        if(type[2] == 'a')
        {
           while(str[q] != '\0')
             {
                arg2[y] = str[q];
                y++;
                q++;
                arg2[y] = '\0';
              }
           step5(arg1,arg2);
        }
    }
    else
     {
        putsUart0("sorry, this command requires atleast two arguments");
        valid_cmd = false;
     }
}

if(iscommand("voltage", 0))
            {
                V1 = readAdc0Ss3();
                V2 = readAdc1Ss3();
                V1 = (V1*3.3/4095);
                V2 = (V2*3.3/4095);
                diff_V = ((V2 - V1)*3.3/4095);
                sprintf(strun1,"%f", V1);
                sprintf(strun2,"%f", V2);
                sprintf(strun, "%f", diff_V);
                putsUart0("\n\r");
                putsUart0(strun1);
                putsUart0("\n\r");
                putsUart0(strun2);
                putsUart0("\n\r");
                putsUart0(strun);
            }

if(iscommand("reset",0))
       {
           NVIC_APINT_R = 0x005FA0004;
       }

if(iscommand("resistor", 0))
{
    LOWSIDE_R = 1;
    MEASURE_LR = 0;
    MEASURE_C = 0;
    INTEGRATE = 1;
    HIGHSIDE_R = 0;

    waitMicrosecond(3000000);

    MEASURE_LR = 1;
    MEASURE_C = 0;
    LOWSIDE_R = 0;
    //INTEGRATE = 1;
    HIGHSIDE_R = 0;
    NVIC_EN0_R |= (1 << (INT_COMP0 - 16));
    waitMicrosecond(3000000);
    NVIC_EN0_R &= ~(1 << (INT_COMP0 - 16));
    putsUart0("\r\n ");
    putsUart0("tou = ");
    sprintf(stop ,"%f ", time);
    if(time >= 1000 && time <= 3999)
    {
        resistance = (0.02868*time) -23.011;
    }
    if(time >= 4000 && time <= 39999 )
    {
        resistance = (0.02056*time) + 7.5501;
    }

    if(time >= 40000 && time <= 60000000 )
    {
    resistance = (0.02065*time) + 120.6096;
    }
    putsUart0(stop);
    putsUart0("\n\r");
    putsUart0("resistance =");
    sprintf(resis , "%f", resistance);
    putsUart0(resis);
    putsUart0("ohm");
    resistance = 0;
}

if(iscommand("capacitor", 0))
{
    LOWSIDE_R = 1;
    MEASURE_LR = 0;
    MEASURE_C = 1;
    INTEGRATE = 0;
    HIGHSIDE_R = 0;

    waitMicrosecond(20000000);

    MEASURE_LR = 0;
    MEASURE_C = 1;
    LOWSIDE_R = 0;
    INTEGRATE = 0;
    HIGHSIDE_R = 1;
    NVIC_EN0_R |= (1 << (INT_COMP0 - 16));
    waitMicrosecond(50000000);
    NVIC_EN0_R &= ~(1 << (INT_COMP0 - 16));
    time /=40;
    putsUart0("\r\n ");
    putsUart0("tou = ");
    sprintf(stop ,"%f", time);
    putsUart0(stop);
    V1 = readAdc0Ss3();
    V2 = readAdc1Ss3();
    V1 = (V1*3.3/4095);
    V2 = (V2*3.3/4095);
    diff_V = ((V2 - V1)*3.3/4095);
    sprintf(strun1,"%f", V1);
    sprintf(strun2,"%f", V2);
    sprintf(strun, "%f", diff_V);
    putsUart0("\n\r");
    putsUart0(strun1);
    putsUart0("\n\r");
    putsUart0(strun2);
    putsUart0("\n\r");
    putsUart0(strun);
    if(time >= 40 && time <= 999)
    {
        capacitance = (0.000007413*time) - 0.000296;
    }
    if(time >= 1000 && time <= 15000000)
    {
    capacitance = (0.0000068492*time) + 0.004596416;
    }
    putsUart0("\n\r");
    putsUart0("capacitance =");
    sprintf(resis , "%f",capacitance);
    putsUart0(resis);
    putsUart0("micro Farad");
    capacitance = 0;
    time = 0;
    //MEASURE_LR = 0;
        //MEASURE_C = 1;
        //LOWSIDE_R = 1;
        //INTEGRATE = 0;
        //HIGHSIDE_R = 1;

}

if(iscommand("inductor", 0))
{
    V1 = readAdc0Ss3();
    V2 = readAdc1Ss3();
    V1 = (V1*3.3/4095);
    V2 = (V2*3.3/4095);
    sprintf(strun1,"%f", V1);
    sprintf(strun2,"%f", V2);
    putsUart0("\n\r");
    putsUart0(strun1);
    putsUart0("\n\r");
    putsUart0(strun2);
    LOWSIDE_R = 1;
    MEASURE_LR = 1;
    MEASURE_C = 0;
    INTEGRATE = 0;
    HIGHSIDE_R = 0;
    WTIMER5_TAV_R = 0;
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;
    NVIC_EN0_R |= (1 << (INT_COMP0 - 16));
    waitMicrosecond(20000000);
    NVIC_EN0_R &= ~(1 << (INT_COMP0 - 16));
    time /=40;
    putsUart0("\r\n ");
    putsUart0("tou = ");
    sprintf(stop ,"%f", time);
    putsUart0(stop);
    V1 = readAdc0Ss3();
    V2 = readAdc1Ss3();
    V1 = (V1*3.3/4095);
    V2 = (V2*3.3/4095);
    sprintf(strun1,"%f", V1);
    sprintf(strun2,"%f", V2);
    putsUart0("\n\r");
    putsUart0(strun1);
    putsUart0("\n\r");
    putsUart0(strun2);
    if(time >= 1 && time <= 2)
    {
        inductance = (76.389*time) + 3.9659;
    }
    else
    {
    inductance = (31.765*time) - 0.02745;
    }
    putsUart0("\n\r");
    putsUart0("inductance =");
    sprintf(resis , "%f",inductance);
    putsUart0(resis);
    putsUart0("micro Henry");

}
if(iscommand("esr",0))
{
    LOWSIDE_R = 1;
    MEASURE_LR = 1;
    MEASURE_C = 0;
    INTEGRATE = 0;
    HIGHSIDE_R = 0;
    waitMicrosecond(10000000);
    V1 = readAdc0Ss3();
    V2 = readAdc1Ss3();
    V1 = (V1*3.3/4095);
    V2 = (V2*3.3/4095);
    diff_V = ((V2 - V1)*3.3/4095);
    sprintf(strun1,"%f", V1);
    sprintf(strun2,"%f", V2);
    sprintf(strun, "%f", diff_V);
    putsUart0("\n\r");
    putsUart0("voltage across DUT1 is");
    putsUart0(strun1);
    putsUart0("\n\r");
    putsUart0("voltage across DUT2 is");
    putsUart0(strun2);
    putsUart0("\n\r");
    putsUart0("voltage diiference is");
    putsUart0(strun);
    Rl = ((33*(V2 - V1))/V1);
    Rl = 1.166*Rl;
    putsUart0("\n\r");
    sprintf(ir,"%f", Rl);
    putsUart0(ir);
    putsUart0("ohm");
}
}
*/


 char step2()
 {
     int count = 0;
     while(count<MAX_CHAR)
     {
         a =  getcUart0();
         putcUart0(a);
         if (a == 8)
         {
             if(count!=0)
                 count --;
                 continue;
         }
         else if(a == 13)
         {
             break;
         }
         else
         {
             if(a >= 32)
             {
                 str[count++] = tolower(a);
             }
             else
             {
                 continue;
             }
             if(count == MAX_CHAR)
             {
                 putsUart0("you cannot exceed more than 80 characters");
                 break;
             }
             else
             {
                 continue;
             }
         }
     }
str[count]='\0';
return 0;
 }

 void step3()
 {
 uint8_t index = 0;
 uint8_t d,b,l;
 l = strlen(str);
 while(index<=l)
 {
     d = ((str[index-1]>=32 && str[index-1]<=47) || (str[index-1]>=58&&str[index-1]<=64) || (str[index-1]>=91&&str[index-1]<=96) || (str[index-1]>=123&&str[index-1]<=127));
     b = ((str[index]>=97&&str[index]<=122) || (str[index]>=48&&str[index]<=57));
     if ((str[index]>=32 && str[index]<=47) || (str[index]>=58&&str[index]<=64) || (str[index]>=91&&str[index]<=96) || (str[index]>=123&&str[index]<=127))
     {

     index++;
     continue;
     }
 else
     {
     if(b)
     {
         if(index == 0 && b)
         {
             pos_arg[m] = index;
             index++;
             if(str[0]>=97&&str[0]<=122)
                     {
                     type[m] = 'a';
                     }
                     else if(str[0]>=48&&str[0]<=57)
                     {
                      type[m] = 'n';
                     }
             m++;
         }

         pos_arg[m] = index;
         if(str[index]>=97&&str[index]<=122)
         {
         type[m] = 'a';
         }
         else if(str[index]>=48&&str[index]<=57)
         {
          type[m] = 'n';
         }
         index++;
         if( d && b )
         {
             str[index-2] = '\0';
             m++;
         }

         if(index==l)
                 {
             pos_arg[m] = '\0';
             type[m] = '\0';
             argc = m-1;
             itostring(argc);
             putsUart0("\r\n");
             //putsUart0(s);
             // putsUart0(" arguments were found\r\n");
             break;
                 }
     }

     }
 }
 }

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();


             RED_LED = 1;
             waitMicrosecond(500000);
             RED_LED = 0;
             waitMicrosecond(500000);


    // Display greeting
    putsUart0("enter the characters\r\n");

    while(1)
    {
    step2();
    putsUart0("\n\r");
    putsUart0(str);
    step3();
    //step4();
    //resetglobalvariables();
    }

}
