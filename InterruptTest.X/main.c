#include <stdio.h>
#include <stdlib.h>
#include <p32xxxx.h>
#include <stdio.h>
#include <strings.h>
#include <string.h>

/* Disable JTAG to use RA0 */
#pragma config JTAGEN = OFF
#pragma config FWDTEN = OFF

/* Device Config Bits in DEVCFG1: */
#pragma config FNOSC = 7
#pragma config FSOSCEN = OFF
#pragma config POSCMOD = XT
#pragma config OSCIOFNC = ON
#pragma config FPBDIV = DIV_2

/* Device Config Bits in DEVCFG2: */
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLMUL = MUL_20
#pragma config FPLLODIV = DIV_1

// funzione che abilita la gestione degli interrupt
#define macro_enable_interrupts(){\
unsigned int val = 0;\
asm volatile("mfc0 %0,$13" : "=r"(val));\
val |= 0x00800000;\
asm volatile("mtc0 %0,$13" : "+r"(val));\
INTCONbits.MVEC = 1;\
__builtin_enable_interrupts();}

// only with plib.h
//void __ISR(_TIMER_2_VECTOR, ipl1)
//Timer2IntHandler(void){
//    LATAINV=2;
//    IFS0bits.T2IF=0;
//}


void __attribute__((interrupt(ipl1), vector(_TIMER_2_VECTOR)))
Timer2IntHandler(void){
    LATAbits.LATA0^=1;
    IFS0bits.T2IF=0;
}

//#pragma interrupt Timer2IntHandler ipl1 vector 8
//void Timer2IntHandler(void){
//    LATAbits.LATA0^=1;
//    IFS0bits.T2IF=0;
//}


int main(int argc, char** argv) {
    
    macro_enable_interrupts();
    
    // init led0
    TRISAbits.TRISA0=0;
    LATAbits.LATA0=0;
    
    // init timer
    T2CONbits.ON = 0; // turn off the timer
    T2CONbits.T32 = 0; // set 16-bit mode
    T2CONbits.TCKPS = 7; //0b111, 0x7
    TMR2 = 0; // TMR is the counter, let's set it to 0
    PR2 = 39063; // target the counter must reach
    IEC0bits.T2IE=1; //enable interrupts
    
    IPC2bits.T2IP = 1; //priority
    IPC2bits.T2IS = 0; //sub priority
    
    T2CONbits.ON = 1; // Now it is safe to enable the timer

    while (1){
    }
    
}