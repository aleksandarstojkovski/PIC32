/* 
 * File:   main.c
 * Author: alesto
 *
 * Created on November 10, 2019, 12:56 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <p32xxxx.h>


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


void __attribute__((interrupt(ipl1), vector(7)))
Timer2IntHandler(void){
    LATAbits.LATA0^=1;
    IFS0bits.INT1IF=0;
}
/*
 * 
 */

int main(int argc, char** argv) {

    macro_enable_interrupts();
    
    // sw0 e' un input
    TRISFbits.TRISF3=1;
    // led0 e' un output
    TRISAbits.TRISA0=0;
    // mapping INT1 a RF3 (sw0)
    INT1R=0b1000;
    // abilito interrupt su INT1
    IEC0bits.INT1IE=1;
    // priorita'
    IPC1bits.INT1IP=1;
    // sotto priorita'
    IPC1bits.INT1IS=0;
    // interrup su rising edge, quando da 0 diventa 1
    INTCONbits.INT1EP = 0;
    
    
    while (1){
    
    }
    
}

