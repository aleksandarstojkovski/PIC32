/* 
 * File:   main.c
 * Author: alesto
 *
 * Created on October 6, 2019, 12:04 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <p32xxxx.h>
#include <stdio.h>
#include <strings.h>
#include <string.h>
#include "uart.h"

/* Disable JTAG to use RA0 */
#pragma config JTAGEN = OFF
#pragma config FWDTEN = OFF

/* Device Config Bits in DEVCFG1: */
#pragma config FNOSC = FRCPLL
#pragma config FSOSCEN = OFF
#pragma config POSCMOD = XT
#pragma config OSCIOFNC = ON
#pragma config FPBDIV = DIV_2

/* Device Config Bits in DEVCFG2: */
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLMUL = MUL_20
#pragma config FPLLODIV = DIV_1

#define DELAY 1000000 // 1 second

void delay(){
    int counter=DELAY;
    while(counter>0){
        counter--;
    }
}

int main(int argc, char** argv) {

    T2CONbits.ON = 0; // turn off the timer
    T2CONbits.T32 = 0; // set 16-bit mode
    T2CONbits.TCKPS = 7; //0b111, 0x7
    TMR2 = 0; // TMR is the counter, let's set it to 0
    PR2 = 39063; // target the counter must reach
    T2CONbits.ON = 1; // Now it is safe to enable the timer
    
    // uart configuration variables
    unsigned int baudRate = 9600;    
    utils_uart_ConfigurePins();
    utils_uart_ConfigureUart(baudRate);
    
    // Tutti i led sono degli output
    TRISA=0x0000;
    // Tutti i led sono spenti inizialmente
    // 1 = acceso, 0 = spento
    LATA=0x0000;
    // Tutti gli switch sono input
    // 1 = input, 0 = output
    TRISF=0xFFFF;
    
    char buffer[30];
    
    while (1){
        
        utils_uart_getU4_string(buffer,30);
        
        if (strcmp(buffer, "ledon\r\n") == 0){
            TMR2=0;
            while (TMR2<PR2){}
            // accendi primo led
            LATAINV=1;
            TMR2=0;
            while (TMR2<PR2){}
            LATAINV=2;
            TMR2=0;
            while (TMR2<PR2){}
            LATAINV=4;
            TMR2=0;
            while (TMR2<PR2){}
            LATAINV=8;
            TMR2=0;
            while (TMR2<PR2){}
            LATAINV=16;
            TMR2=0;
            while (TMR2<PR2){}
            LATAINV=32;
            TMR2=0;
            while (TMR2<PR2){}
            LATAINV=64;
            TMR2=0;
            while (TMR2<PR2){}
            LATAINV=128;
            TMR2=0;
            while (TMR2<PR2){}   
            // spegni tutti i led
            LATAINV=255;            
        }
        
    }
    
}

