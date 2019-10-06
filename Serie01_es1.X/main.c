/* 
 * File:   main.c
 * Author: alesto
 *
 * Created on September 20, 2019, 11:37 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include <p32xxxx.h>

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

void main() {
    
    // Porta A -> LEDS
    
    int i;
   
    // Tutti i led sono degli output
    TRISA=0x0000;
        
    while (1){
        
        // Tutti i led sono spenti inizialmente
        LATA=0x0000;

        for (i=0 ; i<8 ; i++){
            delay();
            switch(i){
                    case 0:
                        LATAbits.LATA0=1;
                        break;
                    case 1:
                        LATAbits.LATA1=1;
                        break;
                    case 2:
                        LATAbits.LATA2=1;
                        break;
                    case 3:
                        LATAbits.LATA3=1;
                        break;
                    case 4:
                        LATAbits.LATA4=1;
                        break;
                    case 5:
                        LATAbits.LATA5=1;                    
                        break;
                    case 6:
                        LATAbits.LATA6=1;                    
                        break;     
                    case 7:
                        LATAbits.LATA7=1;                 
                        break;                    
            }
        }
        // delay prima di spegnere tutti i led
        delay();
    }


}

