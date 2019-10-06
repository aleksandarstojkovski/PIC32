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


void main() {
    
    // Porta A -> LEDS
    // Porta F -> Switches
    
    int i;
   
    // Tutti i led sono degli output
    TRISA=0x0000;
    // Tutti i led sono spenti inizialmente
    // 1 = acceso, 0 = spento
    LATA=0x0000;
    // Tutti gli switch sono input
    // 1 = input, 0 = output
    TRISF=0xFFFF;
    
    // disabilito segnale analogico, su SW5, 6 e 7
    ANSELBbits.ANSB11 = 0; // RB11 (SW5) disabled analog
    ANSELBbits.ANSB10 = 0; // RB10 (SW6) disabled analog
    ANSELBbits.ANSB9 = 0; // RB9 (SW7) disabled analog
        
    while (1){
        LATAbits.LATA0 = PORTFbits.RF3; // accendo il led0 se switch 0 on
        LATAbits.LATA1 = PORTFbits.RF5; // accendo il led1 se switch 1 on
        LATAbits.LATA2 = PORTFbits.RF4; // accendo il led2 se switch 2 on
        LATAbits.LATA3 = PORTDbits.RD15; // accendo il led3 se switch 3 on
        LATAbits.LATA4 = PORTDbits.RD14; // accendo il led4 se switch 4 on
        LATAbits.LATA5 = PORTBbits.RB11; // accendo il led5 se switch 5 on
        LATAbits.LATA6 = PORTBbits.RB10; // accendo il led6 se switch 6 on
        LATAbits.LATA7 = PORTBbits.RB9; // accendo il led7 se switch 7 on
    }


}

