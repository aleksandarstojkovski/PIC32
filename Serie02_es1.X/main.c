/* 
 * File:   main.c
 * Author: alesto
 *
 * Created on October 6, 2019, 12:04 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <p32xxxx.h>
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
    
    // disabilito segnale analogico, su SW5, 6 e 7
    ANSELBbits.ANSB11 = 0; // RB11 (SW5) disabled analog
    ANSELBbits.ANSB10 = 0; // RB10 (SW6) disabled analog
    ANSELBbits.ANSB9 = 0; // RB9 (SW7) disabled analog
    
    while (1){
        
        // LED 0
        if (PORTFbits.RF3 == 1 && PORTAbits.RA0 == 0){
            LATAbits.LATA0=1;
            utils_uart_putU4_string("LED0 - ON\r\n");
        } else  if (PORTFbits.RF3 == 0 && PORTAbits.RA0 == 1){
            LATAbits.LATA0=0;
            utils_uart_putU4_string("LED0 - OFF\r\n");
        }
        
        // LED 1
        if (PORTFbits.RF5 == 1 && PORTAbits.RA1 == 0){
            LATAbits.LATA1=1;
            utils_uart_putU4_string("LED1 - ON\r\n");
        } else  if (PORTFbits.RF5 == 0 && PORTAbits.RA1 == 1){
            LATAbits.LATA1=0;
            utils_uart_putU4_string("LED1 - OFF\r\n");
        }
        
        // LED 2
        if (PORTFbits.RF4 == 1 && PORTAbits.RA2 == 0){
            LATAbits.LATA2=1;
            utils_uart_putU4_string("LED2 - ON\r\n");
        } else  if (PORTFbits.RF4 == 0 && PORTAbits.RA2 == 1){
            LATAbits.LATA2=0;
            utils_uart_putU4_string("LED2 - OFF\r\n");
        }
        
        // LED 3
        if (PORTDbits.RD15 == 1 && PORTAbits.RA3 == 0){
            LATAbits.LATA3=1;
            utils_uart_putU4_string("LED3 - ON\r\n");
        } else  if (PORTDbits.RD15 == 0 && PORTAbits.RA3 == 1){
            LATAbits.LATA3=0;
            utils_uart_putU4_string("LED3 - OFF\r\n");
        }
        
        // LED 4
        if (PORTDbits.RD14 == 1 && PORTAbits.RA4 == 0){
            LATAbits.LATA4=1;
            utils_uart_putU4_string("LED4 - ON\r\n");
        } else  if (PORTDbits.RD14 == 0 && PORTAbits.RA4 == 1){
            LATAbits.LATA4=0;
            utils_uart_putU4_string("LED4 - OFF\r\n");
        }
        
        // LED 5
        if (PORTBbits.RB11 == 1 && PORTAbits.RA5 == 0){
            LATAbits.LATA5=1;
            utils_uart_putU4_string("LED5 - ON\r\n");
        } else  if (PORTBbits.RB11 == 0 && PORTAbits.RA5 == 1){
            LATAbits.LATA5=0;
            utils_uart_putU4_string("LED5 - OFF\r\n");
        }
        
        // LED 6
        if (PORTBbits.RB10 == 1 && PORTAbits.RA6 == 0){
            LATAbits.LATA6=1;
            utils_uart_putU4_string("LED6 - ON\r\n");
        } else  if (PORTBbits.RB10 == 0 && PORTAbits.RA6 == 1){
            LATAbits.LATA6=0;
            utils_uart_putU4_string("LED6 - OFF\r\n");
        }
        
        // LED 7
        if (PORTBbits.RB9 == 1 && PORTAbits.RA7 == 0){
            LATAbits.LATA7=1;
            utils_uart_putU4_string("LED7 - ON\r\n");
        } else  if (PORTBbits.RB9 == 0 && PORTAbits.RA7 == 1){
            LATAbits.LATA7=0;
            utils_uart_putU4_string("LED7 - OFF\r\n");
        }        
        
    }
    
}

