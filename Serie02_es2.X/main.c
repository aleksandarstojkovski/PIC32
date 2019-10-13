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
    
    char buffer[30];
    char LED0 [] = "LED0\r\n";
    char LED1 [] = "LED1\r\n";
    char LED2 [] = "LED2\r\n";
    char LED3 [] = "LED3\r\n";
    char LED4 [] = "LED4\r\n";
    char LED5 [] = "LED5\r\n";
    char LED6 [] = "LED6\r\n";
    char LED7 [] = "LED7\r\n";
    
    while (1){
        
        utils_uart_getU4_string(buffer,30);
        
        if ( strcmp(buffer, LED0) == 0){
            LATAbits.LATA0^=1;
            utils_uart_putU4_string(LED0);
        }

        if ( strcmp(buffer, LED1) == 0 ){
            LATAbits.LATA1^=1;
            utils_uart_putU4_string(LED1);
        }      
        
        if ( strcmp(buffer, LED2) == 0 ){
            LATAbits.LATA2^=1;
            utils_uart_putU4_string(LED2);
        }

        if ( strcmp(buffer, LED3) == 0 ){
            LATAbits.LATA3^=1;
            utils_uart_putU4_string(LED3);
        }

        if ( strcmp(buffer, LED4) == 0 ){
            LATAbits.LATA4^=1;
            utils_uart_putU4_string(LED4);
        }

        if ( strcmp(buffer, LED5) == 0 ){
            LATAbits.LATA5^=1;
            utils_uart_putU4_string(LED5);
        }

        if ( strcmp(buffer, LED6) == 0 ){
            LATAbits.LATA6^=1;
            utils_uart_putU4_string(LED6);
        }           

        if ( strcmp(buffer, LED7) == 0 ){
            LATAbits.LATA7^=1;
            utils_uart_putU4_string(LED7);
        }        
        
        if ( strcmp(buffer, "OFF\r\n") == 0 ){
            LATA=0x0000;
            utils_uart_putU4_string("ON");
        }     

        if ( strcmp(buffer, "ON\r\n") == 0 ){
            LATA=0xFFFF;
            utils_uart_putU4_string("OFF");
        }             
        
    }
    
}

