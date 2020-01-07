/* 
 * File:   main.c
 * Author: alesto
 *
 * Created on December 9, 2019, 7:46 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <p32xxxx.h>
#include "../utils.X/utils_common.h"
#include "../utils.X/utils_led.h"

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
#pragma config FPLLODIV = DIV_4

/*
 * 
 */

void init_spi1(){
    // from schematic RF2 <-> SPI_SI (SDI - Serial Data Input)  
    // set it to input
    TRISFbits.TRISF2 = 0;
    // from schematic RF7 <-> SPI_SO (SDO - Serial Data Output)
    // set it to output
    TRISFbits.TRISF7 = 1;
    // from schematic RF6 <-> SPI_SCK (Serial Clock)
    // set it to input 
    TRISFbits.TRISF6 = 0;
    // from datasheet - SDI1 mapping RF2
    RPF2R = 0b1000;
    // from datasheet - SDO1 mapping RF7
    SDI1R = 0x0F;
    // slave selection - Chip Select (CS)
    // set as output
    TRISFbits.TRISF8 = 1;
    // slave selection - Chip Select (CS)
    // high = disable comunication
    LATFbits.LATF8 = 1;
    
    SPI1CON = 0x8120;
    
    SPI1BRG = 15;
}

int write_spi1(int byte){
    while(!SPI1STATbits.SPITBE);
    SPI1BUF = byte;
    while(!SPI1STATbits.SPIRBF);
    return (int)SPI1BUF;
}

int main(int argc, char** argv) {
    
}

