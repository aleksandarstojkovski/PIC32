/* 
 * File:   main.c
 * Author: alesto
 *
 * Created on January 5, 2020, 10:03 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <p32xxxx.h>
#include "../utils.X/utils_timer.h"
#include "../utils.X/utils_adc.h"

// **********************************
//  CLOCK
// **********************************

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
#pragma config FPLLODIV = DIV_2

/*
 * 
 */
int periph_bus_clock_hz = 20000000;

void delay(int period_ms){
    utils_timer1_delay(period_ms, periph_bus_clock_hz,3);
}

// **********************************
//  LCD
// **********************************

char readLCD(int addr);

#define LCDDATA 1
#define LCDCMD 0
#define PMDATA PMDIN
#define busyLCD() readLCD(LCDCMD) & 0x80
#define putLCD(d) writeLCD(LCDDATA, (d))
#define cmdLCD(c) writeLCD(LCDCMD,(c))
#define homeLCD() writeLCD(LCDCMD,2)
#define clrLCD() writeLCD(LCDCMD,1)


/*
 * 
 */

int peripheral_bus_freq=20000000;

void init_lcd(void){
    // PMP initialization
    // enable the PMP, long waits
    PMCON=0x83BF;
    // master mode 1
    PMMODE=0x3FF;
    // PMA0 enabled
    PMAEN=0x0001;
    delay(36);
    PMADDR=LCDCMD;
    PMDATA=0x38;
    delay(1);
    PMDATA=0x0c;
    delay(1);
    PMDATA=0x01;
    delay(2);
    PMDATA=0x06;
    delay(2);
}

void writeLCD(int addr, char c){
    while(busyLCD());
    delay(1);
    while(PMMODEbits.BUSY);
    PMADDR=addr;
    // content of PMDATA (8bit) will be sent to LCD
    PMDATA=c;
}

char readLCD(int addr){
    int dummy;
    while(PMMODEbits.BUSY);
    PMADDR=addr;
    dummy=PMDATA;
    while(PMMODEbits.BUSY);
    return(PMDATA);
}

void putsLCD(char *s){
    while (*s){
        putLCD(*s++);
    }
}

int main(int argc, char** argv) {
    
    utils_adc_init();
    init_lcd();
    char buffer[10];
    putsLCD("ADC Val:");
    
    while(1){
        cmdLCD(0x80 | 0x40);
        int read_number = utils_adc_get_int(100,periph_bus_clock_hz,3);
        sprintf(buffer, "%d", read_number);
        putsLCD(buffer);
    }
}

