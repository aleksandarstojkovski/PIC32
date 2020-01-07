/* 
 * File:   main.c
 * Author: alesto
 *
 * Created on November 24, 2019, 1:04 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <p32xxxx.h>
#include "../utils.X/utils_timer.h"

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

void Delay_ms(unsigned int time_to_wait){
    // 80 corresponds to 1ms with prescaler 256 and freq to 20Mhz
    int tmp_pr1=80*time_to_wait;
    TMR1=0;
    while(TMR1<tmp_pr1);
}

void init_lcd(void){
    // PMP initialization
    // enable the PMP, long waits
    PMCON=0x83BF;
    // master mode 1
    PMMODE=0x3FF;
    // PMA0 enabled
    PMAEN=0x0001;
    Delay_ms(36);
    PMADDR=LCDCMD;
    PMDATA=0x38;
    Delay_ms(1);
    PMDATA=0x0c;
    Delay_ms(1);
    PMDATA=0x01;
    Delay_ms(2);
    PMDATA=0x06;
    Delay_ms(2);
}

void writeLCD(int addr, char c){
    while(busyLCD());
    Delay_ms(1);
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
    int i;
    utils_timer1_init(100,peripheral_bus_freq,3,0,0,0);
    init_lcd();
    while(1){
        putsLCD("Ale Sto");
        // sposta cursore sulla cella 0x40
        // guaradre reference manual dell'LCD KS0066U per piu info
        cmdLCD(0x80 | 0x40);
        // cmdLCD(0b11000000); equivalente a sopra
        putsLCD("SERIE 6");
        for(i=0;i<1000000;i++){}
        cmdLCD(0b00000001);
        Delay_ms(1);
        for(i=0;i<1000000;i++){}
    }
}

