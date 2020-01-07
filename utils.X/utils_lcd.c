/* 
 * File:   utils_lcd.c
 * Author: alesto
 *
 * Created on January 6, 2020, 11:48 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include <p32xxxx.h>
#include "utils_lcd.h"

void utils_init_lcd(int bus_freq){
    // PMP initialization
    // enable the PMP, long waits
    PMCON=0x83BF;
    // master mode 1
    PMMODE=0x3FF;
    // PMA0 enabled
    PMAEN=0x0001;
    utils_timer1_delay(36, bus_freq, 3);
    PMADDR=LCDCMD;
    PMDATA=0x38;
    utils_timer1_delay(1, bus_freq, 3);
    PMDATA=0x0c;
    utils_timer1_delay(1, bus_freq, 3);
    PMDATA=0x01;
    utils_timer1_delay(2, bus_freq, 3);
    PMDATA=0x06;
    utils_timer1_delay(2, bus_freq, 3);
}

char utils_lcd_read(int addr) {
    while(PMMODEbits.BUSY) {} // wait for PMP available
    
    PMADDR = addr; // select the command address
    int dummy = PMDATA; // init read cycle , dummy read
    
    while(PMMODEbits.BUSY) {} // wait for PMP to be available
    return (PMDATA); // read the status register
}

void utils_lcd_write(int addr, char c) {
    while (utils_lcd_busy()) {} // check busy flag of LCD
    while (PMMODEbits.BUSY) {} // wait for PMP available
    PMADDR = addr;
    PMDATA = c;
}

void utils_lcd_write_str(const char *str) {
    while (*str) {
        utils_lcd_put(*str++);
    }
}

