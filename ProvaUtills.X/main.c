/* 
 * File:   main.c
 * Author: alesto
 *
 * Created on November 10, 2019, 2:25 PM
 */

#include <stdio.h>
#include <stdlib.h>

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

/*
 * 
 */
int main(int argc, char** argv) {
    
}

