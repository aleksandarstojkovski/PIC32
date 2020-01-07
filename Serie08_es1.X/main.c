/* 
 * File:   main.c
 * Author: alesto
 *
 * Created on December 2, 2019, 6:48 PM
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

void __attribute__((interrupt(ipl1), vector(_TIMER_2_VECTOR)))
Timer2IntHandler(void){
    IFS0bits.T2IF = 0;
}

int main(int argc, char** argv) {
    utils_macro_enable_interrupts();
    
    TRISBbits.TRISB8=0;
    ANSELBbits.ANSB8=0;
    RPB8R=11;
    
    utils_timer2_init(20,10000000,2,1,1,0);
    
    int percentuale=3;
    int pressed_up=0;
    int pressed_down=0;
    
    OC5CON = 0x0000;
    OC5R = (PR2/100)*percentuale;
    OC5CON = 0x0006;
    OC5CONSET=0x8000;
    
    TRISBbits.TRISB1=1;
    ANSELBbits.ANSB1=0;
    
    TRISAbits.TRISA15=1;
    utils_led_init();
    
    while (1){
        if (PORTBbits.RB1 && ! pressed_up){
            pressed_up=1;
            if (percentuale < 12){
                percentuale++;
            }
        }else if (! PORTBbits.RB1 && pressed_up){
            pressed_up=0;
        }

        if (PORTAbits.RA15 && ! pressed_down){
            pressed_down=1;
            if (percentuale > 3){
                percentuale--;
            }
        }else if (! PORTAbits.RA15 && pressed_down){
            pressed_down=0;
        }
        OC5RS = (PR2/100)*percentuale;
    }
    
}

