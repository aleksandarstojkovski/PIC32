/* 
 * File:   main.c
 * Author: alesto
 *
 * Created on October 13, 2019, 8:41 PM
 */

/* Pragma definitio for clock configuration */
// Device Config Bits in DEVCFG1 :
# pragma config FNOSC = FRCPLL // Select XTPLL , HSPLL , ECPLL , FRCPLL in FNOXC mux
# pragma config FSOSCEN = OFF // Disable Secondary oscillator
# pragma config POSCMOD = XT // external crystal / resonator oscillator modes
# pragma config OSCIOFNC = ON // CLKO Enable Configuration bit
# pragma config FPBDIV = DIV_2 // Peripheral Bus Clock Divisor
// Device Config Bits in DEVCFG2 :
# pragma config FPLLIDIV = DIV_2 // PLL Input Divider
# pragma config FPLLMUL = MUL_20 // PLL Multiplier
# pragma config FPLLODIV = DIV_2 // PLL Output Divider
// disable JTAG
# pragma config JTAGEN = OFF
# pragma config FWDTEN = OFF
#include <p32xxxx.h>

int main(int argc, char** argv) {

    // DOMANDA 1:
    // La peripheral bus frequency che ne deriva `e 20 MHz.
    // Vogliamo utilizzare il TIMER2 con un prescaler di 256.
    // Che valore di match value deve contenere il registro PR2 affinch´e il periodo del TIMER2 sia di 0.5 secondi (500 ms)?
    // T_pr1 = 500ms 
    // f_clk (pheripheral clock frequency) = 20 Mhz
    // 20 Mhz to Hz --> 20 * 10^-3
    // t_clk (clock time) = 1 / f_clk (in Hz) --> 1/(20*10^-3))
    // PR2 value = T_pr1 / (T_clk * PDR)) --> 39062.5
    
    
    // turn off the timer
    T2CONbits.ON = 0;
    // set 32-bit mode
    T2CONbits.T32 = 0;
    // set the prescaler to 256
    T2CONbits.TCKPS = 7; //0b111, 0x7
    // TMR is the counter, let's set it to 0
    TMR2 = 0;  
    // Peripheral bus frequency == 20 Mhz
    // 39063 calcolato sopra
    PR2 = 39063;
    // Now it is safe to enable the timer
    T2CONbits.ON = 1;
    
    // Tutti i led sono degli output
    TRISA=0x0000;
    // Tutti i led sono spenti inizialmente, 1 = acceso, 0 = spento
    LATA=0x0000;
   
    
    while (1){
        while (TMR2<PR2){}
        TMR2=0;
        LATAINV = 1;
    }
    
}

