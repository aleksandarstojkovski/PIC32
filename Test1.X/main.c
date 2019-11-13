/* 
 * File:   main.c
 * Author: alesto
 *
 * Created on November 11, 2019, 6:11 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <p32xxxx.h>

// *********************************************
// CLOCK CONFIGURATION
// 8 MHz / (2*16*2) = 32 MHz [System Clock]
// 32 MHz / 2 = 16 MHz [Peripheral Bus Clock]
// *********************************************

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
#pragma config FPLLMUL = MUL_16
#pragma config FPLLODIV = DIV_2

// enable interrupts
#define utils_macro_enable_interrupts(){\
unsigned int val = 0;\
asm volatile("mfc0 %0,$13" : "=r"(val));\
val |= 0x00800000;\
asm volatile("mtc0 %0,$13" : "+r"(val));\
INTCONbits.MVEC = 1;\
__builtin_enable_interrupts();}

// *********************************************
// UART FUNCTIONS
// *********************************************

void utils_uart4_config_uart(int baud, int periph_bus_clock_hz) {
    U4MODEbits.ON = 0;      // UART OFF
    U4MODEbits.PDSEL = 0b00;// PDSEL - defines parity and number of data bits
                            // 0b11 -> [9 data bits, no parity]
                            // 0b10 -> [8 data bits, odd parity]
                            // 0b01 -> [8 data bits, even parity]
                            // 0b00 -> [8 data bits, no parity]

    U4MODEbits.UEN = 0b00;  // UEN: defines if use only TX and RX, or also RTS and CTS
                            // 0b00 -> [Only the UxTX and UxRX are used]
                            // 0b10 -> [UxTX, UxRX, UxCTS, and UxRTS are used]

    U4MODEbits.STSEL = 0;   // STSEL: defines number of stop bits
                            // 0 -> [1 stop bit], 1 -> [2 stop bits]
    
    U4MODEbits.SIDL = 0;    // SIDL: Stop in Idle Mode bit
    U4MODEbits.IREN = 0;    // IREN: IrDA Encoder and Decoder Enable bit
    U4MODEbits.RTSMD = 0;   // RTSMD: Mode Selection for UxRTS Pin bit
    U4MODEbits.WAKE = 0;    // WAKE: Enable Wake-up on Start bit Detect During Sleep Mode bit
    U4MODEbits.LPBACK = 0;  // LPBACK: UARTx Loopback Mode Select bit
    U4MODEbits.ABAUD = 0;   // ABAUD: Auto-Baud Enable bit
    U4MODEbits.RXINV = 0;   // RXINV: Receive Polarity Inversion bit
    U4MODEbits.BRGH = 0;    // BRGH: High Baud Rate Enable bit
                            // 1 = High-Speed mode ? 4x baud clock enabled
                            // 0 = Standard Speed mode ? 16x baud clock enabled
    // U4BRG port speed
    U4BRG  = (int)(((float)periph_bus_clock_hz / (16*baud) -1) + 0.5);
    
    U4STAbits.UTXEN = 1;    // abilita trasmissione
    U4STAbits.URXEN = 1;    // abilita ricezione
    U4MODEbits.ON = 1;      // UART ON
}

void utils_uart4_init(int baud, int pbus_clock) {
    // Configure PINS
    // RP<n>:   Remappable Peripheral Port (Digital only) 
    //          Serial Comm, Timer, Interrupt on change
    
    // Table 12-2 Data Sheet - 0 PIN SELECTION
    TRISFbits.TRISF12 = 0;  // TX digital 0
    RPF12R = 0b0010;             // mapping U4TX (0010) to RPF12 

    // Table 12-1 Data Sheet - 1 PIN SELECTION
    TRISFbits.TRISF13 = 1;  // RX digital 1
    U4RXR = 0b1001;             // mapping RPF13 (1001) to U4RX
    
    utils_uart4_config_uart(baud, pbus_clock);
        // Micro Data Sheet - Tabella 7.2 INTERRUPT REGISTER MAP
        IPC9bits.U4IP = 1;  
        IPC9bits.U4IS = 0;

        IFS2bits.U4RXIF = 0;    //Clear the Uart4 interrupt flag.
        IEC2bits.U4RXIE = 1;    // enable RX interrupt
}

int utils_uart4_putc(int c) {
    while(U4STAbits.UTXBF == 1);  // wait while the buffer is full
    U4TXREG=c;                    // write the character to the output channel
}

void utils_uart4_puts(const char* buffer) {
    size_t size = strlen(buffer);
    while(size > 0) {
        utils_uart4_putc(*buffer);
        buffer++;
        size--;
    }
    
    while( !U4STAbits.TRMT); // wait for last trasmission to finish
}

// *********************************************
// TIMER1 FUNCTIONS
// *********************************************

// Timer 1 prescaler 0..3: 1,8,64,256
int tm1_prescaler_vals[] = {1, 8, 64, 256};

// Timer 2,3,4,5 Prescaler 0..7: 1,2,4,8,16,32,64,256
int tmx_prescaler_vals[] = {1, 2, 4, 8, 16, 32, 64, 256};

int calc_pr(int period_ms, int bus_freq, int prescaler_val) {
   int ms_to_micros = 1000;
   return  period_ms / ( (1.0/bus_freq) * ms_to_micros * prescaler_val);
}

int calc_pr_16bit(int period_ms, int bus_freq, int prescaler_val) {
    unsigned int pr = calc_pr(period_ms, bus_freq, prescaler_val);
    if(pr > 0xFFFF) {
        int wrong_config = 1;
    }
    return pr;
}

void utils_timer1_init() {
    T1CON = 0x00;                   // STOP timer 1 and reset configuration 
    T1CONbits.TCKPS = 0b10;    // set prescaler
    TMR1 = 0;                       // reset counters to 0
    PR1 = 24999;
    // configure interrupt
    IPC1bits.T1IP = 1;
    IPC1bits.T1IS = 0;
    IEC0bits.T1IE = 1;      
    T1CONbits.ON = 1;
}

// *********************************************
// GLOBAL VARIABLES
// *********************************************

double dsec=0;
int sec=0;
int min=0;
int flag=0;

// *********************************************
// ISR
// *********************************************

void __attribute__((interrupt(ipl1), vector(_TIMER_1_VECTOR)))
Timer1IntHandler(void){
    //utils_uart4_puts("I am into the interrupt");
    // 100ms passed -> increment
    dsec+=1;
    if(dsec >= 10) {
        // 1 second passed -> set decs to 10 and increment seconds
        dsec = 0;
        sec++;
        if(sec >= 60) {
            // 60 seconds passed -> set sec to 0 and increment minutes
            sec = 0;
            min++;
        }
        // let the main print the time
        flag=1;
    }
    // interrupt served
    IFS0bits.T1IF=0;
}

// *********************************************
// MAIN
// *********************************************

int main(int argc, char** argv) {
    
    // baud rate as per request
    int baud_rate=115200;
    
    // peripheral bus clock as per request
    int pbus_clock=16000000;
    
    // buffer for printing text
    char buffer[50];
    
    // enable interrupts
    utils_macro_enable_interrupts();
    
    // uart configuration:
    //  - 8 bit
    //  - no parity
    //  - 1 stop bit
    //  - baud-rate 115200
    //  - no interrupts
    utils_uart4_init(baud_rate,pbus_clock);
    
    // timer config
    //  - irescaler 64
    //  - t_pr1 = 100ms
    //  - interrupts enabled
    utils_timer1_init();
    //  - timer1 starts off
    T1CONbits.ON = 0;
    
    // button config
    //  - must be an input
    TRISFbits.TRISF0=1;
    
    // print msg starting
    utils_uart4_puts("\n\rPIC32 is starting\n\r");
    
    while (1) {
        if (PORTFbits.RF0 == 1){
            //utils_uart4_puts("\n\rbutton pressed, starting the timer\n\r");
            // reset timer counter to 0
            TMR1=0;
            // start the timer
            T1CONbits.ON = 1;
        }
        if (flag == 1){
            sprintf(buffer, "TIME: %d:%d:%d\r\n", min,sec,(int)dsec);
            utils_uart4_puts(buffer);
            flag=0;
        }
    }
        
}

