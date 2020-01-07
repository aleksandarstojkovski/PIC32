/* 
 * File:   main.c
 * Author: alesto
 *
 * Created on January 5, 2020, 8:32 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <p32xxxx.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <xc.h>
#include <sys/attribs.h>
#include "../utils.X/utils_timer.h"
#include "../utils.X/utils_uart.h"

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
#pragma config FPLLODIV = DIV_4

// **********************************
//  GLOBAL VARIABLES
// **********************************

int periph_bus_clock_hz = 10000000;
char uart_buffer[50];
int TR=5;
int TR_counter=5;
int TG=3;
char command[30];
int new_char_pos = 0;
int new_word = 0;
unsigned short *pAudioSamples;
// global variables that store audio buffer position and size
int cntAudioBuf, idxAudioBuf;
// This array contains the values that implement one syne period, over 25 samples. 
// They are generated using this site: 
// http://www.daycounter.com/Calculators/Sine-Generator-Calculator.phtml
unsigned short rgSinSamples[] = {
256,320,379,431,472,499,511,507,488,453,
406,350,288,224,162,106, 59, 24,  5,  1,
 13, 40, 81,133,192
};
#define RGSIN_SIZE  (sizeof(rgSinSamples) / sizeof(rgSinSamples[0]))
// the definitions for the SOUND and SINE frequencies, to be used when Timer3 is configured in specific modes. 
#define TMR_FREQ_SOUND   16000 // 16 kHz
#define TMR_FREQ_SINE   48000 // 48 kHz

// **********************************
//  ENABLE INTERRUPTS FUNCTION
// **********************************

#define utils_macro_enable_interrupts(){\
unsigned int val = 0;\
asm volatile("mfc0 %0,$13" : "=r"(val));\
val |= 0x00800000;\
asm volatile("mtc0 %0,$13" : "+r"(val));\
INTCONbits.MVEC = 1;\
__builtin_enable_interrupts();}

// **********************************
//  DELAY TIMER2 FUNCTION
// **********************************

void delay_timer2(int period_ms){
    if (period_ms <= 1000){
        utils_timer2_delay(period_ms, periph_bus_clock_hz,7);
    } else {
        int i;
        for (i=0;i<period_ms/1000;i++){
            utils_timer2_delay(1000, periph_bus_clock_hz,7);
        }
    }
}

// **********************************
//  ADC
// **********************************

void init_adc() {
    ANSELB = 0xFFFB; // PORTB = Digital, RB = analog
    AD1CON1 = 0;     // ends sampling, start converting
    AD1CHS = 0x0020000; // Connect RB2/AN2 as CH0 input
    AD1CSSL = 0;
    AD1CON3 = 0x0002;   // manual Sample, Tad = internal 6 TPB
    AD1CON2 = 0;
    AD1CON1SET = 0x8000; // turn on the ADC
}

int adc_get_int(int delay_ms, int bus_freq, int prescaler) {
    AD1CON1SET = 0x0002; // start sampling
    utils_timer2_delay(delay_ms, bus_freq, prescaler);
    AD1CON1CLR = 0x0002; // start converting
    while(!(AD1CON1 & 0x0001)); // conversion done?
    return ADC1BUF0;
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

void init_lcd(void){
    // PMP initialization
    // enable the PMP, long waits
    PMCON=0x83BF;
    // master mode 1
    PMMODE=0x3FF;
    // PMA0 enabled
    PMAEN=0x0001;
    delay_timer2(36);
    PMADDR=LCDCMD;
    PMDATA=0x38;
    delay_timer2(1);
    PMDATA=0x0c;
    delay_timer2(1);
    PMDATA=0x01;
    delay_timer2(2);
    PMDATA=0x06;
    delay_timer2(2);
}

void writeLCD(int addr, char c){
    while(busyLCD());
    delay_timer2(1);
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

void clear_and_write_to_lcd(char * string){
    cmdLCD(0b00000001);
    delay_timer2(2);
    putsLCD(string);
}

void clear_lcd(){
    cmdLCD(0b00000001);
    delay_timer2(2);
}

// **********************************
//  GPIO
// **********************************

void init_buttons(){
    TRISBbits.TRISB1 = 1; // RB1 (BTNU) configured as input
    ANSELBbits.ANSB1 = 0; // RB1 (BTNU) disabled analog
    TRISBbits.TRISB0 = 1; // RB1 (BTNL) configured as input
    ANSELBbits.ANSB0 = 0; // RB1 (BTNL) disabled analog
    TRISFbits.TRISF4 = 1; // RF0 (BTNC) configured as input
    TRISBbits.TRISB8 = 1; // RB8 (BTNR) configured as input
    ANSELBbits.ANSB8 = 0; // RB8 (BTNR) disabled analog
    TRISAbits.TRISA15 = 1; // RA15 (BTND) configured as input
    TRISDbits.TRISD2 = 0; // RED led configured ad output
    TRISDbits.TRISD12 = 0; // GREEN led configured ad  output
    TRISDbits.TRISD3 = 0; // BLUE led configured ad  output
    TRISBbits.TRISB2 = 1; // Analog Input Control configured as input
    ANSELBbits.ANSB2 = 1; // Analog Input Control configured as analog
}

void set_led_color(char * color){
    LATDbits.LATD12 = 0;
    LATDbits.LATD2 = 0;
    LATDbits.LATD3 = 0;
    if (color == "green"){
        LATDbits.LATD12 = 1;
    }
    if (color == "red"){    
        LATDbits.LATD2 = 1;
    }
    if (color == "blue"){    
        LATDbits.LATD3 = 1;
    }
    if (color == "yellow"){    
        LATDbits.LATD12 = 1;
        LATDbits.LATD2 = 1;
    }      
}

void toggle_led_color(char * color){
    if (color == "green"){
        LATDbits.LATD2 = 0;
        LATDbits.LATD3 = 0;
        LATDbits.LATD12 ^= 1;
    }
    if (color == "red"){
        LATDbits.LATD3 = 0;
        LATDbits.LATD12 = 0;
        LATDbits.LATD2 ^= 1;
    }
    if (color == "blue"){   
        LATDbits.LATD2 = 0;
        LATDbits.LATD12 = 0;
        LATDbits.LATD3 ^= 1;
    }
    if (color == "yellow"){ 
        LATDbits.LATD3 = 0;
        LATDbits.LATD12 ^= 1;
        LATDbits.LATD2 ^= 1;
    }      
}

// **********************************
//  AUDIO
// **********************************

void AUDIO_ConfigurePins() {
    TRISBbits.TRISB14 = 0; // Audio Out configured as output
    ANSELBbits.ANSB14 = 0; // Audio Out disabled analog
    RPB14R = 0x0C; // OC1 // RB4 map to OC1
}

void AUDIO_InitPlayBack(unsigned short *pAudioSamples1, int cntBuf1) {

    // init playback
    idxAudioBuf = 0;
    cntAudioBuf = cntBuf1;
    pAudioSamples = pAudioSamples1;

    // load first value
    OC1RS = pAudioSamples[0];

}

void AUDIO_Init() {   
    AUDIO_ConfigurePins();
    // play sine
    PR3 = (int)((float)((float)periph_bus_clock_hz/TMR_FREQ_SINE) + 0.5);               
    AUDIO_InitPlayBack(rgSinSamples, RGSIN_SIZE);
    TMR3 = 0;
    T3CONbits.TCKPS = 0;     //1:1 prescale value
    T3CONbits.TGATE = 0;     //not gated input (the default)
    T3CONbits.TCS = 0;       //PCBLK input (the default)
    OC1CONbits.ON = 0;       // Turn off OC1 while doing setup.
    OC1CONbits.OCM = 6;      // PWM mode on OC1; Fault pin is disabled
    OC1CONbits.OCTSEL = 1;   // Timer3 is the clock source for this Output Compare module
    IPC3bits.T3IP = 7;      // interrupt priority
    IPC3bits.T3IS = 3;      // interrupt subpriority
    IEC0bits.T3IE = 1;      // enable Timer3 interrupt    
    IFS0bits.T3IF = 0;      // clear Timer3 interrupt flag
}

void AUDIO_Off() {
        T3CONbits.ON = 0;       // turn off Timer3
        OC1CONbits.ON = 0;      // Turn off OC1      
}

void AUDIO_On() {
        T3CONbits.ON = 1;       // turn off Timer3
        OC1CONbits.ON = 1;      // Turn off OC1      
}

// **********************************
//  ISRs
// **********************************

void __attribute__((interrupt(ipl1), vector(_TIMER_1_VECTOR)))
Timer1IntHandler(void){
    toggle_led_color("red");
    AUDIO_On();
    if (TR_counter>0){
        TR_counter--;
    }
    IFS0bits.T1IF=0; // interrupt served
}

void __attribute__((interrupt(ipl1), vector(_UART_4_VECTOR)))
Uart4IntHandler(void){
    char ch = U4RXREG;
    if(ch == '\r' || ch == '\n') {
        new_word = 1;
        ch = '\0';
    }
    command[new_char_pos] = ch;
    new_char_pos++;
    IFS2bits.U4RXIF=0; // interrupt served
    IFS2bits.U4TXIF=0; // interrupt served
}

void __ISR(_TIMER_3_VECTOR, IPL7AUTO) Timer3ISR(void) {  
    OC1RS = 4*pAudioSamples[(++idxAudioBuf) % cntAudioBuf];
    IFS0bits.T3IF = 0;
}

// **********************************
//  MAIN
// **********************************

int main(int argc, char** argv) {

    int button_btnc_pressed=0;
    int button_btnd_pressed=0;
    int button_btnr_pressed=0;
    int button_btnl_pressed=0;
    int button_btnu_pressed=0;
    int baud_rate=9600;
    char buffer[10];
    int i;
    
    utils_macro_enable_interrupts();
    
    init_lcd();
    init_adc();
    init_buttons();
    
    utils_timer1_init(1000,periph_bus_clock_hz,3,1,1,1);
    T1CONbits.ON = 0; // keep the timer stopped
    
    utils_uart4_init(baud_rate,periph_bus_clock_hz,1,1,1);
    utils_uart4_puts("STARTING...\n\r");

    AUDIO_Init();
    
    while (1){

        // button mapping
        button_btnc_pressed = PORTFbits.RF0; // btnc
        button_btnr_pressed = PORTBbits.RB8; // btnr
        button_btnl_pressed = PORTBbits.RB0; // btnl
        button_btnu_pressed = PORTBbits.RB1; // btnu
        
        // center button pressed
        if (button_btnc_pressed){
            set_led_color("blue");
            clear_and_write_to_lcd("config");
            while (!button_btnu_pressed){
                cmdLCD(0x80 | 0x40);
                TR = 3 + (adc_get_int(100,periph_bus_clock_hz,3)/1023.0)*7;
                TR_counter = TR;
                sprintf(buffer, "%02d", TR);
                putsLCD(buffer);
                button_btnu_pressed = PORTBbits.RB1;
            }
        }
        
        // left button pressed
        if (button_btnl_pressed == 1){
            clear_and_write_to_lcd("Call SX");
            set_led_color("yellow");
            // wait TG time
            delay_timer2(TG*1000);
            TMR1=0;
            T1CONbits.ON = 1;
            while (TR_counter>0);
            AUDIO_Off();
            T1CONbits.ON = 0;
        }
        
        // right button pressed
        if (button_btnr_pressed){
            clear_and_write_to_lcd("Call DX");
            set_led_color("yellow");
            // wait TG time
            delay_timer2(TG*1000);
            TMR1=0;
            T1CONbits.ON = 1;
            while (TR_counter>0);
            AUDIO_Off();
            T1CONbits.ON = 0;
        }
        
        // uart word typed
        if (new_word){
            if (strcmp(command, "datalog") == 0){
                sprintf(uart_buffer, "TR: %d, TG: %d\r\n", TR,TG);
                utils_uart4_puts(uart_buffer);
            }
            new_word=0;
            new_char_pos=0;
        }
        
        clear_lcd();
        TR_counter=TR;
        set_led_color("green");
        button_btnu_pressed = 0;
        button_btnc_pressed = 0;
        button_btnl_pressed = 0;
        button_btnr_pressed = 0;
  
    }
    
}