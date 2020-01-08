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
//  TIMERS
// **********************************

typedef enum { 
    TM1_DIV_1 = 0b00, 
    TM1_DIV_8 = 0b01, 
    TM1_DIV_64 = 0b10, 
    TM1_DIV_256 = 0b11 
} tm1_prescaler_t;

typedef enum { 
    TMx_DIV_1 = 0b000, 
    TMx_DIV_2 = 0b001, 
    TMx_DIV_4 = 0b010, 
    TMx_DIV_8 = 0b011, 
    TMx_DIV_16 = 0b100, 
    TMx_DIV_32 = 0b101, 
    TMx_DIV_64 = 0b110, 
    TMx_DIV_256 = 0b111 
} tmx_prescaler_t;

// Timer 1 prescaler 0..3: 1,8,64,256
int tm1_prescaler_vals[] = {1, 8, 64, 256};

// Timer 2,3,4,5 Prescaler 0..7: 1,2,4,8,16,32,64,256
int tmx_prescaler_vals[] = {1, 2, 4, 8, 16, 32, 64, 256};

// Calc Period Registration
int calc_pr(int period_ms, int peripheral_bus_freq, int prescaler_val) {
   int ms_to_micros = 1000;
   return  period_ms / ( (1.0/peripheral_bus_freq) * ms_to_micros * prescaler_val);
}

int calc_pr_16bit(int period_ms, int peripheral_bus_freq, int prescaler_val) {
    unsigned int pr = calc_pr(period_ms, peripheral_bus_freq, prescaler_val);
    if(pr > 0xFFFF) {
        int wrong_config = 1;
    }
    return pr;
}

void utils_timer1_init(
        int period_ms, int peripheral_bus_freq, tm1_prescaler_t prescaler, 
        int use_interrupt, int priority, int sub_priority) {
    T1CON = 0x00;                   // STOP timer 1 and reset configuration 
    T1CONbits.TCKPS = prescaler;    // set prescaler
    TMR1 = 0;                       // reset counters to 0
    PR1 = calc_pr_16bit(period_ms, peripheral_bus_freq, tm1_prescaler_vals[prescaler]); 
    
    // configure interrupt
    IPC1bits.T1IP = priority;
    IPC1bits.T1IS = sub_priority;
    IEC0bits.T1IE = use_interrupt;

}

void utils_timer2_init(
        int period_ms, int peripheral_bus_freq, tmx_prescaler_t prescaler, 
        int use_interrupt, int priority, int sub_priority) {
    T2CON = 0x00;                   // STOP timer 2 and reset configuration            
    T2CONbits.TCKPS = prescaler;    // set prescaler
    TMR2 = 0;                       // reset counters to 0 
    PR2 = calc_pr_16bit(period_ms, peripheral_bus_freq, tmx_prescaler_vals[prescaler]);
    
    // configure interrupt
    IEC0bits.T2IE = 0; // disable interrupt
    IFS0bits.T2IF = 0; //Interrupt flag put at zero
    IPC2bits.T2IP = priority;
    IPC2bits.T2IS = sub_priority;
    IEC0bits.T2IE = use_interrupt;
    
    T2CONbits.ON = 1; // turn ON the timer 
}

void utils_timer1_delay(int period_ms, int bus_freq, tm1_prescaler_t prescaler) {
    utils_timer1_init(period_ms, bus_freq, prescaler, 0, 0, 0);
    TMR1 = 0; 
    while (TMR1 < PR1); // wait
}

void utils_timer2_delay(int period_ms, int bus_freq, tmx_prescaler_t prescaler) {
    utils_timer2_init(period_ms, bus_freq, prescaler, 0, 0, 0);
    TMR2 = 0; 
    while (TMR2 < PR2); // wait
}

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
//  UART
// **********************************

void utils_uart4_config_pins();
void utils_uart4_config_uart(int baud, int pbusClock);

void utils_uart4_init(int baud, int pbus_clock, 
    int use_interrupt, int interrupt_priority, int interrupt_sub_priority) {
    utils_uart4_config_pins();
    utils_uart4_config_uart(baud, pbus_clock);
    if(use_interrupt) {
        // Micro Data Sheet - Tabella 7.2 INTERRUPT REGISTER MAP
        IPC9bits.U4IP = interrupt_priority;  
        IPC9bits.U4IS = interrupt_sub_priority;

        IFS2bits.U4RXIF = 0;    // Clear the Uart4 interrupt flag.
        IEC2bits.U4RXIE = 1;    // enable RX interrupt
    }
}

void utils_uart4_config_pins() {
    // Configure PINS
    // RP<n>:   Remappable Peripheral Port (Digital only) 
    //          Serial Comm, Timer, Interrupt on change
    
    // Table 12-2 Data Sheet - 0 PIN SELECTION
    TRISFbits.TRISF12 = 0;  // TX digital 0
    RPF12R = 0b0010;             // mapping U4TX (0010) to RPF12 

    // Table 12-1 Data Sheet - 1 PIN SELECTION
    TRISFbits.TRISF13 = 1;  // RX digital 1
    U4RXR = 0b1001;             // mapping RPF13 (1001) to U4RX
}

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

int utils_uart4_putc(int c) {
    while(U4STAbits.UTXBF == 1);  // wait while the buffer is full
    U4TXREG=c;                    // write the character to the output channel
}

char utils_uart4_getc(void) {
    while(!U4STAbits.URXDA);    //wait for a new char to arrive
    char ch = U4RXREG;          //read char from receive buffer
    return ch; 
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

int utils_uart4_gets(char* buffer, int max_sz) {
    int i=0;
    char ch = 0;
    for(i=0; i<max_sz && ch != '\r'; i++) {
        ch = utils_uart4_getc();
        buffer[i] = ch;
    }
    buffer[i-1] = 0;
    return i;
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

unsigned short *pAudioSamples;
// global variables that store audio buffer position and size
int cntAudioBuf, idxAudioBuf;
// This array contains the values that implement one syne period, over 25 samples. They are generated using this site: http://www.daycounter.com/Calculators/Sine-Generator-Calculator.phtml
unsigned short rgSinSamples[] = {
256,320,379,431,472,499,511,507,488,453,
406,350,288,224,162,106, 59, 24,  5,  1,
 13, 40, 81,133,192
};
#define RGSIN_SIZE  (sizeof(rgSinSamples) / sizeof(rgSinSamples[0]))
// the definitions for the SOUND and SINE frequencies, to be used when Timer3 is configured in specific modes. 
#define TMR_FREQ_SOUND   16000 // 16 kHz
#define TMR_FREQ_SINE   48000 // 48 kHz

void init_audio() {
    // config pins
    TRISBbits.TRISB14 = 0; // Audio Out configured as output
    ANSELBbits.ANSB14 = 0; // Audio Out disabled analog
    RPB14R = 0x0C; // OC1 // RB4 map to OC1
    // play sine
    PR3 = (int)((float)((float)periph_bus_clock_hz/TMR_FREQ_SINE) + 0.5); 
    // init playback
    idxAudioBuf = 0;
    cntAudioBuf = RGSIN_SIZE;
    pAudioSamples = rgSinSamples;
    OC1RS = pAudioSamples[0];
    // config timer and outputcompare
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

void set_audio(char * setting){
    if (setting == "on"){
        T3CONbits.ON = 1;       // turn off Timer3
        OC1CONbits.ON = 1;      // Turn off OC1  
    } else {
        T3CONbits.ON = 0;       // turn off Timer3
        OC1CONbits.ON = 0;      // Turn off OC1      
    }
}

// **********************************
//  ISRs
// **********************************

void __attribute__((interrupt(ipl1), vector(_TIMER_1_VECTOR)))
Timer1IntHandler(void){
    toggle_led_color("red");
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

    // init variables
    int button_btnc_pressed=0;
    int button_btnd_pressed=0;
    int button_btnr_pressed=0;
    int button_btnl_pressed=0;
    int button_btnu_pressed=0;
    int baud_rate=9600;
    char buffer[10];
    
    // enable interrupts
    utils_macro_enable_interrupts();
    
    // init everything
    init_lcd();
    init_adc();
    init_buttons();
    init_audio();
    utils_timer1_init(1000,periph_bus_clock_hz,3,1,1,1);    
    utils_uart4_init(baud_rate,periph_bus_clock_hz,1,1,1);
    
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
            set_audio("on");
            // wait TG time
            delay_timer2(TG*1000);
            set_audio("off");
            TMR1=0;
            set_led_color("red");
            T1CONbits.ON = 1;
            while (TR_counter>0);
            T1CONbits.ON = 0;
        }
        
        // right button pressed
        if (button_btnr_pressed){
            clear_and_write_to_lcd("Call DX");
            set_led_color("yellow");
            set_audio("on");
            // wait TG time
            delay_timer2(TG*1000);
            set_audio("off");
            TMR1=0;
            set_led_color("red");
            T1CONbits.ON = 1;
            while (TR_counter>0);
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