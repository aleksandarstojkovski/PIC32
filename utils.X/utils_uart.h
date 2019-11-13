/* 
 * File:   utils_uart.h
 * Author: alesto
 *
 * Created on November 10, 2019, 2:41 PM
 */

#ifndef UTILS_UART_H
#define	UTILS_UART_H

#ifdef	__cplusplus
extern "C" {
#endif
    void utils_uart4_init(int baud, int pbus_clock, 
        int use_interrupt, int interrupt_priority, int interrupt_sub_priority);
    
    int utils_uart4_putc(int c);
    char utils_uart4_getc(void);
    void utils_uart4_puts(const char* buffer);
    int utils_uart4_gets(char* buffer, int max_sz);
    
#ifdef	__cplusplus
}
#endif

#endif	/* UTILS_UART_H */

