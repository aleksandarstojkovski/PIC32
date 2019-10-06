/* 
 * File:   uart.h
 * Author: alesto
 *
 * Created on October 6, 2019, 11:57 AM
 */

#ifndef UART_H
#define	UART_H

#ifdef	__cplusplus
extern "C" {
#endif

void utils_uart_ConfigurePins(void);
void utils_uart_ConfigureUart(int baud);
int utils_uart_putU4(int c);
char utils_uart_getU4(void);
void utils_uart_putU4_string(const char* buffer);
int utils_uart_getU4_string(char* buffer, int max_sz);

#ifdef	__cplusplus
}
#endif

#endif	/* UART_H */

