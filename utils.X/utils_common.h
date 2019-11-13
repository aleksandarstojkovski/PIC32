/* 
 * File:   utils_common.h
 * Author: alesto
 *
 * Created on November 10, 2019, 2:55 PM
 */

#ifndef UTILS_COMMON_H
#define	UTILS_COMMON_H

#ifdef	__cplusplus
extern "C" {
#endif
    void utils_common_delay(int counter);
    void utils_common_tolower(char* str);

#define utils_macro_enable_interrupts(){\
unsigned int val = 0;\
asm volatile("mfc0 %0,$13" : "=r"(val));\
val |= 0x00800000;\
asm volatile("mtc0 %0,$13" : "+r"(val));\
INTCONbits.MVEC = 1;\
__builtin_enable_interrupts();}

#ifdef	__cplusplus
}
#endif

#endif	/* UTILS_COMMON_H */

