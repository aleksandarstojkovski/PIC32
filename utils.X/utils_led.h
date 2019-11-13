/* 
 * File:   utils_led.h
 * Author: alesto
 *
 * Created on November 3, 2019, 3:35 PM
 */

#ifndef UTILS_LED_H
#define	UTILS_LED_H

#ifdef	__cplusplus
extern "C" {
#endif

    void utils_led_init();
    void utils_led_inv(int idx);
    void utils_led_set(int idx, int value);
    int utils_led_get(int idx);
    
#ifdef	__cplusplus
}
#endif

#endif	/* UTILS_LED_H */
