/* 
 * File:   utils_switch.h
 * Author: alesto
 *
 * Created on November 10, 2019, 2:32 PM
 */

#ifndef UTILS_SWITCH_H
#define	UTILS_SWITCH_H

#ifdef	__cplusplus
extern "C" {
#endif

    void utils_switch_init();
    void utils_switch_init_sw0_interrupt_cn(int priority, int subpriority);
    int utils_switch_get(int idx);

#ifdef	__cplusplus
}
#endif

#endif	/* UTILS_SWITCH_H */

