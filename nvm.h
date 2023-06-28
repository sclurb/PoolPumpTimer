/* 
 * File:                nvm.h
 * Author:              Robert Dononvan
 * Comments:            This is the header file for nvm.c which has methods for
 *                      writing and reading a Microchip 25LC256 SPI memory chip.
 * Revision history: 
 */

#ifndef nvm_H
#define	nvm_H

#include <xc.h> // include processor files - each processor file is guarded.  

void InitNvm(void);
unsigned char ReadStatusReg();
unsigned char WriteSpi(unsigned char  i);
unsigned int ReadSpi(unsigned int address);
void WriteNvm(unsigned int address, unsigned char dataMsb, unsigned char dataLsb);

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

