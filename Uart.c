/* 
 * File:   Uart.c
 * Author: Robert Donovan
 *
 * Created on June 24, 2023, 9:53 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include "Uart.h"

void InitUart(void)
{
    PIR1bits.RCIF = 1;
    PIE1bits.RCIE = 1;
    IPR1bits.RCIP = 1;
    RCSTAbits.SPEN = 1;
    RCSTAbits.RX9 = 0;
    RCSTAbits.SREN = 0;
    RCSTAbits.CREN = 1;
    RCSTAbits.ADDEN = 0;
    SPBRG = 12;
    SPBRGH = 0;
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 0;
    TXSTAbits.TXEN = 0;
    BAUDCONbits.BRG16 = 0;
    BAUDCONbits.RXDTP = 0;
    BAUDCONbits.ABDEN = 0;
    
}


    



