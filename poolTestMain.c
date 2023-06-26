/* 
 * File:   poolTestMain.c
 * Author: Robert Donovan
 *
 * Created on May 22, 2023, 11:30 PM
 */

#include <stdio.h>
#include <stdlib.h>

// PIC18F4520 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = INTIO7    // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTBE  // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

/*
 * 
 */
#define _XTAL_FREQ 8000000

#define Relay1           LATBbits.LB0
#define Relay2           LATBbits.LB1
#define Relay3           LATBbits.LB2
#define Relay4           LATBbits.LB3
#define Relay5           LATBbits.LB4
#define Relay6           LATBbits.LB5



#include "LCDpic18.h"
#include "poolTestMain.h"
#include "Uart.h"
#include "nvm.h"

//Global Variables
static uint16_t state1 = 0x0000;
static uint16_t state2 = 0x00;
static uint16_t state3 = 0x00;
static uint16_t state4 = 0x00;
static uint16_t state5 = 0x00;
unsigned char MenuButton = 0x00;
unsigned char NextButton = 0x00;
unsigned char IncButton = 0x00;
unsigned char DecButton = 0x00;
unsigned char PmpButton = 0x00;
unsigned char lcdIsWritten = 0x00;
char lcdLine1[16];
char lcdLine2[16];
char rx_string[100];
uint8_t rx_flag = 0;
char data = 0x00;
char data1[20];
uint8_t count = 0;


controlState_t controlState = RUN;


int main(void) { 
    
    OSCCON=0x72; // Select internal oscillator with frequency = 8MHz
    ADCON1 = 0x0e;
    TRISAbits.RA0 = 1;
    TRISAbits.RA1 = 1;
    TRISAbits.RA2 = 1;
    TRISAbits.RA3 = 1;
    TRISAbits.RA4 = 1;
    TRISAbits.RA5 = 0;
    TRISAbits.RA6 = 0;
    TRISAbits.RA7 = 0;
    TRISBbits.RB0 = 0;
    TRISBbits.RB1 = 0;
    TRISBbits.RB2 = 0;
    TRISBbits.RB3 = 0;
    TRISBbits.RB4 = 0;
    TRISBbits.RB5 = 0;
    TRISBbits.RB6 = 1;
    TRISBbits.RB7 = 1;
    TRISCbits.RC0 = 0;
    TRISCbits.RC1 = 1;
    TRISCbits.RC2 = 1;
    TRISCbits.RC3 = 0;
    TRISCbits.RC4 = 1;
    TRISCbits.RC5 = 0;
    TRISCbits.RC6 = 0;
    TRISCbits.RC7 = 1;
    TRISDbits.RD0 = 0;
    TRISDbits.RD1 = 0;
    TRISDbits.RD2 = 0;
    TRISDbits.RD3 = 0;
    TRISDbits.RD4 = 0;
    TRISDbits.RD5 = 0;
    TRISDbits.RD6 = 0;
    TRISDbits.RD7 = 0;
    TRISEbits.RE0 = 0;
    TRISEbits.RE1 = 0;
    TRISEbits.RE2 = 0;
    
    INTCON = 0xc0;
    INTCON2 = 0x00;
    INTCON3 = 0x00;


    
    InitT1();
    InitUart();
    InitNvm();
    LCD_Initialize();
    LCDPutStr(" Hello World!");         //Display String "Hello World"
    __delay_ms(500);
    LCDGoto(8,1);                            //Go to column 8 of second line
    LCDPutChar('1');                         //Display character '1'
    DisplayClr();                               // Clear the display

    LCDPutStr("Bobby has Done");       // Display a string "LCD Display"
    LCDGoto(0,1);                           //Go to second line 
    LCDPutStr("It Again!!");  
    Relay2 = 0; 

    while(1){
        
        if(rx_flag)
        {
            //LCDPutChar(data);
            data1[count] = data;
            count++;
            rx_flag = 0;
            if(count == 14)
            {
                count = 0;
                LCDLine2();
                LCDPutChar('1'); 
                LCDPutChar(' '); 
                LCDPutStr(data1); 
            }
        }
        
        if(MenuButton)
        {
            controlState = TOP_MENU;
            lcdIsWritten = 0;
            MenuButton = 0;
        }
        if(NextButton)
        {
            controlState = RUN;
            lcdIsWritten = 0;
            NextButton = 0;
        }
        if(IncButton)
        {
            controlState = START_TIME;
            lcdIsWritten = 0;
            IncButton = 0;
        }
        if(DecButton)
        {
            controlState = END_TIME;
            lcdIsWritten = 0;
            DecButton = 0;
        }
        switch(controlState)
        {
            case RUN:
            {
                if(!lcdIsWritten){
                    DisplayClr();
                    lcdLine1[0] = 'R';
                    lcdLine1[1] = 'u';
                    lcdLine1[2] = 'n';
                    lcdLine1[3] = 'n';
                    lcdLine1[4] = 'i';
                    lcdLine1[5] = 'n';
                    lcdLine1[6] = 'g';
                    LCDPutStr(lcdLine1); 
                    LCDLine2();
                    lcdLine2[0] = 'A';
                    lcdLine2[1] = ' ';
                    lcdLine2[2] = 'b';
                    lcdLine2[3] = 'e';
                    lcdLine2[4] = 't';
                    lcdLine2[5] = 't';
                    lcdLine2[6] = 'e';
                    lcdLine2[7] = 'r';
                    lcdLine2[8] = ' ';
                    lcdLine2[9] = 'w';
                    lcdLine2[10] = 'a';
                    lcdLine2[11] = 'y';
                    lcdLine2[12] = ' ';
                    lcdLine2[13] = ' ';
                    lcdLine2[14] = ' ';
                    lcdLine2[15] = ' ';
                    LCDPutStr(lcdLine2);  
                    lcdIsWritten = 1;
                }

                break;
            };
            case TOP_MENU:
            {
                if(!lcdIsWritten)
                {
                    DisplayClr();
                    lcdLine1[0] = 'T';
                    lcdLine1[1] = 'h';
                    lcdLine1[2] = 'i';
                    lcdLine1[3] = 's';
                    lcdLine1[4] = ' ';
                    lcdLine1[5] = 'i';
                    lcdLine1[6] = 's';
                    LCDPutStr(lcdLine1); 
                    LCDLine2();
                    lcdLine2[0] = 'M';
                    lcdLine2[1] = 'e';
                    lcdLine2[2] = 'n';
                    lcdLine2[3] = 'u';
                    lcdLine2[4] = ' ';
                    lcdLine2[5] = 'n';
                    lcdLine2[6] = 'u';
                    lcdLine2[7] = 'm';
                    lcdLine2[8] = 'b';
                    lcdLine2[9] = 'e';
                    lcdLine2[10] = 'r';
                    lcdLine2[11] = ' ';
                    lcdLine2[12] = 't';
                    lcdLine2[13] = 'w';
                    lcdLine2[14] = 'o';
                    lcdLine2[15] = ' ';
                    LCDPutStr(lcdLine2);  
                    lcdIsWritten = 1;
                }
                break;
            }
            case START_TIME:
            {
                if(!lcdIsWritten)
                {
                   WriteNvm(0x12, 0x37, 'A', 'G');
                    unsigned int result = ReadSpi(0x12, 0x37);
                    unsigned char lsb = result & 0x00ff;
                    result =  result >> 8;
                    unsigned char msb = result & 0x00ff;
                    DisplayClr();
                    lcdLine1[0] = 'R';
                    lcdLine1[1] = 'e';
                    lcdLine1[2] = 's';
                    lcdLine1[3] = 'u';
                    lcdLine1[4] = 'l';
                    lcdLine1[5] = 't';
                    lcdLine1[6] = ' ';
                    lcdLine1[7] = msb;
                    lcdLine1[8] = lsb;
                    lcdLine1[9] = ' ';
                    lcdLine1[10] = ' ';
                    lcdLine1[11] = ' ';
                    lcdLine1[12] = ' ';
                    lcdLine1[13] = ' ';
                    lcdLine1[14] = ' ';
                    lcdLine1[15] = ' ';
                    LCDPutStr(lcdLine1); 
                    LCDLine2();
                    lcdLine2[0] = 'B';
                    lcdLine2[1] = 'e';
                    lcdLine2[2] = 'e';
                    lcdLine2[3] = 't';
                    lcdLine2[4] = 'h';
                    lcdLine2[5] = 'o';
                    lcdLine2[6] = 'v';
                    lcdLine2[7] = 'a';
                    lcdLine2[8] = 'n';
                    lcdLine2[9] = ' ';
                    lcdLine2[10] = ' ';
                    lcdLine2[11] = ' ';
                    lcdLine2[12] = ' ';
                    lcdLine2[13] = ' ';
                    lcdLine2[14] = '5';
                    lcdLine2[15] = '9';
                    LCDPutStr(lcdLine2);  
                    lcdIsWritten = 1;
                }
            }
            case END_TIME:
            {
                if(!lcdIsWritten)
                {
                    WriteNvm(0x12, 0x34, 'R', 'D');
                    unsigned int result = ReadSpi(0x12, 0x34);
                    unsigned int temp = result;
                    temp = temp >> 8;
                    unsigned char lsb = result & 0x00ff;
                    unsigned char msb = temp & 0x00ff;
                    DisplayClr();
                    lcdLine1[0] = 'R';
                    lcdLine1[1] = 'e';
                    lcdLine1[2] = 's';
                    lcdLine1[3] = 'u';
                    lcdLine1[4] = 'l';
                    lcdLine1[5] = 't';
                    lcdLine1[6] = ' ';
                    lcdLine1[7] = msb;
                    lcdLine1[8] = lsb;
                    lcdLine1[9] = ' ';
                    lcdLine1[10] = ' ';
                    lcdLine1[11] = ' ';
                    lcdLine1[12] = ' ';
                    lcdLine1[13] = ' ';
                    lcdLine1[14] = ' ';
                    lcdLine1[15] = ' ';
                    LCDPutStr(lcdLine1); 
                    LCDLine2();
                    lcdLine2[0] = 'B';
                    lcdLine2[1] = 'e';
                    lcdLine2[2] = 'e';
                    lcdLine2[3] = 't';
                    lcdLine2[4] = 'h';
                    lcdLine2[5] = 'o';
                    lcdLine2[6] = 'v';
                    lcdLine2[7] = 'a';
                    lcdLine2[8] = 'n';
                    lcdLine2[9] = ' ';
                    lcdLine2[10] = ' ';
                    lcdLine2[11] = ' ';
                    lcdLine2[12] = ' ';
                    lcdLine2[13] = ' ';
                    lcdLine2[14] = '6';
                    lcdLine2[15] = '0';
                    LCDPutStr(lcdLine2);  
                    lcdIsWritten = 1;  
                }
            }
            __delay_ms(50); 
        }
        
        

    }    
    return (EXIT_SUCCESS);
}

void InitT1(void)
{
    T1CON = 0x01;
    PIE1bits.TMR1IE = 1;
    PIR1bits.TMR1IF = 0;
}



void __interrupt(high_priority) tcInt(void)
{
    
    if(PIR1bits.TMR1IF == 1)
    {
        PIE1bits.TMR1IE = 0;
        state1 = (state1 <<1) | PORTAbits.RA1 | 0x0000;
        state2 = (state2 <<1) | PORTAbits.RA2 | 0x0000;
        state3 = (state3 <<1) | PORTAbits.RA3 | 0x0000;
        state4 = (state4 <<1) | PORTAbits.RA4 | 0x0000;
        state5 = (state5 <<1) | PORTCbits.RC2 | 0x0000;
        if(state1 == 0xf000)
        {
            MenuButton = 1;
        }
        if(state2 == 0xf000)
        {
            NextButton = 1;
        }
        if(state3 == 0xf000)
        {
            IncButton = 1;
        }
        if(state4 == 0xf000)
        {
            DecButton = 1;
        }        
        if(state5 == 0xf000){
            PmpButton = 1;
            Relay1 = !Relay1;
        }
        else
        {
            PmpButton = 0;
        }
        TMR1H = 0xff;
        TMR1L = 0x80;
        
        PIE1bits.TMR1IE = 1;
        PIR1bits.TMR1IF = 0;
    }
    if(PIR1bits.RCIF == 1)
    {
        
        PIE1bits.RCIE = 0;

        if(RCSTAbits.FERR)
        {
            uint8_t error = RCREG;
        }
        if(RCSTAbits.OERR)
        {
            RCSTAbits.CREN = 0;
            RCSTAbits.CREN = 1;
        }
        data = RCREG;
        rx_flag = 1;
        PIR1bits.RCIF = 0;
        PIE1bits.RCIE = 1;
    }
    
}



