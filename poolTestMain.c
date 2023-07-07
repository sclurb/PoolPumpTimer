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
#pragma config OSC = INTIO67    // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
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
#define StartTimeAddress  0x1001
#define EndTimeAddress  0x1003
#define TimeOffsettAddress  0x1005
#define IsTwelveHourClock   0x1006



#include "LCDpic18.h"
#include "poolTestMain.h"
#include "Uart.h"
#include "nvm.h"

//Global Variables
static uint16_t state1 = 0x0000;
static uint16_t state2 = 0x0000;
static uint16_t state3 = 0x0000;
static uint16_t state4 = 0x0000;
static uint16_t state5 = 0x0000;
unsigned char MenuButton = 0x00;
unsigned char NextButton = 0x00;
unsigned char IncButton = 0x00;
unsigned char DecButton = 0x00;
unsigned char PmpButton = 0x00;
unsigned char lcdIsWritten = 0x00;
static char lcdLine1[17];
static char lcdLine2[17];
char rx_string[100];
uint8_t rx_flag = 0;
char data = 0x00;
uint8_t count = 0;
uint8_t rxIndex = 0;
int time_offset = -4;
int twelve_hour = 0;
unsigned int current_time = 0x0000;
timeNumber_t current_adjusted_time;
timeNumber_t start_adjusted_time;
timeNumber_t end_adjusted_time;
unsigned int start_time;
unsigned int temp_start_time;
unsigned int end_time;
unsigned int temp_end_time;
int temp_hour = 0;
int temp_min = 0;
timeNumber_t displayed_start_time;
timeNumber_t displayed_end_time;
char man_auto[4];
unsigned char mode;


controlState_t controlState = RUN;
rxDataState rxState = NOT_VALID;


int main(void) { 
    
    OSCCON = 0x72; // Select internal oscillator with frequency = 8MHz
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
    
    INTCON  =   0xc0;
    INTCON2 =   0x00;
    INTCON3 =   0x00;
    
    LATEbits.LE0 = 1;
    LATEbits.LE1 = 1;
    LATEbits.LE2 = 1;

    InitT1();
    InitUart();
    InitNvm();
    LCD_Initialize();
    // WriteNvm(StartTimeAddress, 0x0c, 0x1e);
    // WriteNvm(EndTimeAddress, 0x16, 0x1e);
    start_time = ReadSpi(StartTimeAddress);
    temp_start_time = ReadSpi(StartTimeAddress);
    end_time = ReadSpi(EndTimeAddress);
    temp_end_time = ReadSpi(EndTimeAddress);
    twelve_hour = 1;
    current_adjusted_time.am_pm = 0;
    current_adjusted_time.time = 0;
    Relay1 = 0;
    mode = 1;
    
    LCDPutStr("Pool Pump Timer"); 
    LCDLine2(); 
    LCDPutStr("Using GPS antenna"); 
    __delay_ms(500);

    while(1){
        if(IncButton && controlState != RUN)
        {
            temp_min = temp_min + 10;
            if(temp_min >= 60)
            {
                temp_hour++;
                if(temp_hour > 24)
                    temp_hour = 24;
                temp_min = 0;
            }
            if(controlState == START_TIME)
            {
                start_time = (unsigned int)((temp_hour << 8) | temp_min);             
            }
            else if(controlState == END_TIME)
            {
                end_time = (unsigned int)((temp_hour << 8) | temp_min); 
            }
            lcdIsWritten = 0;
            IncButton = 0;
        }
        else if(DecButton && controlState != RUN)
        {
            temp_min = temp_min - 10;
            if(temp_min <= 0)
            {
                temp_hour--;
                if(temp_hour < 0)
                    temp_hour = 0;
                temp_min = 50;
            }
            if(controlState == START_TIME)
            {
                start_time = (unsigned int)((temp_hour << 8) | temp_min);             
            }
            else if(controlState == END_TIME)
            {
                end_time = (unsigned int)((temp_hour << 8) | temp_min);                 
            }
            lcdIsWritten = 0;
           DecButton = 0;
        }
        else if(DecButton && controlState == RUN)
        {
            mode = !mode;
             DecButton = 0;
        }
        switch(controlState)
        {
            case RUN:
            {
                PIE1bits.RCIE = 1;
                timeNumber_t displayed_current_time;
                if(rxState == COMPLETE)
                {
                    char result[16];
                    getDate(rx_string, result);
                    displayed_current_time = convertGpsDataToTimeNumber(rx_string[1], rx_string[2], rx_string[3], rx_string[4]);
                    current_time = displayed_current_time.time;

                    LCDLine1();
                    lcdLine1[0] = man_auto[0];
                    lcdLine1[1] = man_auto[1];
                    lcdLine1[2] = man_auto[2];
                    lcdLine1[3] = ' ';
                    lcdLine1[4] = ' ';
                    lcdLine1[5] = ' ';
                    lcdLine1[6] = displayed_current_time.hour_msb;
                    lcdLine1[7] = displayed_current_time.hour_lsb;
                    lcdLine1[8] = ':';
                    lcdLine1[9] = displayed_current_time.min_msb;
                    lcdLine1[10] = displayed_current_time.min_lsb;
                    lcdLine1[11] = ':';
                    lcdLine1[12] = rx_string[5];
                    lcdLine1[13] = rx_string[6];
                    lcdLine1[14] = displayed_current_time.a_or_p;
                    lcdLine1[15] = displayed_current_time.mmm;
                    lcdLine1[16] = '\0';
                    LCDPutStr(lcdLine1);
                   
                    LCDLine2();
                    lcdLine2[0] = ' ';
                    lcdLine2[1] = ' ';
                    lcdLine2[2] = ' ';
                    lcdLine2[3] = ' ';
                    lcdLine2[4] = ' ';
                    lcdLine2[5] = ' ';
                    lcdLine2[6] = result[0];
                    lcdLine2[7] = result[1];
                    lcdLine2[8] = '/';
                    lcdLine2[9] = result[2];
                    lcdLine2[10] = result[3];
                    lcdLine2[11] = '/';
                    lcdLine2[12] = result[4];
                    lcdLine2[13] = result[5];
                    lcdLine2[14] = ' ';
                    lcdLine2[15] = ' ';
                    lcdLine2[16] = '\0';
                    LCDPutStr(lcdLine2);
                    rxState = NOT_VALID;
                    if(count <= 20)
                        count++;
                }
                break;
            };
            case START_TIME:
            {
                PIE1bits.RCIE = 0;
                if(!lcdIsWritten)
                {
                    displayed_start_time = convertStoredTimeToTimeNumber(start_time);
                    temp_hour = displayed_start_time.hour;
                    temp_min = displayed_start_time.mins;
                    DisplayClr();
                    lcdLine1[0] = 'S';
                    lcdLine1[1] = 't';
                    lcdLine1[2] = 'a';
                    lcdLine1[3] = 'r';
                    lcdLine1[4] = 't';
                    lcdLine1[5] = ' ';
                    lcdLine1[6] = 'T';
                    lcdLine1[7] = 'i';
                    lcdLine1[8] = 'm';
                    lcdLine1[9] = 'e';
                    lcdLine1[10] = ' ';
                    lcdLine1[11] = ' ';
                    lcdLine1[12] = ' ';
                    lcdLine1[13] = ' ';
                    lcdLine1[14] = ' ';
                    lcdLine1[15] = ' ';
                    lcdLine1[16] = '\0';
                    LCDPutStr(lcdLine1); 
                    LCDLine2();
                    lcdLine2[0] = displayed_start_time.hour_msb;
                    lcdLine2[1] = displayed_start_time.hour_lsb;
                    lcdLine2[2] = ':';
                    lcdLine2[3] = displayed_start_time.min_msb;
                    lcdLine2[4] = displayed_start_time.min_lsb;
                    lcdLine2[5] = displayed_start_time.a_or_p;
                    lcdLine2[6] = displayed_start_time.mmm;
                    lcdLine2[7] = ' ';
                    lcdLine2[8] = ' ';
                    lcdLine2[9] = ' ';
                    lcdLine2[10] = ' ';
                    lcdLine2[11] = ' ';
                    lcdLine2[12] = ' ';
                    lcdLine2[13] = ' ';
                    lcdLine2[14] = '5';
                    lcdLine2[15] = '9';
                    lcdLine2[16] = '\0';
                    LCDPutStr(lcdLine2);  
                    lcdIsWritten = 1;
                }
                break;
            }
            case END_TIME:
            {
                PIE1bits.RCIE = 0;
                if(!lcdIsWritten)
                {

                    displayed_end_time = convertStoredTimeToTimeNumber(end_time);
                    temp_hour = displayed_end_time.hour;
                    temp_min = displayed_end_time.mins;
                    DisplayClr();
                    lcdLine1[0] = 'E';
                    lcdLine1[1] = 'n';
                    lcdLine1[2] = 'd';
                    lcdLine1[3] = ' ';
                    lcdLine1[4] = 'T';
                    lcdLine1[5] = 'i';
                    lcdLine1[6] = 'm';
                    lcdLine1[7] = 'e';
                    lcdLine1[8] = ' ';
                    lcdLine1[9] = ' ';
                    lcdLine1[10] = ' ';
                    lcdLine1[11] = ' ';
                    lcdLine1[12] = ' ';
                    lcdLine1[13] = ' ';
                    lcdLine1[14] = ' ';
                    lcdLine1[15] = ' ';
                    lcdLine1[16] = '\0';
                    LCDPutStr(lcdLine1); 
                    LCDLine2();
                    lcdLine2[0] = displayed_end_time.hour_msb;
                    lcdLine2[1] = displayed_end_time.hour_lsb;
                    lcdLine2[2] = ':';
                    lcdLine2[3] = displayed_end_time.min_msb;
                    lcdLine2[4] = displayed_end_time.min_lsb;
                    lcdLine2[5] = displayed_end_time.a_or_p;
                    lcdLine2[6] = displayed_end_time.mmm;
                    lcdLine2[7] = ' ';
                    lcdLine2[8] = ' ';
                    lcdLine2[9] = ' ';
                    lcdLine2[10] = ' ';
                    lcdLine2[11] = ' ';
                    lcdLine2[12] = ' ';
                    lcdLine2[13] = ' ';
                    lcdLine2[14] = '6';
                    lcdLine2[15] = '0';
                    lcdLine2[16] = '\0';
                    LCDPutStr(lcdLine2);  
                    lcdIsWritten = 1; 
                }
            }
            case OFFSET_TIME:
            {
                if(!lcdIsWritten)
                {
                    PIE1bits.RCIE = 0;
                    DisplayClr();
                    lcdLine1[0] = 'O';
                    lcdLine1[1] = 'f';
                    lcdLine1[2] = 'f';
                    lcdLine1[3] = 's';
                    lcdLine1[4] = 'e';
                    lcdLine1[5] = 't';
                    lcdLine1[6] = ' ';
                    lcdLine1[7] = 'T';
                    lcdLine1[8] = 'i';
                    lcdLine1[9] = 'm';
                    lcdLine1[10] = 'e';
                    lcdLine1[11] = ' ';
                    lcdLine1[12] = ' ';
                    lcdLine1[13] = ' ';
                    lcdLine1[14] = ' ';
                    lcdLine1[15] = ' ';
                    lcdLine1[16] = '\0';
                    LCDPutStr(lcdLine1); 
                    LCDLine2();
                    lcdLine2[0] = '-';
                    lcdLine2[1] = '0';
                    lcdLine2[2] = '4';
                    lcdLine2[3] = ' ';
                    lcdLine2[4] = 'H';
                    lcdLine2[5] = 'o';
                    lcdLine2[6] = 'u';
                    lcdLine2[7] = 'r';
                    lcdLine2[8] = 's';
                    lcdLine2[9] = ' ';
                    lcdLine2[10] = ' ';
                    lcdLine2[11] = ' ';
                    lcdLine2[12] = ' ';
                    lcdLine2[13] = ' ';
                    lcdLine2[14] = ' ';
                    lcdLine2[15] = ' ';
                    lcdLine2[16] = '\0';
                    LCDPutStr(lcdLine2);  
                    lcdIsWritten = 1;
                }
            }
            case TWELVE_TWENTYFOUR:
            {
                if(!lcdIsWritten)
                {
                    PIE1bits.RCIE = 0;
                    DisplayClr();
                    lcdLine1[0] = 'U';
                    lcdLine1[1] = 's';
                    lcdLine1[2] = 'i';
                    lcdLine1[3] = 'n';
                    lcdLine1[4] = 'g';
                    lcdLine1[5] = ' ';
                    lcdLine1[6] = '1';
                    lcdLine1[7] = '2';
                    lcdLine1[8] = ' ';
                    lcdLine1[9] = 'H';
                    lcdLine1[10] = 'o';
                    lcdLine1[11] = 'u';
                    lcdLine1[12] = 'r';
                    lcdLine1[13] = '?';
                    lcdLine1[14] = ' ';
                    lcdLine1[15] = ' ';
                    lcdLine1[16] = '\0';
                    LCDPutStr(lcdLine1); 
                    LCDLine2();
                    lcdLine2[0] = ' ';
                    lcdLine2[1] = ' ';
                    lcdLine2[2] = ' ';
                    lcdLine2[3] = ' ';
                    lcdLine2[4] = ' ';
                    lcdLine2[5] = ' ';
                    lcdLine2[6] = ' ';
                    lcdLine2[7] = ' ';
                    lcdLine2[8] = ' ';
                    lcdLine2[9] = ' ';
                    lcdLine2[10] = ' ';
                    lcdLine2[11] = ' ';
                    lcdLine2[12] = ' ';
                    lcdLine2[13] = ' ';
                    lcdLine2[14] = ' ';
                    lcdLine2[15] = ' ';
                    lcdLine2[16] = '\0';
                    LCDPutStr(lcdLine2);  
                    lcdIsWritten = 1;
                }
            }
        }
        if(start_time != temp_start_time)
        {
            unsigned char a = (unsigned char)((start_time & 0xff00) >> 8);
            unsigned char b = (unsigned char)(start_time & 0x00ff);
            WriteNvm(StartTimeAddress, a, b);
        }
        if(end_time != temp_end_time)
        {
            unsigned char a = (unsigned char)((end_time & 0xff00) >> 8);
            unsigned char b = (unsigned char)(end_time & 0x00ff);
            WriteNvm(EndTimeAddress, a, b);
        }
        if(!mode)
        {
            man_auto[0] = 'm';
            man_auto[1] = 'a';
            man_auto[2] = 'n';
            man_auto[3] = '\0';
        }
        else
        {
            man_auto[0] = 'R';
            man_auto[1] = 'u';
            man_auto[2] = 'n';
            man_auto[3] = '\0';
            if(count > 20)
            {
                if(start_time < current_time && current_time < end_time)
                {
                    Relay1 = 1;
                }
                else
                {
                    Relay1 = 0;
                }
            }
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


timeNumber_t convertGpsDataToTimeNumber(unsigned char hour_msb, unsigned char hour_lsb, unsigned char min_msb, unsigned char min_lsb)
{
    timeNumber_t result;
    result.hour = (int)((hour_msb & 0x0f) * 10) + (hour_lsb & 0x0f);
    result.mins = (int)((min_msb & 0x0f) * 10) + (min_lsb & 0x0f);
    result.time = (unsigned int)((result.hour << 8) | result.mins);
    result.adjustedHour = result.hour + time_offset;
    if (result.adjustedHour > 11)
    {
        result.am_pm = 1;
        result.a_or_p = 'p';
    }
    else
    {
        result.am_pm = 0;
        result.a_or_p = 'a';
    }
    if(result.am_pm)
        result.adjustedTwelveHour = result.adjustedHour - 12;
    else
        result.adjustedTwelveHour = result.adjustedHour;
    if(twelve_hour)
    {
        result.hour_msb = (unsigned char)(((result.adjustedTwelveHour / 10) & 0x000f) | 0x30);
        result.hour_lsb = (unsigned char)(((result.adjustedTwelveHour % 10) & 0x000f) | 0x30);       
    }
    else
    {
        result.hour_msb = (unsigned char)(((result.adjustedHour / 10) & 0x000f) | 0x30);
        result.hour_lsb = (unsigned char)(((result.adjustedHour % 10) & 0x000f) | 0x30);        
    }
    result.min_msb = (unsigned char)(((result.mins / 10) & 0x000f) | 0x30);
    result.min_lsb = (unsigned char)(((result.mins % 10) & 0x000f) | 0x30);
    result.mmm = 'm';
    
    return result;
}

timeNumber_t convertStoredTimeToTimeNumber(unsigned int storedTime)
{
    timeNumber_t result;
    unsigned char hour_msn, hour_lsn, min_msn, min_lsn;
    hour_msn =  (unsigned char)(((storedTime & 0xff00) >> 8) / 10);
    hour_lsn =  (unsigned char)(((storedTime & 0xff00) >> 8) % 10);
    min_msn =   (unsigned char)((storedTime & 0x00ff) / 10);
    min_lsn =   (unsigned char)((storedTime & 0x00ff) % 10);
    result = convertGpsDataToTimeNumber(hour_msn, hour_lsn, min_msn, min_lsn);
    return result;
}

void getDate(char gpsData[], char result[])
{
    unsigned char count = 0;
    for (int i = 0; i < 100; i++ )
    {
        if(gpsData[i] == ',')
            count++;
        if(count == 8)
        {
            if(gpsData[i + 4] == '0')
                result[0] = ' ';
            else
            result[0] = gpsData[i + 4];
            result[1] = gpsData[i + 5];
            result[2] = gpsData[i + 2];
            result[3] = gpsData[i + 3];
            result[4] = gpsData[i + 6];
            result[5] = gpsData[i + 7];
        }
    }
}

void __interrupt(high_priority) tcInt(void)
{
    if(PIR1bits.TMR1IF == 1 && PIE1bits.TMR1IE == 1)
    {
        PIE1bits.TMR1IE = 0;
        state1 = (state1 <<1) | PORTAbits.RA1 | 0x0000;
        state2 = (state2 <<1) | PORTAbits.RA2 | 0x0000;
        state3 = (state3 <<1) | PORTAbits.RA3 | 0x0000;
        state4 = (state4 <<1) | PORTAbits.RA4 | 0x0000;
        state5 = (state5 <<1) | PORTCbits.RC2 | 0x0000;
        if(state1 == 0xf000)
        {
            if(controlState == RUN)
            {
                controlState = START_TIME;
            }
            else if(controlState != RUN)
            {
                controlState = RUN;
            }
            lcdIsWritten = 0;
        }
        if(state2 == 0xf000)
        {
            if(controlState == START_TIME)
            {
            controlState = END_TIME;                
            }
            else if (controlState == END_TIME)
            {
                controlState = OFFSET_TIME;
            }
            else if(controlState == OFFSET_TIME)
            {
                controlState = TWELVE_TWENTYFOUR;
            }
            else if(controlState == TWELVE_TWENTYFOUR)
            {
                controlState = RUN;
            }
            lcdIsWritten = 0;
        }
        if(state3 == 0xf000)
        {
            IncButton = 1;
        }
        else if(state4 == 0xf000)
        {
            DecButton = 1;
        }
        if(state5 == 0xf000)
        {
            Relay1 = !Relay1;
        }

        LATAbits.LA6 = !LATAbits.LA6;
        TMR1H = 0xff;
        TMR1L = 0xd0;
        
        
        PIE1bits.TMR1IE = 1;
        PIR1bits.TMR1IF = 0;
    }
    if(PIR1bits.RCIF == 1 && PIE1bits.RCIE == 1)
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
        if(data == '$' && rxState == NOT_VALID)
        {
            rxState = VALID_1;
            rxIndex = 0;
        }
        else if(data == 'G' && rxState == VALID_1)
        {
            rxState = VALID_2;
        }
        else if(data == 'P' && rxState == VALID_2)
        {
            rxState = VALID_3;
        }
        else if(data == 'R' && rxState == VALID_3)
        {
            rxState = VALID_4;
        }
        else if(data == 'M' && rxState == VALID_4)
        {
            rxState = VALID_5;
        }
        else if(data == 'C' && rxState == VALID_5)
        {
            rxState = VALID_6;
        }
        else if(rxState == VALID_6 && data != '\r')
        {
            rx_string[rxIndex] = data;
            rxIndex++;
        }        
        else if(data == '\r' && rxState == VALID_6)
        {
            rx_string[rxIndex] = '\0';
            rxState = COMPLETE;
        }
        PIR1bits.RCIF = 0;
        PIE1bits.RCIE = 1;
    } 
}



