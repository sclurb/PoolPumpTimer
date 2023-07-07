#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "LCDpic18.h"
#define _XTAL_FREQ 8000000
//#define __delay_ms(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000.0)))
// set up the timing for the LCD delays
#define LCD_delay 750 // ~5mS
#define LCD_Startup 15 // ~15mS

// Command set for Hitachi 44780U LCD display controller
#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_CURSOR_BACK 0x10
#define LCD_CURSOR_FWD 0x14
#define LCD_PAN_LEFT 0x18
#define LCD_PAN_RIGHT 0x1C
#define LCD_CURSOR_OFF 0x0C
#define LCD_CURSOR_ON 0x0E
#define LCD_CURSOR_BLINK 0x0F
#define LCD_CURSOR_LINE2 0xC0

// display controller setup commands from page 46 of Hitachi datasheet
#define FUNCTION_SET 0x28 // 4 bit interface, 2 lines, 5x8 font
#define ENTRY_MODE 0x06 // increment mode
#define DISPLAY_SETUP 0x0C // display on, cursor off, blink offd

#define LCDLine1() LCDPutCmd(LCD_HOME) // legacy support
#define LCDLine2() LCDPutCmd(LCD_CURSOR_LINE2) // legacy support
#define shift_cursor() LCDPutCmd(LCD_CURSOR_FWD) // legacy support
#define cursor_on() LCDPutCmd(LCD_CURSOR_ON) // legacy support
#define DisplayClr() LCDPutCmd(LCD_CLEAR) // Legacy support

// single bit for selecting command register or data register
#define instr 0
#define data 1

// These #defines create the pin connections to the LCD in case they are changed on a future demo board
#define LCD_PORT PORTD
#define LCD_PWR PORTDbits.RD7 // LCD power pin
#define LCD_EN PORTDbits.RD6 // LCD enable
#define LCD_RW PORTDbits.RD5 // LCD read/write line
#define LCD_RS PORTDbits.RD4 // LCD register select line

#define NB_LINES 2 // Number of display lines
#define NB_COL 16 // Number of characters per line

void LCD_Initialize()
{
    LCD_PORT = 0;               // clear latches before enabling TRIS bits
    TRISD = 0x00;
    LCD_PWR = 1;                // power up the LCD
    __delay_ms(LCD_Startup);    // required by display controller to allow power to stabilize
    LCDPutCmd(0x32);            // required by display initialization
    LCDPutCmd(FUNCTION_SET);    // set interface size, # of lines and font
    LCDPutCmd(DISPLAY_SETUP);   // turn on display and sets up cursor
    DisplayClr();
    LCDPutCmd(ENTRY_MODE);      // set cursor movement direction
}

void LCDWriteNibble(char ch, char rs)
{
    ch = (ch >> 4);             // always send the upper nibble
    ch = (ch & 0x0F);           // mask off the nibble to be transmitted
    LCD_PORT = (LCD_PORT & 0xF0);  // clear the lower half of LCD_PORT
    LCD_PORT = (LCD_PORT | ch); // move the nibble onto LCD_PORT
    LCD_RS = rs;                // set data/instr bit to 0 = insructions; 1 = data
    LCD_RW = 0;                 // RW - set write mode
    LCD_EN = 1;                 // set up enable before writing nibble
    LCD_EN = 0;                 // turn off enable after write of nibble
}

void LCDPutChar(char ch)
{
    __delay_us(LCD_delay);
    LCDWriteNibble(ch,data);    //Send higher nibble first
    ch = (ch << 4);             //get the lower nibble
    LCDWriteNibble(ch,data);    // Now send the low nibble
}


void LCDPutCmd(char ch)
{
    __delay_us(LCD_delay);
    LCDWriteNibble(ch,instr);   //Send the higher nibble
    ch = (ch << 4);             //get the lower nibble
    __delay_us(LCD_delay);
    LCDWriteNibble(ch,instr);  //Now send the lower nibble
}

void LCDPutStr(const char *str)
{
    char i=0;
    while (str[i])              // While string has not been fully traversed
    {
        LCDPutChar(str[i++]);   // Go display current char
    }
}

void LCDGoto(char pos,char ln)
{
    if ((ln > (NB_LINES-1)) || (pos > (NB_COL-1)))  // if incorrect line or column
    {
        return;                 // Just do nothing
    }
    LCDPutCmd((ln == 1) ? (0xC0 | pos) : (0x80 | pos));  // LCD_Goto command
    __delay_us(LCD_delay);      // Wait for the LCD to finish
}
/**
End of File
*/