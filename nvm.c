/* 
 * File:   nvm.c
 * Author: Robert Donovan
 *
 * Created on June 25, 2023, 1:31 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "nvm.h"

#define chip_select LATCbits.LC0

#define WRSR   0x01
#define WRITE   0x02
#define READ    0x03
#define WRDI       0x04
#define RDSR    0x05
#define WREN     0x06

/*
 * 
 */
void InitNvm(void)
{
    chip_select = 1;
    SSPCON1bits.SSPEN = 0;
    SSPCON1bits.CKP = 0;
    SSPCON1bits.SSPM3 = 0;
    SSPCON1bits.SSPM2 = 0;
    SSPCON1bits.SSPM1 = 0;
    SSPCON1bits.SSPM0 = 0;
    SSPSTATbits.CKE = 1;
    SSPCON1bits.SSPEN = 1;
}

unsigned char ReadStatusReg()
{
    unsigned char i;
    chip_select = 0;
    WriteSpi(RDSR);
    i = WriteSpi(0);
    chip_select = 1;
    return i;
}

unsigned char WriteSpi(unsigned char  i)
{
    SSPBUF = i;
    while(!SSPSTATbits.BF);
    return SSPBUF;
}

unsigned int ReadSpi(char addressMsb, char addressLsb)
{
    char lsb, msb;
    unsigned int i = 0x0000;
    while(ReadStatusReg() & 0x03);
    chip_select = 0;
    WriteSpi(READ);
    WriteSpi(addressMsb);
    WriteSpi(addressLsb);
    msb = WriteSpi(0x00);
    lsb = WriteSpi(0x00);
    chip_select = 1;
   i = (unsigned int)msb;
   i = i <<8;
   i = i | (unsigned int)lsb;
   
   return i;
}

void WriteEnable(void)
{
    chip_select = 0;
    WriteSpi(WREN);
    chip_select = 1;
}

void WriteNvm(unsigned char addressMsb, unsigned char addressLsb, unsigned char dataMsb, unsigned char dataLsb)
{
    unsigned char x;
    while(ReadStatusReg() & 0x03);
    WriteEnable();
    chip_select = 0;
    x = WriteSpi(WRITE);
    WriteSpi(addressMsb);
    WriteSpi(addressLsb);
    WriteSpi(dataMsb);
    WriteSpi(dataLsb);
    chip_select = 1;
}

