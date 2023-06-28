

/* 
 * File:   poolTestMain.h
 * Author: Robert Donovan
 * Comments:    6/24/2023
/ * Revision history: 1.0
/ */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef POOLTESTMAIN_TEMPLATE_H
#define	POOLTESTMAIN_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  


typedef enum controlState
{
           
         RUN,
         START_TIME,
         END_TIME,
         OFFSET_TIME
}controlState_t;

typedef enum rxDataState
{
        NOT_VALID,
        VALID_1,
        VALID_2,
        VALID_3,
        VALID_4,
        VALID_5,
        VALID_6,
        COMPLETE
}rxDataState;

typedef struct time
{
    unsigned char hour_msb;
    unsigned char hour_lsb;
    unsigned char min_msb;
    unsigned char min_lsb;
}time;


// TODO Insert C++ class definitions if appropriate

void InitT1(void);
void initUart(void);
unsigned int convertTimeToNumber(unsigned char hour_msb, unsigned char hour_lsb, unsigned char min_msb, unsigned char min_lsb);
time convertNumberToTime(unsigned int time_num);
unsigned int applyOffset(unsigned int time);


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* POOLTESTMAIN_TEMPLATE_H */

