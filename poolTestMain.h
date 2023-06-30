/* 
 * File:   poolTestMain.h
 * Author: Robert Donovan
 * Comments:    6/24/2023
/ * Revision history: 1.0
/ */

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

typedef struct time_t
{
    unsigned char hour_msb;
    unsigned char hour_lsb;
    unsigned char min_msb;
    unsigned char min_lsb;
    char a_or_p;
    char mmm;
}time_t;

typedef struct timeNumber_t
{
    unsigned int time;
    unsigned char am_pn;
}timeNumber_t;



void InitT1(void);
void initUart(void);
unsigned int convertTimeToNumber(unsigned char hour_msb, unsigned char hour_lsb, unsigned char min_msb, unsigned char min_lsb);
time_t convertNumberToTime(timeNumber_t time_num);
timeNumber_t applyOffset(unsigned int time_num);
void makeTimeTwelveHour(timeNumber_t *time_num);


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */


#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* POOLTESTMAIN_TEMPLATE_H */

