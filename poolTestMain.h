

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

// TODO Insert appropriate #include <>

typedef enum controlState
{
           
         RUN,
         TOP_MENU,
         START_TIME,
         END_TIME
}controlState_t;


// TODO Insert C++ class definitions if appropriate

void InitT1(void);
void initUart(void);


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* POOLTESTMAIN_TEMPLATE_H */

