#ifndef DELAY_LINE_H
#define DELAY_LINE_H

// #include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include <stdint.h>

typedef struct Delay_line struct_DL;


struct Delay_line
{
    int delaySize; // tableSize
    int step; // store current step of the delay line
    float *delay_lf_ptr; //pointer to array of float

};

// initialize Delay line.
void CLASS_DL_Delay_line_Ini(struct_DL *p);
// set the size of delay line.
void CLASS_DL_setParameter(struct_DL *p, int delaySize);
// put the signal in the delay line
void CLASS_DL_writeIn(struct_DL *p, float out);
/* this is debugging function to print out
the data in the delay line */
void CLASS_DL_printDelayLf(struct_DL *p, int indx);
// special modulation function
int CLASS_DL_mod(struct_DL *p, int x,int m);
// get data from delay line
float CLASS_DL_readFr(struct_DL *p, int delay);
// update the delay line
void CLASS_DL_step_one(struct_DL *p);
// free  before malloced Parameter
void CLASS_DL_free(struct_DL *p);

#endif //DELAY_LINE_H
