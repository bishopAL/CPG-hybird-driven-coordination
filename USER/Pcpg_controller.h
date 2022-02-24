#ifndef PCPG_CONTROLLER_H
#define PCPG_CONTROLLER_H

// #include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
//#include <stdint.h>

typedef struct Pcpg_controller struct_PC;


struct Pcpg_controller
{
    bool shift;
    float pcpg_out[2];
    float pcpg_step[2];
    float set[2];
    float setold[2];
    float diffset[2];
    float countup[2];
    float countupold[2];
    float countdown[2];
    float countdownold[2];
    float deltaxup[2];
    float deltaxdown[2];
    float xup[2];
    float xdown[2];
    float yup[2];
    float ydown[2];
    float pcpg_slope;
    float threshold;

    int zero_count[2];

};

// initialize post processing CPG network with default value.
void CLASS_PC_Pcpg_controller_Ini(struct_PC *p);

// set the parameter of PCPG controller (slope)
void CLASS_PC_setParameter(struct_PC *p, float slope,float threshold);

// pass the input signals through the PCPG network
void CLASS_PC_run(struct_PC *p, float signal1, float signal2);

/* this function return the output signal of the PCPG
network, 
        if number is 1 the output is signal 1
        if number is 2 the output is signal 2     */
float CLASS_PC_getSignal(struct_PC *p, int number);

#endif //PCPG_CONTROLLER_H
