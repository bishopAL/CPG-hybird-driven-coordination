#ifndef BASIC_CPG_CONTROLLER_H
#define BASIC_CPG_CONTROLLER_H

// #include <iostream>
#include  <stdio.h>
#include  <stdlib.h>
#include  <math.h>
//#include <stdint.h>

typedef struct Basic_cpg_controller struct_BCC;


struct Basic_cpg_controller
{
    /* The constructor doesn't take any input parameter.
    the CPG consist of two neuron with weigth w12, w21
    w11 and w22, this give 2 periodic output signals */

    float output1;
    float output2;
    float bias1_2;
    float w11_22;
    float w12;
    float w21;
    float act1;
    float act2;
    float wd;
};

void CLASS_BCC_Basic_cpg_controller_Ini(struct_BCC *p);
// set the initial parameter of CPG controller
void CLASS_BCC_setParameter(struct_BCC *p, float output1, float output2, float bias1_2, float w11_22,float wd1,float MI);
// function to change MI for adapation
void CLASS_BCC_setMI(struct_BCC *p, float newMI);
// function that updates the output of the CPG network
void CLASS_BCC_run(struct_BCC *p);
/* function that return the output signal of the CPG
network, 
        if number is 1 the output is signal 1
        if number is 2 the output is signal 2     */
float CLASS_BCC_getSignal(struct_BCC *p, int number);

#endif //BASIC_CPG_CONTROLLER_H
