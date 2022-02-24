#include "Basic_cpg_controller.h"


// define a basic CPG output class named "BasicCPG"
/* The constructor doesn't take any input parameter.
    the CPG consist of two neuron with weigth w12, w21
    w11 and w22, this give 2 periodic output signals */
void CLASS_BCC_Basic_cpg_controller_Ini(struct_BCC *p)
{
    // initial class attributes
    p->output1 =0;
    p->output2=0;
    p->bias1_2=0;
    p->w11_22=0;
    p->w12=0;
    p->w21=0;
    p->act1=0;
    p->act2=0;
    p->wd = 0.0;
}

// set parameter which define the properties of single CPG
// set the initial parameter of CPG controller
void CLASS_BCC_setParameter(struct_BCC *p, float output1, float output2, float bias1_2, float w11_22,float wd1,float MI)
 {

    p->output1 = output1;
    p->output2 = output2;
    p->bias1_2 = bias1_2;
    p->w11_22 = w11_22;
    p->w12 = wd1 + MI;
    p->w21 = -(wd1 + MI);
    p->wd = wd1;
}

// function to change MI for adapation
void CLASS_BCC_setMI(struct_BCC *p, float newMI)
{
    p->w12 = p->wd+newMI;
    p->w21 = -(p->wd+newMI);
}

// let CPG run for once
// function that updates the output of the CPG network
void CLASS_BCC_run(struct_BCC *p)
{    
    p->act1 = p->w11_22 * p->output1 + p->w12 * p->output2 + p->bias1_2;
    p->act2 = p->w11_22 * p->output2 + p->w21 * p->output1 + p->bias1_2;
    p->output1 =  tanh(p->act1);
    p->output2 =  tanh(p->act2);
}

// get the CPG output1/output2
/* function that return the output signal of the CPG
    network, 
            if number is 1 the output is signal 1
            if number is 2 the output is signal 2     */
float CLASS_BCC_getSignal(struct_BCC *p, int number)
{
    if(number == 1){
        return p->output1;
    }else{
        return p->output2;
    }
}



