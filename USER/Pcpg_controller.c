#include "Pcpg_controller.h"


// initialize post processing CPG network with default value.
void CLASS_PC_Pcpg_controller_Ini(struct_PC *p)
{
    // initial class attributes
    p->shift = false;
    for(int i=0; i<2; i++)
    {
        p->pcpg_out[i]=0;
        p->pcpg_step[i]=0;
        p->set[i]=0;
        p->setold[i]=0;
        p->diffset[i]=0;
        p->countup[i]=0;
        p->countupold[i]=0;
        p->countdown[i]=0;
        p->countdownold[i]=0;
        p->deltaxup[i]=0;
        p->deltaxdown[i]=0;
        p->xup[i]=0;
        p->xdown[i]=0;
        p->yup[i]=0;
        p->ydown[i]=0;
        p->zero_count[i] = 0;
    }
    p->pcpg_slope = 2;
    p->threshold = 0.87;

    
}

// set the parameter of PCPG controller (slope)
void CLASS_PC_setParameter(struct_PC *p, float slope,float threshold)
{
    p->pcpg_slope = slope;
    p->threshold = threshold;
}

// pass the input signals through the PCPG network
void CLASS_PC_run(struct_PC *p, float signal1, float signal2)
{
    p->pcpg_step[0] = signal1;
    p->pcpg_step[1] = signal2;
        
    p->setold[0] = p->set[0];
    p->setold[1] = p->set[1];
        
    p->countupold[0] = p->countup[0];
    p->countupold[1] = p->countup[1];

    p->countdownold[0] = p->countdown[0];
    p->countdownold[1] = p->countdown[1];

    
    if (p->pcpg_step[1] >=  p->threshold) {p->set[1] = 1.0;}
    if (p->pcpg_step[1] < p->threshold) {p->set[1] = -1.0;}
    

    p->diffset[0] = p->set[0] - p->setold[0]; 
    p->diffset[1] = p->set[1] - p->setold[1];

    ///count how many step of swing
    if (p->set[0] == 1.0) {
        p->countup[0] = p->countup[0] + 1.0;
        p->countdown[0] = 0.0;
    } 
    if (p->set[0] == -1.0){
        p->countdown[0] = p->countdown[0] + 1.0;
        p->countup[0] = 0.0;
    }
        
    if (p->set[1] == 1.0) {
        p->countup[1] = p->countup[1] + 1.0;
        p->countdown[1] = 0.0;
    }
    if (p->set[1] == -1.0) {
        p->countdown[1] = p->countdown[1] + 1.0;
        p->countup[1] = 0.0;
    }
   
    // memorized the total steps of swing and stance

    if (p->countup[0] == 0.0  &&  p->diffset[0] == -2.0  &&  p->set[0] == -1.0)  {
        p->deltaxup[0] = p->countupold[0];
    }
        
    if (p->countdown[0] == 0.0 && p->diffset[0] == 2.0 && p->set[0] == 1.0) {
        p->deltaxdown[0] = p->countdownold[0];
    }
        
    if (p->countup[1] == 0.0 && p->diffset[1] == -2.0 && p->set[1] == -1.0) {
        p->deltaxup[1] = p->countupold[1];
    }
        
    if (p->countdown[1] == 0.0 && p->diffset[1] == 2.0 && p->set[1] == 1.0) {
        p->deltaxdown[1] = p->countdownold[1];
    }

        
    // comput y up and down
    p->xup[0] = p->countup[0];
    p->xdown[0] = p->countdown[0];
        
    p->xup[1] = p->countup[1];
    p->xdown[1] = p->countdown[1];
        
    p->yup[0] = ((2./p->deltaxup[0])*p->xup[0]) - 1;
    p->ydown[0] = ((-1./p->deltaxdown[0])*p->xdown[0]) + 1;
        
    p->yup[1] = ((2./p->deltaxup[1])*p->xup[1]) - 1;
    
    /*if(p->shift){
        p->yup[1]=((4/p->deltaxup[1])*p->xup[1]);
    }*/
    p->ydown[1] = ((-p->pcpg_slope/p->deltaxdown[1])*p->xdown[1]) + 1;

    // combine y up and down
    if (p->set[0] >= 0.0) {
        p->pcpg_out[0] = p->yup[0];
    }
        
    if (p->set[0] < 0.0) {
        p->pcpg_out[0] = p->ydown[0];
    }
        
    if (p->set[1] >= 0.0) {
        p->pcpg_out[1] = p->yup[1];
    }
        
    if (p->set[1] < 0.0) {
        p->pcpg_out[1] = p->ydown[1];
    }

    // limit upper and lower boundary

    if (p->pcpg_out[0] > 1.0) {
        p->pcpg_out[0] = 1.0;
    }
        
    if (p->pcpg_out[0] < -1.0) {
        p->pcpg_out[0] = -1.0;
    }

    if (p->pcpg_out[1] > 1.0) {
        p->pcpg_out[1] = 1.0;
    }
        
    if (p->pcpg_out[1] < -1.0) {
        p->pcpg_out[1] = -1.0;
    }

}
    

/* this function return the output signal of the PCPG
    network, 
            if number is 1 the output is signal 1
            if number is 2 the output is signal 2     */
float CLASS_PC_getSignal(struct_PC *p, int number)
{
    if(number == 1)
    {
        return p->pcpg_out[0];
    }else{
        return p->pcpg_out[1];
    }
}
