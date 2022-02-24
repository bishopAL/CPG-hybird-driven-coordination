#include "Delay_line.h"


// initialize Delay line.
void CLASS_DL_Delay_line_Ini(struct_DL *p)
{
    // initial class attributes
    p->delaySize = 0;
    p->step = 0;
}

// set the size of delay line.
void CLASS_DL_setParameter(struct_DL *p, int delaySize)
{

    p->delaySize = delaySize;
    p->delay_lf_ptr = (float*)  malloc  (sizeof(float) * delaySize);
    for(int i=0;i<p->delaySize;i++)
    {
        p->delay_lf_ptr[i] = 0;
    }
}

// free  before malloced Parameter
void CLASS_DL_free(struct_DL *p)
{
    free(p->delay_lf_ptr);
}

// put the signal in the delay line
void CLASS_DL_writeIn(struct_DL *p, float out)
{
    p->delay_lf_ptr[p->step+1] = out;
}

/* p is debugging function to print out
    the data in the delay line */
void CLASS_DL_printDelayLf(struct_DL *p, int indx)
{
    //cout << p->delay_lf_ptr[indx] << '\t' << indx << endl;
}

// private method
    // special modulation function
int CLASS_DL_mod(struct_DL *p, int x,int m)
{
    int r = x%m;
    if(x<=0)
    {
        return (x+m);
    }else{
        return x;
    }
}

// get data from delay line
float CLASS_DL_readFr(struct_DL *p, int delay)
{
    float y = p->delay_lf_ptr[CLASS_DL_mod(p, p->step+1- delay, p->delaySize)];
    return y;
}

// update the delay line
void CLASS_DL_step_one(struct_DL *p)
{
    p->step += 1;
    if(((p->step+1) % (p->delaySize+1)) == 0){
        p->step = 0;
    }
}


