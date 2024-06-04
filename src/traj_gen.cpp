#include "traj_gen.h"
#include <stdlib.h>



// return a malloc array of size step_num with the trajectory. note the caller is responsible for freeing the memory
int *traj_gen_linear_ramp(uint32_t step_num, int start_pos, int end_pos)
{
    int *traj = (int *)malloc(step_num * sizeof(int));
    if (traj == NULL)
    {
        return NULL;
    }

    int delta = end_pos - start_pos;
    for (uint32_t i = 0; i < step_num; i++)
    {
        traj[i] = start_pos + (delta * i) / step_num;
    }



    return traj;
}

int *traj_gen_s_curve(uint32_t step_num, int start_pos, int end_pos, int start_vel, int end_vel)
{
    int *traj = (int *)malloc(step_num * sizeof(int));
    if (traj == NULL)
    {
        return NULL;
    }

    // implement the s-curve trajectory here

    return traj;
}