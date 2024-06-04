#pragma once
#include <stdint.h>

int *traj_gen_linear_ramp(uint32_t step_num, int start_pos, int end_pos);

int *traj_gen_s_curve(uint32_t step_num, int start_pos, int end_pos, int start_vel, int end_vel);