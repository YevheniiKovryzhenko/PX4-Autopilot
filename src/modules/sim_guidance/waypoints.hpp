#pragma once

#include <matrix/math.hpp>

// #include "tools.h"
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "rt_nonfinite.h"
#include "solver_codegen.h"
#include "solver_codegen_emxAPI.h"
#include "solver_codegen_types.h"

#define MAX_WPTS 200
#define MAX_DIM 4
#define N_COEFFS 10


bool solve_trajectory_min_snap(double wpts_data_org_2D[MAX_DIM][MAX_WPTS], uint16_t size[2], double Tf, bool use_time_allocation, bool show_details,\
        double pp_out[MAX_WPTS-1][N_COEFFS][MAX_DIM], double* T_out, double* offset_out, uint16_t* N_dim_out, uint16_t* N_segments_out);


void traj_line_x_1(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf);
void traj_line_xy_1(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf);
void traj_line_xyz_1(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf);
void traj_rrt_test_1(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf);
void traj_circle_1(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf);
void traj_circle_1_slow(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf);
void traj_square_1(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf);
void traj_8_1(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf);
void traj_8_1_slow(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf);
void traj_8_yaw_1(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf);
void traj_8_yaw_1_slow(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf);
void traj_square_1_slow(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf);
void test_solver_codegen(void);
