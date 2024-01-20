#include <px4_platform_common/module.h>
//#include <px4_platform_common/module_params.h>

//#include <uORB/Publication.hpp>
//#include <uORB/Subscription.hpp>

#include "waypoints.hpp"


void reshape2D_into_1D(double in_2D[MAX_DIM][MAX_WPTS], double* out_1D, int size[2])
{
  int id = 0;
  for (int i = 0; i < size[0]; i++)
  {
    for (int ii = 0; ii < size[1]; ii++)
    {
        out_1D[id] = in_2D[i][ii];
        id++;
    }
  }
}

void reshape1D_into_2D(double* in_1D, double out_2D[MAX_DIM][MAX_WPTS], int size[2])
{
  int id = 0;
  for (int i = 0; i < size[0]; i++)
  {
    for (int ii = 0; ii < size[1]; ii++)
    {
        out_2D[i][ii] = in_1D[id];
        id++;
    }
  }
}

void printf_emxArray_real_T(emxArray_real_T* in)
{
  /*
    struct emxArray_real_T {
    double *data;
    int *size;
    int allocatedSize;
    int numDimensions;
    boolean_T canFreeData;
    };
  */
  for(int i = 0; i < in->allocatedSize; i++)
  {
    printf("data[%i] = %f\n", i, in->data[i]);
  }
  for(int i = 0; i < in->allocatedSize; i++)
  {
    printf("size[%i] = %i\n", i, in->size[i]);
  }

  printf("allocatedSize: %i\n", in->allocatedSize);
  printf("numDimensions: %i\n", in->numDimensions);
  printf("canFreeData: %i\n", in->canFreeData);
}

bool solve_trajectory_min_snap(double wpts_data_org_2D[MAX_DIM][MAX_WPTS], uint16_t size[2], double Tf, bool use_time_allocation, bool show_details,\
        double pp_out[MAX_WPTS-1][N_COEFFS][MAX_DIM], double* T_out, double* offset_out, uint16_t* N_dim_out, uint16_t* N_segments_out)
{
  bool all_good = false;
  if (Tf > 0.0)
  {
    if (size[0] > MAX_DIM || size[0] < 1 || size[1] > MAX_WPTS || size[1] < 1)
    {
      printf("ERROR: data size must be between [1][1] and [%i][%i]\n", MAX_DIM, MAX_WPTS);
    }
    else
    {
        emxArray_real_T *T = NULL;
        emxArray_real_T *n_dim_ids = NULL;
        emxArray_real_T *pp = NULL;
        double wpts_data[800];
        emxArray_real_T *offset = NULL;
        double Cost;
        double ExitFlag;
        double Iterations;
        double N_segments;
        double n_dim;


        int wpts_size[2] = {(int)size[0], (int)size[1]};
        reshape2D_into_1D(wpts_data_org_2D, wpts_data, wpts_size);


        emxInitArray_real_T(&T, 2);
        emxInitArray_real_T(&pp, 3);
        emxInitArray_real_T(&n_dim_ids, 1);
        emxInitArray_real_T(&offset, 1);


        //printf("Initilized, starting execution!\n");
        //uint64_t time_stamp = get_time_usec();

        solver_codegen(wpts_data, wpts_size, Tf,
                      use_time_allocation, show_details, T, pp,
                      &n_dim, n_dim_ids, &N_segments, offset, &Cost, &ExitFlag,
                      &Iterations);

        //printf("Time elapsed: %f s\n", get_dt_s(time_stamp));

        //printf("Results out:\n");
        *N_segments_out = (uint16_t)N_segments;
        *N_dim_out = (uint16_t)n_dim;

        //printf("Cost = %f, n_dim = %i, N_segments = %i\n", Cost, (int)*N_dim_out, (int)*N_segments_out);
        //printf("offset = [ ");
        for (int i = 0; i < MAX_DIM; i++)
        {
          offset_out[i] = offset->data[i];
          //printf("%f ", offset_out[i]);
        }
        //printf("]\n");
        //printf("T = [ ");
        for (int i = 0; i < N_segments+1; i++)
        {
          T_out[i] = T->data[i];
          //printf("%f ", T_out[i]);
        }
        //printf("]\n");
        //printf("n_dim_ids = [ ");
        //for (int i = 0; i < n_dim; i++) printf("%i ", (int) n_dim_ids->data[i]);
        //printf("]\n");
        for (int i_seg = 0; i_seg < N_segments; i_seg++)
        {
          for (int i_dim = 0; i_dim < n_dim; i_dim++)
          {
            //printf("pp[seg=%i,dim=%i] = [ ", i_seg, i_dim);
            for (int i_coefs = 0; i_coefs < 10; i_coefs++)
            {
              pp_out[i_seg][i_coefs][(int) n_dim_ids->data[i_dim] - 1] = pp->data[(i_dim + pp->size[2] * i_coefs) +\
                                              pp->size[2] * 10 * i_seg];
              //printf("%f ", pp_out[i_seg][i_coefs][i_dim]);


            }
            //printf("]\n");
          }
        }


        emxDestroyArray_real_T(T);
        emxDestroyArray_real_T(pp);
        emxDestroyArray_real_T(n_dim_ids);
        emxDestroyArray_real_T(offset);

        //printf("Done!\n");

        all_good = (int)ExitFlag == 0 || (int)ExitFlag == 2;
        return true;
    }
  }
  else
  {
    printf("ERROR: Total time must me positive\n");
  }

  return all_good;
}

void reset_wpts(double wpts[MAX_DIM][MAX_WPTS])
{
  memset(wpts, 0, sizeof(wpts[0][0])*MAX_DIM*MAX_WPTS);
}

void traj_line_x_1(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf)
{
  reset_wpts(wpts);

  static const uint16_t n_dim = 1;
  static const uint16_t n_wpts = 3;

  wpts[0][1] = 1.0;
  wpts[0][2] = 3.0;

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;

  *Tf = 5;
}

void traj_line_xy_1(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf)
{
  reset_wpts(wpts);

  static const uint16_t n_dim = 2;
  static const uint16_t n_wpts = 3;

  double tmp[2][3] = {
    {0,   1,   3},
    {0,   1,   3}
    };

  for (uint16_t i_dim = 0; i_dim < n_dim; i_dim++)
  {
    for (uint16_t i_wpt = 0; i_wpt < n_wpts; i_wpt++)
    {
      wpts[i_dim][i_wpt] = tmp[i_dim][i_wpt];
    }
  }

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;

  *Tf = 5;
}

void traj_line_xyz_1(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf)
{
  reset_wpts(wpts);

  static const uint16_t n_dim = 3;
  static const uint16_t n_wpts = 3;

  double tmp[3][3] = {
    {0,   1,   3},
    {0,   1,   3},
    {0,   1,   3}
    };

  for (uint16_t i_dim = 0; i_dim < n_dim; i_dim++)
  {
    for (uint16_t i_wpt = 0; i_wpt < n_wpts; i_wpt++)
    {
      wpts[i_dim][i_wpt] = tmp[i_dim][i_wpt];
    }
  }

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;

  *Tf = 5;
}

void traj_rrt_test_1(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf)
{
  reset_wpts(wpts);

  static const uint16_t n_dim = 4;
  static const uint16_t n_wpts = 5;

  double tmp[4][5] = {
    {5.4,   4.2,  2.8,  0.5,  0.4},
    {3,     1.6,  1.6,  1.8,  0.8},
    {-0.4,  -1.4, -1.4, -0.9, -0.4},
    {M_PI,  M_PI, M_PI, M_PI, M_PI}
    };

  for (uint16_t i_dim = 0; i_dim < n_dim; i_dim++)
  {
    for (uint16_t i_wpt = 0; i_wpt < n_wpts; i_wpt++)
    {
      wpts[i_dim][i_wpt] = tmp[i_dim][i_wpt];
    }
  }

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;

  *Tf = 10;
}

void traj_circle_1(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf)
{
  reset_wpts(wpts);

  static const uint16_t n_dim = 3;
  static const uint16_t n_wpts = 8;
  *Tf = 10;

  double tmp[3][8];

  for (uint16_t i_wpt = 1; i_wpt < n_wpts; i_wpt++)
  {
    tmp[0][i_wpt] = 2*sin(2*M_PI * (*Tf) / (n_wpts - 1));
    tmp[1][i_wpt] = 2*cos(2*M_PI * (*Tf) / (n_wpts - 1));
    tmp[2][i_wpt] = -1;
  }

  for (uint16_t i_dim = 0; i_dim < n_dim; i_dim++)
  {
    for (uint16_t i_wpt = 0; i_wpt < n_wpts; i_wpt++)
    {
      wpts[i_dim][i_wpt] = tmp[i_dim][i_wpt];
    }
  }

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;


}

void traj_circle_1_slow(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf)
{
  traj_circle_1(wpts, wpts_size, Tf);
  *Tf = 20;
}



void traj_square_1(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf)
{
  reset_wpts(wpts);

  static const uint16_t n_dim = 3;
  static const uint16_t n_wpts = 9;

  double tmp[3][9] = {
    {1,   0,     -1,     -1,     -1,     0,      1,   1,   1},
    {1,   1,      1,     0,      -1,     -1,     -1,  0,   1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1}
    };

  for (uint16_t i_dim = 0; i_dim < n_dim; i_dim++)
  {
    for (uint16_t i_wpt = 0; i_wpt < n_wpts; i_wpt++)
    {
      wpts[i_dim][i_wpt] = tmp[i_dim][i_wpt];
    }
  }

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;
  *Tf = 10;
}

void traj_8_1(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf)
{
  reset_wpts(wpts);

  static const uint16_t n_dim = 3;
  static const uint16_t n_wpts = 9;

  double tmp[4][9] = {
    {0,       0.5,    0.0,    -0.5,   0.0,    0.5,    0.0,    -0.5,   0.0},
    {0,       1.0,    2.0,    1.0,    0.0,    -1.0,   -2.0,   -1.0,   0.0},
    {0,       0.25,   0.5,    0.25,   0.0,    -0.25,  -0.5,   -0.25,  0.0}
  };

  for (uint16_t i_dim = 0; i_dim < n_dim; i_dim++)
  {
    for (uint16_t i_wpt = 0; i_wpt < n_wpts; i_wpt++)
    {
      wpts[i_dim][i_wpt] = tmp[i_dim][i_wpt];
    }
  }

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;
  *Tf = 15;
}

void traj_8_1_slow(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf)
{
  traj_8_1(wpts, wpts_size, Tf);
  *Tf = 20;
}

void traj_8_yaw_1(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf)
{
  reset_wpts(wpts);

  static const uint16_t n_dim = 4;
  static const uint16_t n_wpts = 9;

  double tmp[4][9] = {
    {0,       0.5,    0.0,    -0.5,   0.0,    0.5,    0.0,    -0.5,   0.0},
    {0,       1.0,    2.0,    1.0,    0.0,    -1.0,   -2.0,   -1.0,   0.0},
    {0,       0.25,   0.5,    0.25,   0.0,    -0.25,  -0.5,   -0.25,  0.0},
    {45,      90.0,   180.0,  270.0,  315.0,  270.0,  180.0,  90.0,   45.0}
  };

  for (uint16_t i_wpt = 0; i_wpt < n_wpts; i_wpt++)
  {
    tmp[3][i_wpt] = (tmp[3][i_wpt] - 45.0) * M_PI / 180.0;
  }

  for (uint16_t i_dim = 0; i_dim < n_dim; i_dim++)
  {
    for (uint16_t i_wpt = 0; i_wpt < n_wpts; i_wpt++)
    {
      wpts[i_dim][i_wpt] = tmp[i_dim][i_wpt];
    }
  }

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;
  *Tf = 15;
}

void traj_8_yaw_1_slow(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf)
{
  traj_square_1(wpts, wpts_size, Tf);
  *Tf = 20;
}

void traj_square_1_slow(double wpts[MAX_DIM][MAX_WPTS], uint16_t wpts_size[2], double* Tf)
{
  traj_8_yaw_1(wpts, wpts_size, Tf);
  *Tf = 20;
}

void test_solver_codegen(void)
{
  bool use_time_allocation = true;
  bool show_details = false;

  double wpts_data_org_2D[MAX_DIM][MAX_WPTS];

  uint16_t wpts_size[2];
  double Tf;

  double pp[MAX_WPTS-1][N_COEFFS][MAX_DIM];
  double T[MAX_WPTS];
  double offsets[MAX_DIM];
  uint16_t N_dim;
  uint16_t N_segments;

  //uint64_t time_stamp;
  int64_t time_stamp = hrt_absolute_time();

  traj_line_x_1(wpts_data_org_2D, wpts_size, &Tf);
  time_stamp = hrt_absolute_time();
  solve_trajectory_min_snap(wpts_data_org_2D, wpts_size, Tf, use_time_allocation, show_details,\
        pp, T, offsets, &N_dim, &N_segments);
  PX4_INFO("\nTime elapsed: %f s\n\n", static_cast<double>(hrt_absolute_time() - time_stamp) / 1000000.0);

  traj_line_xy_1(wpts_data_org_2D, wpts_size, &Tf);
  time_stamp = hrt_absolute_time();
  solve_trajectory_min_snap(wpts_data_org_2D, wpts_size, Tf, use_time_allocation, show_details,\
        pp, T, offsets, &N_dim, &N_segments);
  PX4_INFO("\nTime elapsed: %f s\n\n", static_cast<double>(hrt_absolute_time() - time_stamp) / 1000000.0);

  traj_rrt_test_1(wpts_data_org_2D, wpts_size, &Tf);
  time_stamp = hrt_absolute_time();
  solve_trajectory_min_snap(wpts_data_org_2D, wpts_size, Tf, use_time_allocation, show_details,\
        pp, T, offsets, &N_dim, &N_segments);
  PX4_INFO("\nTime elapsed: %f s\n\n", static_cast<double>(hrt_absolute_time() - time_stamp) / 1000000.0);

  traj_circle_1(wpts_data_org_2D, wpts_size, &Tf);
  time_stamp = hrt_absolute_time();
  solve_trajectory_min_snap(wpts_data_org_2D, wpts_size, Tf, use_time_allocation, show_details,\
        pp, T, offsets, &N_dim, &N_segments);
  PX4_INFO("\nTime elapsed: %f s\n\n", static_cast<double>(hrt_absolute_time() - time_stamp) / 1000000.0);

  traj_square_1(wpts_data_org_2D, wpts_size, &Tf);
  time_stamp = hrt_absolute_time();
  solve_trajectory_min_snap(wpts_data_org_2D, wpts_size, Tf, use_time_allocation, show_details,\
        pp, T, offsets, &N_dim, &N_segments);
  PX4_INFO("\nTime elapsed: %f s\n\n", static_cast<double>(hrt_absolute_time() - time_stamp) / 1000000.0);

  traj_8_yaw_1(wpts_data_org_2D, wpts_size, &Tf);
  time_stamp = hrt_absolute_time();
  solve_trajectory_min_snap(wpts_data_org_2D, wpts_size, Tf, use_time_allocation, show_details,\
        pp, T, offsets, &N_dim, &N_segments);
  PX4_INFO("\nTime elapsed: %f s\n\n", static_cast<double>(hrt_absolute_time() - time_stamp) / 1000000.0);
}
