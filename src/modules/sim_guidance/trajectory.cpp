#include <matrix/math.hpp>
#include "trajectory.hpp"


double get_dt_s_hrt(hrt_abstime &time_stamp)
{
	return static_cast<double>(hrt_elapsed_time(&time_stamp))*1.0E-6;
}

template <typename Type, size_t n_coefs>
Type poly_val(matrix::Vector<Type, n_coefs> coefs, Type time_int_s, Type tof_int_s, uint8_t deriv_order)
{
	Type out = static_cast<Type>(0.0);
	double tau = static_cast<double>(time_int_s / tof_int_s);
	Type scaling = static_cast<Type>(pow(1.0 / static_cast<double>(tof_int_s), static_cast<double>(deriv_order)));
	if (deriv_order == 0)
	{
		for (int i = 0; i < n_coefs; i++)
		{
			out += coefs(i)*static_cast<Type>(pow(tau,static_cast<double>(deriv_order)));
		}
	}
	else
	{
		for (int i = deriv_order; i < n_coefs; i++)
		{
			Type prod__ = static_cast<Type>(1.0);
			for (int ii = i - static_cast<int>(deriv_order) + 1; ii < i + 1; ii++)
			{
				prod__ *= static_cast<Type>(ii);
			}
			out += coefs(i)*prod__*static_cast<Type>(pow(tau,static_cast<double>(i - static_cast<int>(deriv_order))));
		}
	}
	return (out*scaling);
}

template <typename Type, size_t n_coefs, size_t n_dofs, size_t n_int>
int eval_traj(matrix::Vector<Type, n_dofs> &eval_vec, Type time_int_s, matrix::Vector<matrix::Vector<matrix::Vector<Type, n_coefs>, n_dofs>, n_int> coefs, matrix::Vector<Type, n_int> tof_int_s, uint8_t deriv_order)
{

	//locate segment number and tof for this segment:
	int i_int;
	Type TOF = static_cast<Type>(0.0);
	for (int i = 0; i < n_int; i++) TOF += tof_int_s(i);

	if (time_int_s < static_cast<Type>(0.0))
	{
		i_int = 0;
		time_int_s = static_cast<Type>(0.0);
	}
	else if (time_int_s >= TOF)
	{
		i_int = n_int-1;
		time_int_s = tof_int_s(i_int);
	}
	else
	{
		Type tof_sum = static_cast<Type>(0.0);
		i_int = 0;
		for (int i = 0; i < static_cast<int>(n_int); i++)
		{
			tof_sum += tof_int_s(i);
			if (time_int_s > tof_sum) i_int++;
			else break;
		}

		if (i_int > n_int - 1)
		{
			i_int = n_int - 1;
			time_int_s = tof_int_s(i_int);
		}
		else if (i_int > 0) time_int_s = time_int_s - tof_sum;
	}

	for (int i_dof = 0; i_dof < static_cast<int>(n_dofs); i_dof++)
	{
		eval_vec(i_dof) = poly_val<Type,n_coefs>((coefs(i_int))(i_dof), time_int_s, tof_int_s(i_int), deriv_order);
	}

	return 0;
}


int test_traj(void)
{
	const size_t n_coefs = 10;
	//const size_t n_segments = 2;
	double vec_test[n_coefs] = {0.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};
	matrix::Vector<double, n_coefs> x0_coefs;
	x0_coefs.setZero();
	x0_coefs.copyTo(vec_test);
	x0_coefs.print();

	return 0;
}




int read_from_file(void)
{
	static int _fd = -1;

	if (_fd < 0)
	{
		_fd = px4_open("/fs/microsd/1.txt", O_RDONLY);
		if (_fd < 0)
		{
			PX4_ERR("can't open file, %d", errno);
			return -1;
		}
	}

	PX4_INFO("Opened file!\n");

        char buf[100];
        int st = px4_read(_fd, buf, 100);
        if (st < 0)
	{
            PX4_ERR("can't read from %d, because %d", _fd, errno);
            return -1;
        }
	else
	{
            PX4_INFO("read %d bytes", st);
        }
	if (_fd > -1) px4_close(_fd);
	PX4_INFO("Done!\n");
        return 0;
}







void point::start(void)
{
	reset();
	timestamp = hrt_absolute_time();
	return;
}
void point::reset(void)
{
	pos.setZero();
	vel.setZero();
	acc.setZero();
	jerk.setZero();
	snap.setZero();
	yaw.setZero();
	return;
}


void trajectory::start(void)
{
	initial_pt.start();
	current_pt.start();
	return;
}
void trajectory::reset(void)
{
	initial_pt.reset();
	current_pt.reset();
	return;
}
