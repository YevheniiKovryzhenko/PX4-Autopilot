/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "sim_guidance.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <math.h>
#include <uORB/topics/parameter_update.h>


int SIM_GUIDANCE::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int SIM_GUIDANCE::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	*/

	return print_usage("unknown command");
}


int SIM_GUIDANCE::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("SIM_GUIDANCE",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

SIM_GUIDANCE *SIM_GUIDANCE::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	SIM_GUIDANCE *instance = new SIM_GUIDANCE(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

SIM_GUIDANCE::SIM_GUIDANCE(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

#define DEBUG



void SIM_GUIDANCE::run()
{
	// initialize parameters
	parameters_update(true);

	_boot_timestamp = hrt_absolute_time();
	while (!should_exit()) {
		parameters_update(); // update parameters
		update_guidance(); //update everything related to simulink

		px4_usleep(1000);// don't update too frequenty
	}
}

template <typename Type, size_t M>
void assign_1Darray2Vector(matrix::Vector<Type, M> *output_Vec, Type input_1Darray[M])
{
	for (int i = 0; i < M; i++) output_Vec(i) = input_1Darray[i];
	return;
}

int test_traj(void)
{
	const size_t n_coefs = 10;
	const size_t n_segments = 2;
	const size_t n_dofs = 3;
	double x0_coefs_raw[n_coefs] = {10.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};
	double y0_coefs_raw[n_coefs] = {20.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};
	double z0_coefs_raw[n_coefs] = {30.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};
	double yaw0_coefs_raw[n_coefs] = {20.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};

	double x1_coefs_raw[n_coefs] = {20.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};
	double y1_coefs_raw[n_coefs] = {20.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};
	double z1_coefs_raw[n_coefs] = {20.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};
	double yaw1_coefs_raw[n_coefs] = {20.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};

	matrix::Vector<double, n_coefs> x0_coefs(x0_coefs_raw);
	matrix::Vector<double, n_coefs> y0_coefs(y0_coefs_raw);
	matrix::Vector<double, n_coefs> z0_coefs(z0_coefs_raw);
	matrix::Vector<double, n_coefs> yaw0_coefs(yaw0_coefs_raw);

	matrix::Vector<double, n_coefs> x1_coefs(x1_coefs_raw);
	matrix::Vector<double, n_coefs> y1_coefs(y1_coefs_raw);
	matrix::Vector<double, n_coefs> z1_coefs(z1_coefs_raw);
	matrix::Vector<double, n_coefs> yaw1_coefs(yaw1_coefs_raw);

	matrix::Vector<matrix::Vector<matrix::Vector<double, n_coefs>, n_dofs>, n_segments> coefs;
	coefs.setZero();
	coefs(0)(0) = x0_coefs;
	coefs(1)(0) = y0_coefs;
	coefs(2)(0) = z0_coefs;
	coefs(3)(0) = yaw0_coefs;

	coefs(0)(1) = x1_coefs;
	coefs(1)(1) = y1_coefs;
	coefs(2)(1) = z1_coefs;
	coefs(3)(1) = yaw1_coefs;

	//coefs(2)(0).print();
	//coefs(3)(1).print();

	//matrix::Vector<matrix::Vector<double, n_coefs>,n_dofs> coefs_int;
	//coefs_int.setZero();
	//coefs_int(0) = x0_coefs;
	//coefs_int(1) = y0_coefs;
	//coefs_int(0).print();
	//coefs_int(1).print();
	printf("\n-----------\n");
	return 0;
}

void SIM_GUIDANCE::update_guidance(void)
{
	//int32_t enable_fl = _param_smg_en.get();

	//if (enable_fl == 1)
	//{
		#ifdef DEBUG
		printf("Updating main loop\n");
		test_traj();
		printf("\n");
		#endif
	//}

}



void SIM_GUIDANCE::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int SIM_GUIDANCE::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided SIM_GUIDANCE module functionality.

This is a template for a SIM_GUIDANCE module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this SIM_GUIDANCE module.

### Examples
CLI usage example:
$ SIM_GUIDANCE start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("SIM_GUIDANCE", "custom");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int sim_guidance_main(int argc, char *argv[])
{
	return SIM_GUIDANCE::main(argc, argv);
}
