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

#include "sim_ctrl_mod.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>

sim_data_trafic::sim_data_trafic()
{
	ind = 0;
	for (uint i = 0; i < MAX_SIZE; i++)
	{
		data[i] = 0.f;
	}
}

sim_data_trafic::~sim_data_trafic()
{
}

void sim_data_trafic::send_vec(float out_vec[MAX_SIZE])
{
	for (uint i = 0; i < MAX_SIZE; i++) out_vec[i] = data[i];
	clear_buffer();

}

char sim_data_trafic::fill_buffer(float* in, uint size)
{
	for (uint i = 0; i < size; i++)
	{
		if (fill_buffer(in[i]) < 0) return -1;
	}
	return 0;
}

char sim_data_trafic::fill_buffer(float in)
{
	if (ind < MAX_SIZE) {
		data[ind] = in;
		ind++;
		return 0;
	}
	else
	{
		clear_buffer();
#ifdef DEBUG
PX4_INFO("ERROR: ran out of space in buffer, check data size\n");
#endif
		return -1;
	}

}
void sim_data_trafic::clear_buffer(void)
{
	for (uint i = 0; i < MAX_SIZE; i++) data[i] = 0.f;
	ind = 0;

}

int SIM_CTRL_MOD::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int SIM_CTRL_MOD::custom_command(int argc, char *argv[])
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


int SIM_CTRL_MOD::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("sim_ctrl_mod",
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

SIM_CTRL_MOD *SIM_CTRL_MOD::instantiate(int argc, char *argv[])
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

	SIM_CTRL_MOD *instance = new SIM_CTRL_MOD(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

SIM_CTRL_MOD::SIM_CTRL_MOD(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

#define DEBUG

void SIM_CTRL_MOD::run()
{
	// initialize parameters
	parameters_update(true);


	while (!should_exit()) {

#ifdef DEBUG
//PX4_INFO("run: start main loop\n");
#endif
		if (_param_en_hil.get() == 0) publish_inbound_sim_data(); //if HIL/SITL is enabled, assume this data was already published

		parameters_update(); // update parameters

		px4_usleep(1000);// don't update too frequenty
	}
}

bool SIM_CTRL_MOD::check_ground_contact(void) // this is a quick work-around the weird land-detector logic
{
	int32_t gc_case = _param_gc_opt.get();


	static int64_t time_pressed_gc = hrt_absolute_time();
	static bool was_pressed = false;

	static bool old_value = static_cast<bool>(_param_gc_set.get());


	switch (gc_case)
	{
		case 1:
		{
			int32_t use_lidar = 0;
			param_get(param_find("SENS_EN_SF1XX"), &use_lidar);
			if (use_lidar > 6) {
				distance_sensor_s dist;
				if (_distance_sensor_sub.update(&dist))
				{
					float ekf2_min_rng = 0.f;
					param_get(param_find("EKF2_MIN_RNG"), &ekf2_min_rng);
					old_value = dist.current_distance < ekf2_min_rng;
				}
			}
			break;
		}
		case 2:
		{
			adc_report_s adc;
			if (_adc_report_sub.update(&adc))
			{
				if (adc.resolution == 0) break;

				static const int ADC_GC_INDEX = 4;
				float val = static_cast<float>(adc.raw_data[ADC_GC_INDEX]) * static_cast<float>(adc.raw_data[ADC_GC_INDEX]) / static_cast<float> (adc.resolution);

				if(val < 1.f)
				{
					if (!was_pressed)
					{
						time_pressed_gc = hrt_absolute_time();
						old_value = false;
					}
					else
					{
						float landing_trig_time = 0.f;
						param_get(param_find("LNDMC_TRIG_TIME"), &landing_trig_time);

						float dt_sec = static_cast<float>(hrt_absolute_time() - time_pressed_gc) / 1000000.f;
						old_value = dt_sec > landing_trig_time;
					}

				}
			}
			break;
		}
		default:
			old_value = static_cast<bool>(_param_gc_set.get());
			break;
	}

	was_pressed = false;
	return old_value;

}

bool SIM_CTRL_MOD::check_armed(void)
{
	static bool armed = false;

	if (_actuator_armed_sub.update(&act_armed))
	{
		if (act_armed.armed)
		{
			if (!armed)
			{
				armed = true;
				return false; //let's confirm it on the next run
			}
		}
		else
		{
			armed = false;
		}
	}
	return armed;
}

void
SIM_CTRL_MOD::publish_inbound_sim_data(void)
{
	//poll new data if available:
	bool need_2_pub = false;

	if (_vehicle_local_position_sub.update(&local_pos)) need_2_pub = true;
	if (_vehicle_odometry_sub.update(&odom)) need_2_pub = true;
	if (_vehicle_global_position_sub.update(&global_pos)) need_2_pub = true;
	if (_vehicle_attitude_sub.update(&att)) need_2_pub = true;



	//publish new data if needed:
	if (need_2_pub)
	{
		simulink_inboud_data.fill_buffer(local_pos.vx);
		simulink_inboud_data.fill_buffer(local_pos.vy);
		simulink_inboud_data.fill_buffer(local_pos.vz);

		simulink_inboud_data.fill_buffer(odom.rollspeed);
		simulink_inboud_data.fill_buffer(odom.pitchspeed);
		simulink_inboud_data.fill_buffer(odom.yawspeed);

		simulink_inboud_data.fill_buffer(odom.q, 4);

		simulink_inboud_data.fill_buffer(global_pos.lat);
		simulink_inboud_data.fill_buffer(global_pos.lon);
		simulink_inboud_data.fill_buffer(global_pos.alt);
		simulink_inboud_data.fill_buffer(global_pos.terrain_alt);

		simulink_inboud_data.fill_buffer(local_pos.ax);
		simulink_inboud_data.fill_buffer(local_pos.ay);
		simulink_inboud_data.fill_buffer(local_pos.az);

		simulink_inboud_data.fill_buffer(local_pos.x);
		simulink_inboud_data.fill_buffer(local_pos.y);
		simulink_inboud_data.fill_buffer(local_pos.z);

		simulink_inboud_data.fill_buffer(local_pos.z_deriv);

		simulink_inboud_data.fill_buffer(static_cast<float> (check_ground_contact()));
		simulink_inboud_data.fill_buffer(static_cast<float> (check_armed()));

		debug_array_s debug_topic{};

		debug_topic.timestamp = hrt_absolute_time();
		debug_topic.id = debug_array_s::SIMULINK_INBOUND_ID;
		char message_name[10] = "inbound";
		memcpy(debug_topic.name, message_name, sizeof(message_name));
		debug_topic.name[sizeof(debug_topic.name) - 1] = '\0'; // enforce null termination

		simulink_inboud_data.send_vec(debug_topic.data);
		_simulink_inbound_pub.publish(debug_topic);
	}

}


void SIM_CTRL_MOD::parameters_update(bool force)
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

int SIM_CTRL_MOD::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided sim_ctrl_mod module functionality.

This is a template for a sim_ctrl_mod module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this sim_ctrl_mod module.

### Examples
CLI usage example:
$ sim_ctrl_mod start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sim_ctrl_mod", "custom");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int sim_ctrl_mod_main(int argc, char *argv[])
{
	return SIM_CTRL_MOD::main(argc, argv);
}
