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

void SIM_CTRL_MOD::run()
{
	
	//_sm_full_state_pub.advertise();
	// initialize parameters
	parameters_update(true);	
	
	while (!should_exit()) {
		//vehicle_local_position_s local_pos;
		//vehicle_global_position_s global_pos;
		vehicle_attitude_s att;
		//airspeed_s air_sp;
		//battery_status_s batt;
		//distance_sensor_s dist;
		actuator_armed_s act_armed;
		vehicle_odometry_s odom;
		//adc_report_s adc;
		
		//obstacle_distance_s obs;
		
		//rc_channels_s rc_ch;
		//rc_parameter_map_s rc_map;
		
		//poll new data if available:
		bool need_2_pub = false;
		
		/*
		if (_obstacle_distance_sub.updated())
		{
			_obstacle_distance_sub.copy(&obs);

			for (int i = 0; i < 72; i++)
			{
				_sm_full_state.obs_distances[i] = obs.distances[i];
			}

			need_2_pub = true;
		}
		*/
		/*
		if (_adc_report_sub.updated())
		{
			_adc_report_sub.copy(&adc);

			_sm_full_state.adc_device_id = adc.device_id;
			_sm_full_state.adc_resolution = adc.resolution;
			_sm_full_state.adc_v_ref = adc.v_ref;
			for (int i = 0; i < 12; i++)
			{
				_sm_full_state.adc_channel_id[i] = adc.channel_id[i];
				_sm_full_state.adc_raw_data[i] = adc.raw_data[i];
			}

			need_2_pub = true;
		}
		*/
		/*
		if (_rc_channels_sub.updated())
		{
			_rc_channels_sub.copy(&rc_ch);


			_sm_full_state.rc_timestamp_last_valid = rc_ch.timestamp_last_valid;
			for (int i = 0; i < 18; i++)
			{
				_sm_full_state.rc_channels[i] = rc_ch.channels[i];
			}
			_sm_full_state.rc_channel_count = rc_ch.channel_count;
			for (int i = 0; i < 26; i++)
			{
				_sm_full_state.rc_function[i] = rc_ch.function[i];
			}
			_sm_full_state.rc_rssi = rc_ch.rssi;
			_sm_full_state.rc_signal_lost = rc_ch.signal_lost;
			_sm_full_state.rc_frame_drop_count = rc_ch.frame_drop_count;

			need_2_pub = true;
		}
		*/
		/*
		if (_vehicle_local_position_sub.updated())
		{
			_vehicle_local_position_sub.copy(&local_pos);

			_sm_full_state.veh_loc_x = local_pos.x;
			_sm_full_state.veh_loc_y = local_pos.y;
			_sm_full_state.veh_loc_z = local_pos.z;

			_sm_full_state.veh_loc_vx = local_pos.vx;
			_sm_full_state.veh_loc_vy = local_pos.vy;
			_sm_full_state.veh_loc_vz = local_pos.vz;

			_sm_full_state.veh_loc_z_deriv = local_pos.z_deriv;

			_sm_full_state.veh_loc_ax = local_pos.ax;
			_sm_full_state.veh_loc_ay = local_pos.ay;
			_sm_full_state.veh_loc_az = local_pos.az;

			_sm_full_state.veh_loc_heading = local_pos.heading;

			_sm_full_state.veh_loc_dist_bottom = local_pos.dist_bottom;
			_sm_full_state.veh_loc_dist_bottom_valid = local_pos.dist_bottom_valid;


			need_2_pub = true;
		}
		*/
		if (_vehicle_odometry_sub.updated())
		{
			_vehicle_odometry_sub.copy(&odom);
			for (int i = 0; i < 4; i++)
			{
				_sm_full_state.odo_q[i] = odom.q[i];
				_sm_full_state.odo_q_offset[i] = odom.q_offset[i];
			}

			_sm_full_state.odo_rollspeed = odom.rollspeed;
			_sm_full_state.odo_pitchspeed = odom.pitchspeed;
			_sm_full_state.odo_yawspeed = odom.yawspeed;

			need_2_pub = true;
		}
		/*
		if (_vehicle_global_position_sub.updated())
		{
			_vehicle_global_position_sub.copy(&global_pos);

			_sm_full_state.veh_glob_lat = global_pos.lat;
			_sm_full_state.veh_glob_lon = global_pos.lon;
			_sm_full_state.veh_glob_alt = global_pos.alt;
			_sm_full_state.veh_glob_alt_ellipsoid = global_pos.alt_ellipsoid;

			_sm_full_state.veh_glob_terrain_alt = global_pos.terrain_alt;
			_sm_full_state.veh_glob_terrain_alt_valid = global_pos.terrain_alt_valid;
			_sm_full_state.veh_glob_dead_reckoning = global_pos.dead_reckoning;

			need_2_pub = true;
		}
		*/
		if (_vehicle_attitude_sub.updated())
		{
			_vehicle_attitude_sub.copy(&att);

			for (int i = 0; i < 4; i++)
			{
				_sm_full_state.veh_att_q[i] = att.q[i];
			}

			need_2_pub = true;

		}
		/*
		if (_airspeed_sub.updated())
		{
			_airspeed_sub.copy(&air_sp);

			_sm_full_state.air_indicated_airspeed_m_s = air_sp.indicated_airspeed_m_s;
			_sm_full_state.air_true_airspeed_m_s = air_sp.true_airspeed_m_s;
			_sm_full_state.air_temperature_celsius = air_sp.air_temperature_celsius;
			_sm_full_state.air_confidence = air_sp.confidence;


			need_2_pub = true;
		}
		*/
		/*
		if (_battery_status_sub.updated())
		{
			_battery_status_sub.copy(&batt);

			_sm_full_state.bat_connected = batt.connected;
			_sm_full_state.bat_voltage_v = batt.voltage_v;
			_sm_full_state.bat_voltage_filtered_v = batt.voltage_filtered_v;
			_sm_full_state.bat_current_a = batt.current_a;
			_sm_full_state.bat_current_filtered_a = batt.current_filtered_a;
			_sm_full_state.bat_current_average_a = batt.current_average_a;
			_sm_full_state.bat_discharged_mah = batt.discharged_mah;
			_sm_full_state.bat_remaining = batt.remaining;
			_sm_full_state.bat_scale = batt.scale;
			_sm_full_state.bat_temperature = batt.temperature;
			_sm_full_state.bat_cell_count = batt.cell_count;

			need_2_pub = true;
		}
		*/
		/*
		if (_distance_sensor_sub.updated())
		{
			_distance_sensor_sub.copy(&dist);

			_sm_full_state.dist_current_distance = dist.current_distance;
			_sm_full_state.dist_signal_quality = dist.signal_quality;


			need_2_pub = true;
		}
		*/
		
		if (_actuator_armed_sub.updated())
		{
			_actuator_armed_sub.copy(&act_armed);

			_sm_full_state.armed = act_armed.armed;
			_sm_full_state.prearmed = act_armed.prearmed;
			_sm_full_state.ready_to_arm = act_armed.ready_to_arm;
			_sm_full_state.lockdown = act_armed.lockdown;
			_sm_full_state.manual_lockdown = act_armed.manual_lockdown;
			_sm_full_state.force_failsafe = act_armed.force_failsafe;
			_sm_full_state.in_esc_calibration_mode = act_armed.in_esc_calibration_mode;
			_sm_full_state.soft_stop = act_armed.soft_stop;

			need_2_pub = true;
		}
		

		//publish new data if needed:
		if (need_2_pub)
		{
			_sm_full_state_pub.publish(_sm_full_state);
		}
		

		parameters_update(); // update parameters

		px4_usleep(100);// don't update too frequenty
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
