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

#include <math.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_rc_input.h>

inline char rc_map_stick(float &out, rc_channels_s& rc_ch, uint8_t ch)
{
	if (ch > rc_channels_s::FUNCTION_MAN) {
		out = 0.f;
		return -1;
		}

	uint8_t ind = rc_ch.function[ch];
	if (static_cast<size_t>(ind) > sizeof(rc_ch.channels)) {
		out = 0.f;
		return -1;
	}
	else {
		out = rc_ch.channels[ind];
		return 0;
	}
}

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
		fill_buffer(in[i]);
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

//#define DEBUG

void SIM_CTRL_MOD::debug_loop(void)
{
	//actuator_outputs_s act_out{};
	//if (_simulink_inbound_sub.update(&sm_inbound)) printf_debug_array(sm_inbound);
	//if (_actuator_outputs_sv_sub.update(&act_out)) printf_actuator_output(act_out);
	/*
	if (_simulink_outbound_sub.updated())
	{
		PX4_INFO("received outbound data\n");
	}
	*/

	//test_fake_atuator_data();
}

void SIM_CTRL_MOD::run()
{
	// initialize parameters
	parameters_update(true);

	_boot_timestamp = hrt_absolute_time();
	while (!should_exit()) {
		parameters_update(); // update parameters
		update_simulink_io(); //update everything related to simulink

		px4_usleep(1000);// don't update too frequenty
	}
}

void SIM_CTRL_MOD::update_simulink_io(void)
{
	update_simulink_inputs();

	update_simulink_outputs();

	#ifdef DEBUG
	debug_loop();
	#endif
}

void SIM_CTRL_MOD::update_simulink_inputs(void)
{
	/*
	float SM_IDLE_TH_val = 0;
	param_get(param_find("SM_MASS"),&SM_IDLE_TH_val);
	printf("updating main loop:\t");
	printf("SM_MASS = %f\n",(double)SM_IDLE_TH_val);
	*/

	// we will later do some extra parsing and safety in here before publishing any data
	publish_inbound_sim_data();

}

void SIM_CTRL_MOD::update_simulink_outputs(void)
{
	bool need2update_actuators = false;
	//if simulink has published new outputs, we have to grab those and distribute to proper places in PX4
	if (_simulink_outbound_sub.update(&sm_outbound))
	{
		act_output.timestamp = hrt_absolute_time();
		act_output.noutputs = ACTUATOR_MAX_SIZE;
		for(int i = 0; i < ACTUATOR_MAX_SIZE; i++ )
		{
			act_output.output[i] = sm_outbound.data[ACTUATOR_START_IND + i];
		}

		need2update_actuators = true;
	}

	//we can do some safety checking here first, before publishing the values

	if(need2update_actuators) _actuator_outputs_sv_pub.publish(act_output); //the rest of PX4 needs to know new actuator commands
}

bool SIM_CTRL_MOD::update_man_wing_angle(int input_source_opt, float& wing_cmd)
{
	int32_t wing_opt = _param_sm_wing_src.get();
	bool need2update = false;
	static float cmd = -1.f;

	switch (wing_opt)
	{
	case 1: //always 1
		wing_cmd = 1.f;
		break;

	case 2: //MANUAL use CMD_SRC
		switch (input_source_opt)
			{
			case 1: // RC_IN
			{
				if(_rc_channels_sub.update(&rc_ch)) need2update = true;

				//check every stick so we are sure rc is valid, otherwise quit
				rc_map_stick(wing_cmd, rc_ch, rc_channels_s::FUNCTION_AUX_6);
				break;
			}


			case 2: //INBOUND_MSG
				if (_simulink_inbound_sub.update(&sm_inbound)) need2update = true;
				wing_cmd = sm_inbound.data[CONTROL_VEC_START_ID + AUX6_IND];
				break;//assume everything was already sent correctly


			default: //MANUAL_CONTROL_SETPOINT
				if (_manual_control_setpoint_sub.update(&man_setpoint)) need2update = true;
				wing_cmd = man_setpoint.aux6;
				break;
			}
		return false;
	case 3: //AUTO
		wing_cmd = sm_inbound.data[CONTROL_VEC_START_ID + AUX6_IND]; //simulink will ignore it anyways
		break;
	default: //always -1
		wing_cmd = -1.f;
		break;
	}
	need2update = (need2update || fabsf(wing_cmd - cmd) > 1.f / 1000000.f);
	cmd = wing_cmd; //keep track of the old command
	return need2update;
}

bool SIM_CTRL_MOD::check_ground_contact(void) // this is a quick work-around the weird land-detector logic
{
	int32_t gc_case = _param_gc_opt.get();


	static int64_t time_pressed_gc = hrt_absolute_time();
	static bool was_pressed = false;

	static bool old_value = false;


	switch (gc_case)
	{
		case 1:
			old_value = true;
			break;
		case 2:
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
		case 3:
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
			old_value = false;
			break;
	}

	was_pressed = false;
	return old_value;

}

bool SIM_CTRL_MOD::check_armed(bool &armed, int input_src_opt)
{
	actuator_armed_s act_armed_px4;
	bool commander_updated_armed_state = _actuator_armed_sub.update(&act_armed_px4);

	//if (commander_updated_armed_state) printf("%4d, %4d, %4d\n", act_armed_px4.armed, act_armed_px4.prearmed, act_armed_px4.ready_to_arm);

	int32_t arm_src_opt = _param_sm_overwrite.get();

	if ((hrt_elapsed_time(&_boot_timestamp) > 5000000))
	{
		switch (arm_src_opt)
		{
		case 1:
			armed = true;
			break;

		case 2:
			armed = false;
			break;

		default:
			armed = act_armed.armed;
			switch (input_src_opt)
			{
			case 1: //RC_IN
			{
				if(_rc_channels_sub.update(&rc_ch))
				{
					float tmp_armed = static_cast<float>(armed);
					rc_map_stick(tmp_armed, rc_ch, rc_channels_s::FUNCTION_ARMSWITCH);
					armed = tmp_armed > 0.1f;
				}
				break;
			}

			case 2: //INBOUND_MSG
			{
				if (_simulink_inbound_sub.update(&sm_inbound))
				{
					armed = sm_inbound.data[CONTROL_VEC_START_ID + ARMED_IND] > 0.1f;
				}
				break;
			}

			default:
				if (commander_updated_armed_state || act_armed.armed != act_armed_px4.armed)
				{
					act_armed = act_armed_px4;
					return true;
				}
				else
				{
					return false;
				}

			}
			break;


		}
		if (armed != act_armed.armed || (commander_updated_armed_state && act_armed_px4.armed != armed))
		{
			act_armed.timestamp = hrt_absolute_time();
			act_armed.ready_to_arm = true;
			act_armed.prearmed = true;
			act_armed.armed = armed;
			act_armed.force_failsafe = false;
			act_armed.lockdown =false;
			act_armed.manual_lockdown = false;
			_actuator_armed_pub.publish(act_armed);
			return true;
		}
		else return false;
	}
	return false;
}

enum control_level
{
	CALIBRATION = 1,
	INNER_LOOP_LQI = 2,
	INNER_LOOP_TECS = 3,
	OUTER_LOOP = 4
};

void SIM_CTRL_MOD::printf_debug_array(debug_array_s &array)
{
	static uint64_t old_time = array.timestamp;
	double update_rate_Hz = 1000000.0 / (static_cast<double>(array.timestamp - old_time));
	printf("UpdateRate: %3.1f; HEADER: Name: %10s, ID: %5i, Timestamp %" PRIu64 , update_rate_Hz, array.name, array.id, array.timestamp);
	for (int i = 0; i < 58; i++)
	{
		printf(", %3.1f",static_cast<double>(array.data[i]));
	}
	printf("\33[2K\r");
	old_time = array.timestamp;
}

void SIM_CTRL_MOD::printf_actuator_output(actuator_outputs_s &outputs)
{
	static uint64_t old_time = outputs.timestamp;
	double update_rate_Hz = 1000000.0 / (static_cast<double>(outputs.timestamp - old_time));
	printf("UpdateRate: %3.1f; HEADER: N: %5i, Timestamp %" PRIu64 , update_rate_Hz, outputs.noutputs, outputs.timestamp);
	for (int i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++)
	{
		printf(", %3.1f",static_cast<double>(outputs.output[i]));
	}
	printf("\33[2K\r");
	old_time = outputs.timestamp;
	return;
}

void SIM_CTRL_MOD::test_fake_atuator_data(void)
{
	actuator_outputs_s sv_out{};

	//sv_out.noutputs = 16;
	sv_out.timestamp = hrt_absolute_time();
	for (int i = 0; i < 16; i++)
	{
		sv_out.output[i] = static_cast<float>(i) * 50.f;
	}

	_actuator_outputs_sv_pub.publish(sv_out);
}



bool SIM_CTRL_MOD::update_control_inputs(float in_vec[CONTROL_VEC_SIZE])
{
	bool need_update = false;
	//bool publish_new_manual_control_setpoint = false;
	int input_source_opt = _param_cmd_opt.get();

	//Control vector elements:
	float roll = 0.f;		//[-1 1]
	float pitch = 0.f; 		//[-1 1]
	float yaw = 0.f;		//[-1 1]
	float throttle = 0.f;		//[0 1]
	float manual_wing_ch = 0.f;	//[-1 1]

	float mode_stick = -1.f; 			//[-1 1] actual stick position
	control_level mode_ch = INNER_LOOP_LQI;		//[1 4] mode that corresponds to the stick position

	int32_t en_calibration = _param_sm_en_cal.get();



	switch (input_source_opt)
	{
	case 1: // RC_IN
	{
		if(_rc_channels_sub.update(&rc_ch)) need_update = true;

		//check every stick so we are sure rc is valid, otherwise quit
		rc_map_stick(roll, rc_ch, rc_channels_s::FUNCTION_ROLL);
		rc_map_stick(pitch, rc_ch, rc_channels_s::FUNCTION_PITCH);
		rc_map_stick(yaw, rc_ch, rc_channels_s::FUNCTION_YAW);
		rc_map_stick(throttle, rc_ch, rc_channels_s::FUNCTION_THROTTLE);
		mode_stick = rc_ch.channels[rc_channels_s::FUNCTION_MODE];

		break;
	}


	case 2: //INBOUND_MSG
		//if (_simulink_inbound_sub.update(&sm_inbound)) publish_new_manual_control_setpoint = true;
		roll 		= sm_inbound.data[CONTROL_VEC_START_ID + ROLL_IND];
		pitch 		= sm_inbound.data[CONTROL_VEC_START_ID + PITCH_IND];
		yaw 		= sm_inbound.data[CONTROL_VEC_START_ID + YAW_IND];
		throttle 	= sm_inbound.data[CONTROL_VEC_START_ID + THROTTLE_IND];
		mode_stick 	= static_cast<control_level>(sm_inbound.data[CONTROL_VEC_START_ID + MODE_IND]);


		break;//assume everything was already sent correctly


	default: //MANUAL_CONTROL_SETPOINT
		if (_manual_control_setpoint_sub.update(&man_setpoint)) need_update = true;
		if (_manual_control_switches_sub.update(&man_switches)) need_update = true;

		roll = man_setpoint.y;
		pitch = man_setpoint.x;
		yaw = man_setpoint.r;
		throttle = man_setpoint.z;
		mode_stick = man_switches.mode_slot;

		break;
	}
	bool armed_switch = false;
	if (check_armed(armed_switch, input_source_opt)) need_update = true;
	if (update_man_wing_angle(input_source_opt, manual_wing_ch)) need_update = true;

	if (en_calibration == 1)
	{
		mode_ch = CALIBRATION;
	}
	else
	{
		if (mode_stick > 0.7f) mode_ch = OUTER_LOOP;
		else if(mode_stick < -0.7f) mode_ch = INNER_LOOP_LQI;
		else mode_ch = INNER_LOOP_TECS; //must always assign some default
	}

	in_vec[ROLL_IND] = roll;
	in_vec[PITCH_IND] = pitch;
	in_vec[YAW_IND] = yaw;
	in_vec[THROTTLE_IND] = throttle;
	in_vec[AUX6_IND] = manual_wing_ch;
	in_vec[ARMED_IND] = static_cast<float>(armed_switch);
	in_vec[MODE_IND] = static_cast<float>(mode_ch);

	/*
	if (publish_new_manual_control_setpoint) //let the rest of PX4 know we have control input
	{

		//PX4_INFO("Updating manual control from INBOUND");
		//PARAM_DEFINE_INT32(COM_RC_IN_MODE, 0);

		int32_t com_rc_in_mode = 0;
		param_get(param_find("COM_RC_IN_MODE"), &com_rc_in_mode);


		if (com_rc_in_mode == 1)
		{
			//PX4_INFO("Updating manual control setpoint directly from INBOUND");
			man_setpoint.timestamp = hrt_absolute_time();
			man_setpoint.y = roll;
			man_setpoint.x = pitch;
			man_setpoint.r = yaw;
			man_setpoint.z = throttle;
			man_setpoint.aux6 = manual_wing_ch;
			man_setpoint.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0;

			_manual_control_setpoint_pub.publish(man_setpoint);

			man_switches.timestamp = hrt_absolute_time();
			man_switches.arm_switch = armed_switch;
			man_switches.kill_switch = !armed_switch;
			man_switches.mode_switch = mode_stick;

			_manual_control_switches_pub.publish(man_switches);
		}
		else
		{
			//PX4_INFO("Updating virtual rc_input from INBOUND");
			input_rc_s rc{};
			rc.timestamp = hrt_absolute_time();
			rc.timestamp_last_signal = rc.timestamp;

			rc.channel_count = 8;
			rc.rc_failsafe = false;
			rc.rc_lost = false;
			rc.rc_lost_frame_count = 0;
			rc.rc_total_frame_count = 1;
			rc.rc_ppm_frame_length = 0;
			rc.input_source = input_rc_s::RC_INPUT_SOURCE_MAVLINK;
			rc.rssi = RC_INPUT_RSSI_MAX;

			rc.values[0] = static_cast<uint16_t>(roll * 500.f) + 1500;	// roll
			rc.values[1] = static_cast<uint16_t>(pitch * 500.f) + 1500;	// pitch
			rc.values[2] = static_cast<uint16_t>(yaw * 500.f) + 1500;	// yaw
			rc.values[3] = static_cast<uint16_t>(throttle * 1000.f) + 1000;	// throttle

			// decode all switches which fit into the channel mask
			unsigned max_switch = 8;
			unsigned max_channels = (sizeof(rc.values) / sizeof(rc.values[0]));

			if (max_switch > (max_channels - 4)) {
				max_switch = (max_channels - 4);
			}
			rc.values[4] = static_cast<uint16_t>(manual_wing_ch * 500.f) + 1500;
			rc.values[5] = static_cast<uint16_t>(armed_switch)*1000 + 1000;
			rc.values[6] = static_cast<uint16_t>(mode_stick)*1000 + 1000;

			_input_rc_pub.publish(rc);
		}
	}
	*/

	return need_update;
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
	if (update_control_inputs(control_vec)) need_2_pub = true;



	//publish new data if needed:
	if (need_2_pub)
	{
		simulink_inboud_data.fill_buffer(control_vec, CONTROL_VEC_SIZE);

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


		//publish new data:
		debug_array_s debug_topic{};

		debug_topic.timestamp = hrt_absolute_time();
		debug_topic.id = debug_array_s::SIMULINK_INBOUND_ID;
		char message_name[10] = "inbound";
		memcpy(debug_topic.name, message_name, sizeof(message_name));
		debug_topic.name[sizeof(debug_topic.name) - 1] = '\0'; // enforce null termination

		simulink_inboud_data.send_vec(debug_topic.data);

		if (_param_en_hil.get() == 0) _simulink_inbound_pub.publish(debug_topic); //if HIL/SITL is enabled, assume this data was already published

		//_simulink_outbound_pub.publish(debug_topic);
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
