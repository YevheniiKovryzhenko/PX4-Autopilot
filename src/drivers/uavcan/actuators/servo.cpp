/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

#include "servo.hpp"
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

// these are the settings assumed to be used on the other end of CAN->PWM board:
const float assumed_min = 1000.f;
const float assumed_trim = 1500.f;
const float assumed_max = 2000.f;
const float assumed_range = assumed_max - assumed_min;

const float esc_safety_cmd = 1000.f;
const float assumed_trim_esc = 1000.f; //let's always send min output

UavcanServoController::UavcanServoController(uavcan::INode &node) :
	_node(node),
	_uavcan_pub_array_cmd(node),
	_timer(node)
{
	_uavcan_pub_array_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);

	memset(sv_en_fl, 0, sizeof(sv_en_fl));
	memset(sv_rev_fl, 0, sizeof(sv_en_fl));
	memset(sv_min, 0, sizeof(sv_min));
	memset(sv_max, 0, sizeof(sv_max));
	memset(sv_trim, 0, sizeof(sv_trim));
	memset(sv_disarm, 0, sizeof(sv_disarm));
	memset(sv_fail, 0, sizeof(sv_fail));

	for (int i = 0; i < MAX_ACTUATORS; i++)
	{
		sv_en_fl[i] = false;
		sv_rev_fl[i] = false;
		sv_esc_fl[i] = false;
		sv_id[i] = i + 1;
		sv_min[i] = PWM_DEFAULT_MIN;
		sv_max[i] = PWM_DEFAULT_MAX;
		sv_trim[i] = PWM_DEFAULT_MIN + (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN)/2;
		sv_disarm[i] = -1;
		sv_fail[i] = -1;
	}


}

int UavcanServoController::init()
{
	/*
	 * Setup timer and call back function for periodic updates
	 */
	if (!_timer.isRunning()) {
		_timer.setCallback(TimerCbBinder(this, &UavcanServoController::update));
		_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / MAX_RATE_HZ));
	}

	return 0;
}

void
UavcanServoController::update(const uavcan::TimerEvent &)
{
	actuator_outputs_s actuator_outputs_sv;
	actuator_armed_s actuator_armed;

	if (_actuator_outputs_sv_sub.update(&actuator_outputs_sv) || _actuator_armed_sub.update(&actuator_armed))
	{
		update_outputs(actuator_armed.armed, actuator_armed.force_failsafe, actuator_outputs_sv.output);
	}
}

void
UavcanServoController::update_outputs(bool armed, bool fail, float outputs[MAX_ACTUATORS])
{
	bool write_anything = false;
	uavcan::equipment::actuator::ArrayCommand msg;
	for (unsigned i = 0; i < MAX_ACTUATORS; ++i) {
		uavcan::equipment::actuator::Command cmd;
		if (sv_en_fl[i])
		{
			cmd.actuator_id = i + 1;
			cmd.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;

			float true_range = (float)(sv_max[i] - sv_min[i]);
			float range_ratio = true_range / assumed_range;

			float pwm_cmd = (float)outputs[i] + 1000.f; //input is [0 1000] need [1000 2000]
			if (sv_esc_fl[i]) //we are talking with an ESC with one-shot
			{
				if (fail || !armed) pwm_cmd = esc_safety_cmd; //always send minimum value if disarmed or in failsafe mode
			}
			else // we are talking with a servo
			{
				if (sv_fail[i] > 0 && fail) pwm_cmd = (float)sv_fail[i]; //use fail value if enabled and active
				else if (sv_disarm[i] > 0 && !armed) pwm_cmd = (float)sv_disarm[i]; //use armed value if enabled and active
			}


			// take commanded value:
			float norm_cmd = 2.f * (pwm_cmd - assumed_min) / assumed_range - 1.f; //[-1 1]
			if (sv_rev_fl[i]) norm_cmd = -norm_cmd; //polarity

			norm_cmd = norm_cmd * range_ratio;// range adjusted for true [min, max], which within [-1 1]
			norm_cmd += (float)sv_trim[i] / assumed_range; //trim value offset

			//check final saturation -> trim value can make it go outside the bounds
			if (norm_cmd > 1.f) norm_cmd = 1.f;
			else if (norm_cmd < -1.f) norm_cmd = -1.f;


			cmd.command_value = norm_cmd;

			msg.commands.push_back(cmd);
			write_anything = true;
		}

	}
	if (write_anything) _uavcan_pub_array_cmd.broadcast(msg);
}

void
UavcanServoController::update_params(void)
{
	int32_t pwm_min_default = PWM_DEFAULT_MIN;
	int32_t pwm_max_default = PWM_DEFAULT_MAX;
	int32_t pwm_disarmed_default = -1;
	int32_t pwm_fail_default = -1;

	char str[17];
	const char *prefix;

	prefix = "UAVCAN_SV";

	sprintf(str, "%s_MIN", prefix);
	param_get(param_find(str), &pwm_min_default);

	sprintf(str, "%s_MAX", prefix);
	param_get(param_find(str), &pwm_max_default);

	sprintf(str, "%s_DISARM", prefix);
	param_get(param_find(str), &pwm_disarmed_default);

	sprintf(str, "%s_FAIL", prefix);
	param_get(param_find(str), &pwm_fail_default);



	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		// ENx
		{
			sprintf(str, "%s_EN%u", prefix, i + 1);
			int32_t en_fl = 0;
			if (param_get(param_find(str), &en_fl) == PX4_OK)
			{
				sv_en_fl[i] = en_fl >= 1;
			}
		}

		{
			sprintf(str, "%s_ESC%u", prefix, i + 1);
			int32_t esc_fl = 0;
			if (param_get(param_find(str), &esc_fl) == PX4_OK)
			{
				sv_esc_fl[i] = esc_fl >= 1;
			}
		}

		{
			sprintf(str, "%s_ID%u", prefix, i + 1);
			int32_t tmp_sv_id = i + 1;
			if (param_get(param_find(str), &tmp_sv_id) == PX4_OK)
			{
				if (tmp_sv_id < 0 || tmp_sv_id > 256) // don't send IDs outside of the bounds, something is wrong
				{
					sv_esc_fl[i] = false;
					sprintf(str, "%s_EN%u", prefix, i + 1);
					int32_t en_fl = false;
					param_set(param_find(str),&en_fl);
				}
				else
				{
					sv_esc_fl[i] = tmp_sv_id;
				}
			}
		}

		// MINx and MAXx
		{
			char str_max[17];
			sprintf(str, "%s_MIN%u", prefix, i + 1);
			sprintf(str_max, "%s_MAX%u", prefix, i + 1);
			int32_t pwm_min = pwm_min_default;
			int32_t pwm_max = pwm_max_default;
			// check if min is outside the bounds + check if max is outside the bounds + check if max is less than min
			if ((param_get(param_find(str), &pwm_min) == PX4_OK) && (param_get(param_find(str_max), &pwm_max) == PX4_OK)) {
				if ((pwm_min < pwm_min_default || pwm_min > pwm_max_default) || (pwm_max > pwm_max_default || pwm_max < pwm_min_default) || (pwm_max < pwm_min) )
				{
					//reset to default:
					pwm_min = pwm_min_default;
					pwm_max = pwm_max_default;
					param_set(param_find(str), &pwm_min);
					param_set(param_find(str_max), &pwm_max);

					//disable actuator output:
					//sprintf(str, "%s_EN%u", prefix, i + 1);
					//bool en_fl = false;
					//param_set(param_find(str), &en_fl)
					//sv_en_fl[i] = en_fl;
				}
				sv_min[i] = pwm_min;
				sv_max[i] = pwm_max;
			}
		}

		// PWM_MAIN_FAILx
		{
			sprintf(str, "%s_FAIL%u", prefix, i + 1);
			int32_t pwm_failsafe = -1;
			if (param_get(param_find(str), &pwm_failsafe) == PX4_OK)
			{
				if (pwm_failsafe < 0) pwm_failsafe = pwm_fail_default;
				sv_fail[i] = pwm_failsafe;
			}

		}

		// PWM_MAIN_DISx
		{
			sprintf(str, "%s_DIS%u", prefix, i + 1);
			int32_t pwm_dis = -1;

			if (param_get(param_find(str), &pwm_dis) == PX4_OK) {
				if (pwm_dis < 0) pwm_dis = pwm_disarmed_default;
				sv_disarm[i] = pwm_dis;
			}

		}

		// REVx
		{
			sprintf(str, "%s_REV%u", prefix, i + 1);
			int32_t pwm_rev = 0;

			if (param_get(param_find(str), &pwm_rev) == PX4_OK) {
				sv_rev_fl[i] = pwm_rev >= 1;
			}

		}

		// PWM_MAIN_TRIMx
		if (sv_esc_fl[i]) sv_trim[i] = 0.f; //let's just ignore trim value for ESCs
		else
		{

			sprintf(str, "%s_TRIM%u", prefix, i + 1);
			int32_t pwm_trim = 0;
			param_get(param_find(str), &pwm_trim);

			if (sv_esc_fl[i]) //we are talking with an ESC with one-shot
			{
				if (sv_rev_fl[i]) pwm_trim += (int32_t)assumed_max - sv_max[i]; //this will shift the new output range to start from +1
				else pwm_trim += (int32_t)assumed_min - sv_min[i]; //this will shift new output range to start from -1 (default, since cmd = -1 -> lowest PWM)
			}
			else // we are talking with a servo
			{

				pwm_trim += (sv_max[i] - sv_min[i]) / 2 + sv_min[i] - (int32_t)assumed_trim;
			}

			sv_trim[i] = pwm_trim;
		}
	}
}
