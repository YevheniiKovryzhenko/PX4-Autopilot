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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
//#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

//#include <uORB/topics/sm_full_state.h>

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/adc_report.h>
//#include <uORB/topics/obstacle_distance.h>
//#include <uORB/topics/rc_channels.h>
//#include <uORB/topics/rc_parameter_map.h>

#include <uORB/topics/debug_array.h>
//#include <uORB/topics/simulink_inbound.h>
//#include <uORB/topics/simulink_outbound.h>

extern "C" __EXPORT int sim_ctrl_mod_main(int argc, char *argv[]);


class sim_data_trafic
{
public:
	sim_data_trafic();
	~sim_data_trafic();

	static const uint MAX_SIZE = debug_array_s::ARRAY_SIZE;

	void send_vec(float out_vec[MAX_SIZE]);
	char fill_buffer(float in);
	char fill_buffer(float* in, uint size);
	void clear_buffer(void);

private:

	uint ind;
	float data[MAX_SIZE];
};


class SIM_CTRL_MOD : public ModuleBase<SIM_CTRL_MOD>, public ModuleParams
{
public:

	SIM_CTRL_MOD(int example_param, bool example_flag);

	virtual ~SIM_CTRL_MOD() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static SIM_CTRL_MOD *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;


private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);

	//sm_full_state_s        _sm_full_state{};
	//uORB::Publication<sm_full_state_s>	_sm_full_state_pub{ORB_ID(sm_full_state)};
	//uORB::Publication<debug_array_s> 	_debug_array_pub{ORB_ID(debug_array)};
	uORB::Publication<debug_array_s>	_simulink_inbound_pub{ORB_ID(simulink_inbound)};


	// Subscriptions
	uORB::Subscription		_parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription 		_vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription 		_vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription		_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription		_airspeed_sub{ORB_ID(airspeed)};
	uORB::Subscription		_battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription		_distance_sensor_sub{ORB_ID(distance_sensor)};
	uORB::Subscription		_actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription		_vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription		_adc_report_sub{ORB_ID(adc_report)};
	//uORB::Subscription		_obstacle_distance_sub{ORB_ID(obstacle_distance)};
	//uORB::Subscription		_rc_channels_sub{ORB_ID(rc_channels)};
	//uORB::Subscription		_rc_parameter_map_sub{ORB_ID(rc_parameter_map)};
	//uORB::Subscription		_simulink_outbound_sub{ORB_ID(simulink_outbound)};

	vehicle_local_position_s local_pos;
	vehicle_global_position_s global_pos;
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


	void publish_inbound_sim_data(void);
	sim_data_trafic simulink_inboud_data{};

	bool check_ground_contact(void);
	bool check_armed(void);

	/**
	 * THIS IS WHERE YOU DEFINE NEW PARAMETRS
	 * example:
	 * DEFINE_PARAMETERS(
	 *	(ParamFloat<px4::params::SM_LQI_LG_SAT>) _param_sm_lqi_lg_sat)
	 *
	 * where:
	 *	DEFINE_PARAMETERS is macro (short function)
	 * 	ParamFloat defines the data type of the parameter (float in this case)
	 * 	SM_LQI_LG_SAT is the global name that other app will see (e.g. simulink or QGC). Make sure this is under 14 characters
	 *	_param_sm_lqi_lg_sat is the local variable (used in this app only).
	 *	They MUST BE UPPERCASE AND lowercase respectively (don't mix)
	 */
	DEFINE_PARAMETERS(

		(ParamFloat<px4::params::SM_AUX_MIX_1>) _param_sm_aux_mix_1,
		(ParamFloat<px4::params::SM_AUX_MIX_2>) _param_sm_aux_mix_2,
		(ParamFloat<px4::params::SM_AUX_MIX_3>) _param_sm_aux_mix_3,
		(ParamFloat<px4::params::SM_AUX_MIX_4>) _param_sm_aux_mix_4,
		(ParamFloat<px4::params::SM_AUX_MIX_5>) _param_sm_aux_mix_5,
		(ParamFloat<px4::params::SM_AUX_MIX_6>) _param_sm_aux_mix_6,
		(ParamFloat<px4::params::SM_AUX_MIX_7>) _param_sm_aux_mix_7,
		(ParamFloat<px4::params::SM_AUX_MIX_8>) _param_sm_aux_mix_8,
		(ParamFloat<px4::params::SM_AUX_MIX_9>) _param_sm_aux_mix_9,
		(ParamFloat<px4::params::SM_AUX_MIX_10>) _param_sm_aux_mix_10,
		(ParamFloat<px4::params::SM_AUX_MIX_11>) _param_sm_aux_mix_11,
		(ParamFloat<px4::params::SM_AUX_MIX_12>) _param_sm_aux_mix_12,
		(ParamFloat<px4::params::SM_AUX_MIX_13>) _param_sm_aux_mix_13,
		(ParamFloat<px4::params::SM_AUX_MIX_14>) _param_sm_aux_mix_14,
		(ParamFloat<px4::params::SM_AUX_MIX_15>) _param_sm_aux_mix_15,
		(ParamFloat<px4::params::SM_AUX_MIX_16>) _param_sm_aux_mix_16,

		(ParamFloat<px4::params::SM_IDLE_TH>) _param_sm_idle_th,
		(ParamFloat<px4::params::SM_GAIN_TH_T>) _param_sm_gain_th_t,
		(ParamInt<px4::params::SM_EN_CAL>) _param_sm_en_cal,
		(ParamInt<px4::params::SM_OVERWRITE>) _param_sm_overwrite,
		(ParamInt<px4::params::SM_GC_OPT>) _param_gc_opt,
		(ParamInt<px4::params::SM_GC_SET>) _param_gc_set,
		(ParamInt<px4::params::SM_EN_HIL>) _param_en_hil,

		(ParamFloat<px4::params::SM_LATMPTYP>) _param_sm_latmptyp,

		(ParamFloat<px4::params::SM_TAU_LAT>) _param_sm_tau_lat,
		(ParamFloat<px4::params::SM_LAT_DEADZ>) _param_sm_lat_deadz,
		(ParamFloat<px4::params::SM_VLAT_MAX>) _param_sm_vlat_max,
		(ParamFloat<px4::params::SM_PHMAX_VFM>) _param_sm_phmax_vfm,
		(ParamFloat<px4::params::SM_PHMAX_FFM>) _param_sm_phmax_ffm,
		(ParamFloat<px4::params::SM_PHDOT_MAX>) _param_sm_phdot_max,
		(ParamFloat<px4::params::SM_DIH_LOW>) _param_sm_dih_low,
		(ParamFloat<px4::params::SM_DIH_HIGH>) _param_sm_dih_high,
		(ParamFloat<px4::params::SM_KP_VLAT>) _param_sm_kei_kp_vlat,
		(ParamFloat<px4::params::SM_KI_VLAT>) _param_sm_ki_vlat,
		(ParamFloat<px4::params::SM_ZLAT_LOW>) _param_sm_zlat_low,
		(ParamFloat<px4::params::SM_ZLAT_HIGH>) _param_sm_zlat_high,
		(ParamFloat<px4::params::SM_LONMAPTYP>) _param_sm_lonmaptyp,
		(ParamFloat<px4::params::SM_TAU_LON>) _param_sm_tau_lon,
		(ParamFloat<px4::params::SM_LON_DEADZ>) _param_sm_lon_deadz,
		(ParamFloat<px4::params::SM_VVCMD_MAX>) _param_sm_vvcmd_max,
		(ParamFloat<px4::params::SM_FPA_LIM>) _param_sm_fpa_lim,
		(ParamFloat<px4::params::SM_FPADOTMAX>) _param_sm_fpadotmax,
		(ParamFloat<px4::params::SM_N_MIN>) _param_sm_n_min,
		(ParamFloat<px4::params::SM_N_MAX>) _param_sm_n_max,
		(ParamFloat<px4::params::SM_ZLON_LOW>) _param_sm_zlon_low,
		(ParamFloat<px4::params::SM_ZLON_HIGH>) _param_sm_zlon_high,
		(ParamFloat<px4::params::SM_TAU_DIR>) _param_sm_tau_dir,
		(ParamFloat<px4::params::SM_DIR_DEADZ>) _param_sm_dir_deadz,
		(ParamFloat<px4::params::SM_RCMD_MAX>) _param_sm_rcmd_max,
		(ParamFloat<px4::params::SM_BCMD_MAX>) _param_sm_bcmd_max,
		(ParamFloat<px4::params::SM_KBETA>) _param_sm_kbeta,
		(ParamFloat<px4::params::SM_KNY>) _param_sm_kny,
		(ParamFloat<px4::params::SM_TAU_ACC>) _param_sm_tau_acc,
		(ParamFloat<px4::params::SM_ACC_DEADZ>) _param_sm_acc_deadz,
		(ParamFloat<px4::params::SM_ACC_CM_MX>) _param_sm_acc_cm_mx,
		(ParamFloat<px4::params::SM_ZACC_LOW>) _param_sm_zacc_low,
		(ParamFloat<px4::params::SM_ZACC_HIGH>) _param_sm_zacc_high,
		(ParamFloat<px4::params::SM_KV>) _param_sm_kv,
		(ParamFloat<px4::params::SM_ALTHLD_ON>) _param_sm_althld_on,
		(ParamFloat<px4::params::SM_KH_P>) _param_sm_kh_p,
		(ParamFloat<px4::params::SM_KH_I>) _param_sm_kh_i,
		(ParamFloat<px4::params::SM_DTAU_ALT>) _param_sm_dtau_alt,
		(ParamFloat<px4::params::SM_VVTHRSHEN>) _param_sm_vvthrshen,
		(ParamFloat<px4::params::SM_VVTHRSHSC>) _param_sm_vvthrshsc,
		(ParamFloat<px4::params::SM_HDGHLD_ON>) _param_sm_hdghld_on,
		(ParamFloat<px4::params::SM_RTHRESH>) _param_sm_rthresh,
		(ParamFloat<px4::params::SM_DTAU_HDG>) _param_sm_dtau_hdg,
		(ParamFloat<px4::params::SM_KPSI_P>) _param_sm_kpsi_p,
		(ParamFloat<px4::params::SM_KPSI_I>) _param_sm_kpsi_i,
		(ParamFloat<px4::params::SM_HDG_VMAX>) _param_sm_hdg_vmax,
		(ParamFloat<px4::params::SM_TRKHLD_ON>) _param_sm_trkhld_on,
		(ParamFloat<px4::params::SM_PHITHRESH>) _param_sm_phithresh,
		(ParamFloat<px4::params::SM_KTRK_P>) _param_sm_ktrk_p,
		(ParamFloat<px4::params::SM_KTRK_I>) _param_sm_ktrk_i,
		(ParamFloat<px4::params::SM_TRK_VMIN>) _param_sm_trk_vmin,
		(ParamFloat<px4::params::SM_POSHLD_ON>) _param_sm_poshld_on,
		(ParamFloat<px4::params::SM_KP_LAT_PH>) _param_sm_kp_lat_ph,
		(ParamFloat<px4::params::SM_KI_LAT_PH>) _param_sm_ki_lat_ph,
		(ParamFloat<px4::params::SM_KD_LAT_PH>) _param_sm_kd_lat_ph,
		(ParamFloat<px4::params::SM_KP_LON_PH>) _param_sm_kp_lon_ph,
		(ParamFloat<px4::params::SM_KI_LON_PH>) _param_sm_ki_lon_ph,
		(ParamFloat<px4::params::SM_KD_LON_PH>) _param_sm_kd_lon_ph,
		(ParamFloat<px4::params::SM_PH_VTHRSH>) _param_sm_ph_vthrsh,
		(ParamFloat<px4::params::SM_DTAU_PH>) _param_sm_dtau_ph,
		(ParamFloat<px4::params::SM_AP_VMAX>) _param_sm_ap_vmax,
		(ParamFloat<px4::params::SM_AP_VVMAX>) _param_sm_ap_vvmax,
		(ParamFloat<px4::params::SM_AP_TO_FPM>) _param_sm_ap_to_fpm,
		(ParamFloat<px4::params::SM_AP_TO_AGL>) _param_sm_ap_to_agl,
		(ParamFloat<px4::params::SM_AP_TO_K>) _param_sm_ap_to_k,
		(ParamFloat<px4::params::SM_DXN_VCMD>) _param_sm_dxn_vcmd,

		(ParamFloat<px4::params::SM_KEI_0>) _param_sm_kei_0,
		(ParamFloat<px4::params::SM_KEP_0>) _param_sm_kep_0,
		(ParamFloat<px4::params::SM_KTI_0>) _param_sm_kti_0,
		(ParamFloat<px4::params::SM_KTP_0>) _param_sm_ktp_0,
		(ParamFloat<px4::params::SM_KFFI_0>) _param_sm_kffi_0,
		(ParamFloat<px4::params::SM_KFFP_0>) _param_sm_kffp_0,
		(ParamFloat<px4::params::SM_DAOADV_0>) _param_sm_daoadv_0,
		(ParamFloat<px4::params::SM_KFFA_0>) _param_sm_kffa_0,

		(ParamFloat<px4::params::SM_KEI_1>) _param_sm_kei_1,
		(ParamFloat<px4::params::SM_KEP_1>) _param_sm_kep_1,
		(ParamFloat<px4::params::SM_KTI_1>) _param_sm_kti_1,
		(ParamFloat<px4::params::SM_KTP_1>) _param_sm_ktp_1,
		(ParamFloat<px4::params::SM_KFFI_1>) _param_sm_kffi_1,
		(ParamFloat<px4::params::SM_KFFP_1>) _param_sm_kffp_1,
		(ParamFloat<px4::params::SM_DAOADV_1>) _param_sm_daoadv_1,
		(ParamFloat<px4::params::SM_KFFA_1>) _param_sm_kffa_1,

		(ParamFloat<px4::params::SM_THMAX_0>) _param_sm_thmax_0,
		(ParamFloat<px4::params::SM_THMIN_0>) _param_sm_thmin_0,
		(ParamFloat<px4::params::SM_ACC2P_0>) _param_sm_acc2p_0,
		(ParamFloat<px4::params::SM_VV2P_0>) _param_sm_vv2p_0,
		(ParamFloat<px4::params::SM_ACC2T_0>) _param_sm_acc2t_0,
		(ParamFloat<px4::params::SM_VV2T_0>) _param_sm_vv2t_0,
		(ParamFloat<px4::params::SM_DECKL_0>) _param_sm_deckl_0,

		(ParamFloat<px4::params::SM_THMAX_1>) _param_sm_thmax_1,
		(ParamFloat<px4::params::SM_THMIN_1>) _param_sm_thmin_1,
		(ParamFloat<px4::params::SM_ACC2P_1>) _param_sm_acc2p_1,
		(ParamFloat<px4::params::SM_VV2P_1>) _param_sm_vv2p_1,
		(ParamFloat<px4::params::SM_ACC2T_1>) _param_sm_acc2t_1,
		(ParamFloat<px4::params::SM_VV2T_1>) _param_sm_vv2t_1,
		(ParamFloat<px4::params::SM_DECKL_1>) _param_sm_deckl_1,

		(ParamFloat<px4::params::SM_KQ_0>) _param_sm_kq_0,
		(ParamFloat<px4::params::SM_KTH_0>) _param_sm_kth_0,
		(ParamFloat<px4::params::SM_KI_TH_0>) _param_sm_ki_th_0,

		(ParamFloat<px4::params::SM_KQ_1>) _param_sm_kq_1,
		(ParamFloat<px4::params::SM_KTH_1>) _param_sm_kth_1,
		(ParamFloat<px4::params::SM_KI_TH_1>) _param_sm_ki_th_1,
		(ParamFloat<px4::params::SM_KP_L_0>) _param_sm_kp_l_0,
		(ParamFloat<px4::params::SM_KR_L_0>) _param_sm_kr_l_0,
		(ParamFloat<px4::params::SM_KPH_L_0>) _param_sm_kph_l_0,
		(ParamFloat<px4::params::SM_KI_PHL_0>) _param_sm_ki_phl_0,
		(ParamFloat<px4::params::SM_KI_RL_0>) _param_sm_ki_rl_0,
		(ParamFloat<px4::params::SM_KP_D_0>) _param_sm_Kp_d_0,
		(ParamFloat<px4::params::SM_KR_D_0>) _param_sm_kr_d_0,
		(ParamFloat<px4::params::SM_KPH_D_0>) _param_sm_kph_d_0,
		(ParamFloat<px4::params::SM_KI_PHD_0>) _param_sm_ki_phd_0,
		(ParamFloat<px4::params::SM_KI_RD_0>) _param_sm_ki_rd_0,

		(ParamFloat<px4::params::SM_KP_L_1>) _param_sm_kp_l_1,
		(ParamFloat<px4::params::SM_KR_L_1>) _param_sm_kr_l_1,
		(ParamFloat<px4::params::SM_KPH_L_1>) _param_sm_kph_l_1,
		(ParamFloat<px4::params::SM_KI_PHL_1>) _param_sm_ki_phl_1,
		(ParamFloat<px4::params::SM_KI_RL_1>) _param_sm_ki_rl_1,
		(ParamFloat<px4::params::SM_KP_D_1>) _param_sm_Kp_d_1,
		(ParamFloat<px4::params::SM_KR_D_1>) _param_sm_kr_d_1,
		(ParamFloat<px4::params::SM_KPH_D_1>) _param_sm_kph_d_1,
		(ParamFloat<px4::params::SM_KI_PHD_1>) _param_sm_ki_phd_1,
		(ParamFloat<px4::params::SM_KI_RD_1>) _param_sm_ki_rd_1,



		(ParamFloat<px4::params::SM_WING_R_LIM>) _param_sm_wing_r_lim
	)//MAKE SURE EVERY PARAMETER IS FOLLOWED BY "," AND LAST ONE DOES NOT HAVE ANYTHING
};

