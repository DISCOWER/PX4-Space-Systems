/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include <lib/rate_control/rate_control.hpp>
// #include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

using namespace time_literals;

class MulticopterPosMPControl : public ModuleBase<MulticopterPosMPControl>, public ModuleParams, public px4::WorkItem
{
public:
	MulticopterPosMPControl();
	~MulticopterPosMPControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};


	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};	/**< vehicle local position */
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)}; 	/**< notification of manual control updates */

	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	hrt_abstime _time_stamp_last_loop{0};		/**< time stamp of last loop iteration */
	hrt_abstime _time_position_control_enabled{0};
	hrt_abstime _manual_setpoint_last_called{0};

	trajectory_setpoint_s _setpoint;
	vehicle_control_mode_s _vehicle_control_mode{};
	manual_control_setpoint_s		_manual_control_setpoint{};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<actuator_controls_status_s>	_actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)};
	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)};
	uORB::Publication<vehicle_rates_setpoint_s>	_vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub;
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub;
	vehicle_status_s	_vehicle_status{};

	hrt_abstime _last_run{0};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */
	bool stabilized_pos_sp_initialized{false};
	matrix::Vector3f target_pos_sp;


	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	// keep setpoint values between updates
	//matrix::Vector3f _acro_rate_max;		/**< max attitude rates in acro mode */
	//matrix::Vector3f _rates_setpoint{};
	float _manual_torque_max{1.0};
	float _manual_force_max{1.0};

	//matrix::Vector3f _thrust_setpoint{};
	//matrix::Vector3f _torque_setpoint{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SC_MPC_W_PX>) _param_sc_mpc_w_px,
		(ParamFloat<px4::params::SC_MPC_W_PY>) _param_sc_mpc_w_py,
		(ParamFloat<px4::params::SC_MPC_W_PZ>) _param_sc_mpc_w_pz,
		(ParamFloat<px4::params::SC_MPC_W_VX>) _param_sc_mpc_w_vx,
		(ParamFloat<px4::params::SC_MPC_W_VY>) _param_sc_mpc_w_vy,
		(ParamFloat<px4::params::SC_MPC_W_VZ>) _param_sc_mpc_w_vz,

		(ParamFloat<px4::params::SC_MPC_W_QX>) _param_sc_mpc_w_qx,
		(ParamFloat<px4::params::SC_MPC_W_QY>) _param_sc_mpc_w_qy,
		(ParamFloat<px4::params::SC_MPC_W_QZ>) _param_sc_mpc_w_qz,
		(ParamFloat<px4::params::SC_MPC_W_QW>) _param_sc_mpc_w_qw,
		(ParamFloat<px4::params::SC_MPC_W_WX>) _param_sc_mpc_w_wx,
		(ParamFloat<px4::params::SC_MPC_W_WY>) _param_sc_mpc_w_wy,
		(ParamFloat<px4::params::SC_MPC_W_WZ>) _param_sc_mpc_w_wz,

		(ParamFloat<px4::params::SC_MPC_W_UFX>) _param_sc_mpc_w_ufx,
		(ParamFloat<px4::params::SC_MPC_W_UFY>) _param_sc_mpc_w_ufy,
		(ParamFloat<px4::params::SC_MPC_W_UFZ>) _param_sc_mpc_w_ufz,

		(ParamFloat<px4::params::SC_MPC_W_UTX>) _param_sc_mpc_w_utx,
		(ParamFloat<px4::params::SC_MPC_W_UTY>) _param_sc_mpc_w_uty,
		(ParamFloat<px4::params::SC_MPC_W_UTZ>) _param_sc_mpc_w_utz
	)

	void poll_manual_setpoint(const float dt, const vehicle_local_position_s
				  &vehicle_local_position, const vehicle_attitude_s &_vehicle_att);
};
