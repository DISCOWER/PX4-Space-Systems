/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

/**
 * Multicopter position controller.
 */

#pragma once

#include "SpacecraftMPC/SpacecraftMPC.hpp"

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

using namespace time_literals;

class SpacecraftModelPredictiveControl : public ModuleBase<SpacecraftModelPredictiveControl>,
	public ModuleParams, public px4::ScheduledWorkItem
{
public:
	SpacecraftModelPredictiveControl();
	~SpacecraftModelPredictiveControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	orb_advert_t _mavlink_log_pub{nullptr};

	uORB::Publication<vehicle_attitude_setpoint_s>	     _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};	/**< vehicle local position setpoint publication */

	uORB::SubscriptionCallbackWorkItem 	_local_pos_sub{this, ORB_ID(vehicle_local_position)};	/**< vehicle local position */
	uORB::SubscriptionInterval 		_parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription 			_manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)}; 	/**< notification of manual control updates */

	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};

	hrt_abstime _time_stamp_last_loop{0};		/**< time stamp of last loop iteration */
	hrt_abstime _time_position_control_enabled{0};
	hrt_abstime _manual_setpoint_last_called{0};

	trajectory_setpoint_s 		_setpoint{SpacecraftMPC::empty_trajectory_setpoint};
	vehicle_control_mode_s		_vehicle_control_mode{};
	manual_control_setpoint_s	_manual_control_setpoint{};			    /**< r/c channel data */

	DEFINE_PARAMETERS(
		// Position Control
		(ParamFloat<px4::params::SPC_MPC_Q_POS_X>)        _param_spc_mpc_q_pos_x,
		(ParamFloat<px4::params::SPC_MPC_Q_POS_Y>)        _param_spc_mpc_q_pos_y,
		(ParamFloat<px4::params::SPC_MPC_Q_POS_Z>)	  _param_spc_mpc_q_pos_z,
		(ParamFloat<px4::params::SPC_MPC_Q_VEL_X>) 	  _param_spc_mpc_q_vel_x,
		(ParamFloat<px4::params::SPC_MPC_Q_VEL_Y>) 	  _param_spc_mpc_q_vel_y,
		(ParamFloat<px4::params::SPC_MPC_Q_VEL_Z>)    	  _param_spc_mpc_q_vel_z,
		(ParamFloat<px4::params::SPC_MPC_Q_ATT_X>) 	  _param_spc_mpc_q_att_x,
		(ParamFloat<px4::params::SPC_MPC_Q_ATT_Y>)   	  _param_spc_mpc_q_att_y,
		(ParamFloat<px4::params::SPC_MPC_Q_ATT_Z>)   	  _param_spc_mpc_q_att_z,
		(ParamFloat<px4::params::SPC_MPC_Q_ATT_W>)        _param_spc_mpc_q_att_w,
		(ParamFloat<px4::params::SPC_MPC_Q_OMG_X>)        _param_spc_mpc_q_omg_x,
		(ParamFloat<px4::params::SPC_MPC_Q_OMG_Y>)        _param_spc_mpc_q_omg_y,
		(ParamFloat<px4::params::SPC_MPC_Q_OMG_Z>)        _param_spc_mpc_q_omg_z,
		(ParamFloat<px4::params::SPC_MPC_R_F_X>)          _param_spc_mpc_r_f_x,
		(ParamFloat<px4::params::SPC_MPC_R_F_Y>)          _param_spc_mpc_r_f_y,
		(ParamFloat<px4::params::SPC_MPC_R_F_Z>)    	  _param_spc_mpc_r_f_z,
		(ParamFloat<px4::params::SPC_MPC_R_T_X>)    	  _param_spc_mpc_r_t_x,
		(ParamFloat<px4::params::SPC_MPC_R_T_Y>)     	  _param_spc_mpc_r_t_y,
		(ParamFloat<px4::params::SPC_MPC_R_T_Z>)      	  _param_spc_mpc_r_t_z
	);

	matrix::Vector3f target_pos_sp;
	float yaw_rate;
	bool stabilized_pos_sp_initialized{false};

	// gains
	matrix::Vector3f _pos_gain;
	matrix::Vector3f _vel_gain;
	matrix::Vector4f _att_gain;
	matrix::Vector3f _omg_gain;
	matrix::Vector3f _force_gain;
	matrix::Vector3f _torque_gain;

	SpacecraftMPC _control; // class for MPC control

	hrt_abstime _last_warn{0}; /**< timer when the last warn message was sent out */

	/** Timeout in us for trajectory data to get considered invalid */
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;

	// Manual setpoints on yaw and reset
	bool _reset_yaw_sp{true};
	float _manual_yaw_sp{0.f};
	float _throttle_control{0.f};
	float _yaw_control{0.f};

	// Maximum thrust
	float _thrust_max{1.4f};
	float _torque_max{0.14f};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};

	/**
	 * Update our local parameter cache.
	 * Parameter update can be forced when argument is true.
	 * @param force forces parameter update.
	 */
	void parameters_update(bool force);

	/**
	 * Check for validity of positon/velocity states.
	 */
	SpacecraftMPCStates set_vehicle_states(const vehicle_local_position_s &local_pos, const vehicle_attitude_s &att);

	/**
	 * Check for manual setpoints.
	 */
	void poll_manual_setpoint(const float dt, const vehicle_local_position_s
				  &vehicle_local_position, const vehicle_attitude_s &_vehicle_att);

	/**
	 * @brief publishes target setpoint.
	 *
	 */
    void publishLocalPositionSetpoint(vehicle_attitude_setpoint_s &_att_sp);

        /**
	 * Generate setpoint to bridge no executable setpoint being available.
	 * Used to handle transitions where no proper setpoint was generated yet and when the received setpoint is invalid.
	 * This should only happen briefly when transitioning and never during mode operation or by design.
	 */
	trajectory_setpoint_s generateFailsafeSetpoint(const hrt_abstime &now, const SpacecraftMPCStates &states, bool warn);
};
