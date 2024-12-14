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

#include "SpacecraftModelPredictiveControl.hpp"

#include <float.h>
#include <px4_platform_common/events.h>

using namespace matrix;

SpacecraftModelPredictiveControl::SpacecraftModelPredictiveControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(ORB_ID(vehicle_attitude_setpoint))
{
	parameters_update(true);
}

SpacecraftModelPredictiveControl::~SpacecraftModelPredictiveControl()
{
	perf_free(_cycle_perf);
}

bool SpacecraftModelPredictiveControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

void SpacecraftModelPredictiveControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// Set weights
		// Position control gains
		_pos_gain = {_param_spc_mpc_q_pos_x.get(), _param_spc_mpc_q_pos_y.get(), _param_spc_mpc_q_pos_z.get()};
		_vel_gain = {_param_spc_mpc_q_vel_x.get(), _param_spc_mpc_q_vel_y.get(), _param_spc_mpc_q_vel_z.get()};
		_att_gain = {_param_spc_mpc_q_att_x.get(), _param_spc_mpc_q_att_y.get(), _param_spc_mpc_q_att_z.get(), _param_spc_mpc_q_att_w.get()};
		_omg_gain = {_param_spc_mpc_q_omg_x.get(), _param_spc_mpc_q_omg_y.get(), _param_spc_mpc_q_omg_z.get()};
		_force_gain = {_param_spc_mpc_r_f_x.get(), _param_spc_mpc_r_f_y.get(), _param_spc_mpc_r_f_z.get()};
		_torque_gain = {_param_spc_mpc_r_t_x.get(), _param_spc_mpc_r_t_y.get(), _param_spc_mpc_r_t_z.get()};
		_control.setControlWeights(_pos_gain, _vel_gain, _att_gain, _omg_gain, _force_gain, _torque_gain);
	}
}

SpacecraftMPCStates SpacecraftModelPredictiveControl::set_vehicle_states(const vehicle_local_position_s
		&vehicle_local_position, const vehicle_attitude_s &vehicle_attitude)
{
	SpacecraftMPCStates states;

	const Vector2f position_xy(vehicle_local_position.x, vehicle_local_position.y);

	// only set position states if valid and finite
	if (vehicle_local_position.xy_valid && position_xy.isAllFinite()) {
		states.position.xy() = position_xy;

	} else {
		states.position(0) = states.position(1) = NAN;
	}

	if (PX4_ISFINITE(vehicle_local_position.z) && vehicle_local_position.z_valid) {
		states.position(2) = vehicle_local_position.z;

	} else {
		states.position(2) = NAN;
	}

	if (PX4_ISFINITE(vehicle_attitude.q[0]) && PX4_ISFINITE(vehicle_attitude.q[1]) && PX4_ISFINITE(vehicle_attitude.q[2])
	    && PX4_ISFINITE(vehicle_attitude.q[3])) {
		states.attitude = Quatf(vehicle_attitude.q);

	} else {
		states.attitude = Quatf();
	}

	return states;
}

void SpacecraftModelPredictiveControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// reschedule backup
	ScheduleDelayed(100_ms);

	parameters_update(false);

	perf_begin(_cycle_perf);
	vehicle_local_position_s vehicle_local_position;
	vehicle_attitude_s v_att;
	vehicle_angular_velocity_s angular_velocity;

	if (_local_pos_sub.update(&vehicle_local_position)) {
		const float dt =
			math::constrain(((vehicle_local_position.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = vehicle_local_position.timestamp_sample;

		if (_vehicle_control_mode_sub.updated()) {
			const bool previous_position_control_enabled = _vehicle_control_mode.flag_control_position_enabled;

			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				if (!previous_position_control_enabled && _vehicle_control_mode.flag_control_position_enabled) {
					_time_position_control_enabled = _vehicle_control_mode.timestamp;

				} else if (previous_position_control_enabled && !_vehicle_control_mode.flag_control_position_enabled) {
					// clear existing setpoint when controller is no longer active
					_setpoint = SpacecraftMPC::empty_trajectory_setpoint;
				}
			}
		}

		_trajectory_setpoint_sub.update(&_setpoint);
		_vehicle_attitude_sub.update(&v_att);
		_vehicle_angular_velocity_sub.update(&angular_velocity);

		SpacecraftMPCStates states{set_vehicle_states(vehicle_local_position, v_att)};

		poll_manual_setpoint(dt, vehicle_local_position, v_att);

		if (_vehicle_control_mode.flag_control_position_enabled) {
			// set failsafe setpoint if there hasn't been a new
			// trajectory setpoint since position control started
			if ((_setpoint.timestamp < _time_position_control_enabled)
			    && (vehicle_local_position.timestamp_sample > _time_position_control_enabled)) {
				PX4_INFO("Setpoint time: %f, Vehicle local pos time: %f, Pos Control Enabled time: %f",
					 (double)_setpoint.timestamp, (double)vehicle_local_position.timestamp_sample,
					 (double)_time_position_control_enabled);
				_setpoint = generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, false);
			}
		}

		if (_vehicle_control_mode.flag_control_position_enabled
		    && (_setpoint.timestamp >= _time_position_control_enabled)) {
			_control.setThrustLimit(1.4f);
			_control.setVelocityLimits(0.2f);


			_control.setInputSetpoint(_setpoint);

			_control.setState(states, v_att, angular_velocity);

			// Run position control
			if (!_control.update(dt)) {
				_control.setInputSetpoint(generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, true));
				_control.update(dt);
			}

			// Publish attitude setpoint output
			vehicle_attitude_setpoint_s attitude_setpoint{};
			_control.getAttitudeSetpoint(attitude_setpoint);
			attitude_setpoint.timestamp = hrt_absolute_time();
			_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

			// publish setpoint
			publishLocalPositionSetpoint(attitude_setpoint);
		}
	}

	perf_end(_cycle_perf);
}

void SpacecraftModelPredictiveControl::publishLocalPositionSetpoint(vehicle_attitude_setpoint_s &_att_sp)
{
	// complete the setpoint data structure
	vehicle_local_position_setpoint_s local_position_setpoint{};
	local_position_setpoint.timestamp = hrt_absolute_time();

	local_position_setpoint.x = _setpoint.position[0];
	local_position_setpoint.y = _setpoint.position[1];
	local_position_setpoint.z = _setpoint.position[2];
	local_position_setpoint.yaw = NAN;
	local_position_setpoint.yawspeed = NAN;
	local_position_setpoint.vx = _setpoint.velocity[0];
	local_position_setpoint.vy = _setpoint.velocity[1];
	local_position_setpoint.vz = _setpoint.velocity[2];
	local_position_setpoint.acceleration[0] = _setpoint.acceleration[0];
	local_position_setpoint.acceleration[1] = _setpoint.acceleration[1];
	local_position_setpoint.acceleration[2] = _setpoint.acceleration[2];
	local_position_setpoint.thrust[0] = _att_sp.thrust_body[0];
	local_position_setpoint.thrust[1] = _att_sp.thrust_body[1];
	local_position_setpoint.thrust[2] = _att_sp.thrust_body[2];
	_local_pos_sp_pub.publish(local_position_setpoint);
}

void SpacecraftModelPredictiveControl::poll_manual_setpoint(const float dt,
		const vehicle_local_position_s &vehicle_local_position,
		const vehicle_attitude_s &_vehicle_att)
{
	if (_vehicle_control_mode.flag_control_manual_enabled && _vehicle_control_mode.flag_armed) {
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			if (!_vehicle_control_mode.flag_control_climb_rate_enabled &&
			    !_vehicle_control_mode.flag_control_offboard_enabled) {

				if (_vehicle_control_mode.flag_control_attitude_enabled &&
					_vehicle_control_mode.flag_control_position_enabled) {
					// We are in Stabilized mode
					// Generate position setpoints
					if (!stabilized_pos_sp_initialized) {
						// Initialize position setpoint
						target_pos_sp = Vector3f(vehicle_local_position.x, vehicle_local_position.y,
									 vehicle_local_position.z);

						const float vehicle_yaw = Eulerf(Quatf(_vehicle_att.q)).psi();
						_manual_yaw_sp = vehicle_yaw;
						stabilized_pos_sp_initialized = true;
					}

					// Update velocity setpoint
					Vector3f target_vel_sp = Vector3f(_manual_control_setpoint.pitch, _manual_control_setpoint.roll, 0.0);
					target_pos_sp = target_pos_sp + target_vel_sp * dt;

					// Update _setpoint
					_setpoint.position[0] = target_pos_sp(0);
					_setpoint.position[1] = target_pos_sp(1);
					_setpoint.position[2] = target_pos_sp(2);

					_setpoint.velocity[0] = target_vel_sp(0);
					_setpoint.velocity[1] = target_vel_sp(1);
					_setpoint.velocity[2] = target_vel_sp(2);

					// Generate attitude setpoints
					float yaw_sp_move_rate = 0.0;

					if (_manual_control_setpoint.throttle > -0.9f) {
						yaw_sp_move_rate = _manual_control_setpoint.yaw * yaw_rate;
					}

					_manual_yaw_sp = wrap_pi(_manual_yaw_sp + yaw_sp_move_rate * dt);
					const float roll_body = 0.0;
					const float pitch_body = 0.0;

					Quatf q_sp(Eulerf(roll_body, pitch_body, _manual_yaw_sp));
					q_sp.copyTo(_setpoint.attitude);

					_setpoint.timestamp = hrt_absolute_time();

				} else {
					// We are in Manual mode
					stabilized_pos_sp_initialized = false;
				}

			} else {
				stabilized_pos_sp_initialized = false;
			}

			_manual_setpoint_last_called = hrt_absolute_time();
		}
	}
}

trajectory_setpoint_s SpacecraftModelPredictiveControl::generateFailsafeSetpoint(const hrt_abstime &now,
		const SpacecraftMPCStates &states, bool warn)
{
	// rate limit the warnings
	warn = warn && (now - _last_warn) > 2_s;

	if (warn) {
		PX4_WARN("invalid setpoints");
		_last_warn = now;
	}

	trajectory_setpoint_s failsafe_setpoint = SpacecraftMPC::empty_trajectory_setpoint;
	failsafe_setpoint.timestamp = now;

	failsafe_setpoint.velocity[0] = failsafe_setpoint.velocity[1] = failsafe_setpoint.velocity[2] = 0.f;

	if (warn) {
		PX4_WARN("Failsafe: stop and wait");
	}

	return failsafe_setpoint;
}

int SpacecraftModelPredictiveControl::task_spawn(int argc, char *argv[])
{
	SpacecraftModelPredictiveControl *instance = new SpacecraftModelPredictiveControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int SpacecraftModelPredictiveControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SpacecraftModelPredictiveControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
	### Description
	The controller has two loops: a P loop for position error and a PID loop for velocity error.
	Output of the velocity controller is thrust vector in the body frame, and the same target attitude
	received on the trajectory setpoint as quaternion.

	The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
	logging.
	)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sc_mpc_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sc_mpc_control_main(int argc, char *argv[])
{
	return SpacecraftModelPredictiveControl::main(argc, argv);
}
