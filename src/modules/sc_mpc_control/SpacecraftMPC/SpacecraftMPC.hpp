/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file SpacecraftMPC.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>

/**
 * 	Core Position-Control for MC.
 * 	This class contains P-controller for position and
 * 	PID-controller for velocity.
 * 	Inputs:
 * 		vehicle position/velocity/yaw
 * 		desired set-point position/velocity/thrust/yaw/yaw-speed
 * 		constraints that are stricter than global limits
 * 	Output
 * 		thrust vector and a yaw-setpoint
 *
 * 	If there is a position and a velocity set-point present, then
 * 	the velocity set-point is used as feed-forward. If feed-forward is
 * 	active, then the velocity component of the P-controller output has
 * 	priority over the feed-forward component.
 *
 * 	A setpoint that is NAN is considered as not set.
 * 	If there is a position/velocity- and thrust-setpoint present, then
 *  the thrust-setpoint is ommitted and recomputed from position-velocity-PID-loop.
 */
class SpacecraftMPC
{
public:

	SpacecraftMPC();
	~SpacecraftMPC() = default;

	/**
	 * Set the position control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 */
	void setControlWeights(const matrix::Vector3f &P, const matrix::Vector3f &V,
			const matrix::Vector3f &ATT, const matrix::Vector3f &OMG,
			const matrix::Vector3f &F, const matrix::Vector3f &T)
	{
		_weight_pos = P;
		_weight_vel = V;
		_weight_att = ATT;
		_weight_omg = OMG;
		_weight_force = F;
		_weight_torque = T;
	}

	/**
	 * Set the weight matrices based on defined gains and enabled control components
	 */
	void setWeightMatrices();

	/**
	 * Set the maximum velocity to execute with feed forward and position control
	 * @param vel_max maximum velocity
	 */
	void setVelocityLimits(const float vel_max);

	/**
	 * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
	 * @param min minimum thrust e.g. 0.1 or 0
	 * @param max maximum thrust e.g. 0.9 or 1
	 */
	void setThrustLimit(const float max);

	/**
	 * Pass the current vehicle state to the controller
	 * @param SpacecraftMPCStates structure
	 */

	template <typename T> void setState(const T &states, vehicle_attitude_s &att, vehicle_angular_velocity_s &ang_vel)
	{
		_pos = states.position;
		_vel = states.velocity;
		_att = matrix::Quatf(att.q);
		_ang_vel(0) = ang_vel.xyz[0];
		_ang_vel(1) = ang_vel.xyz[1];
		_ang_vel(2) = ang_vel.xyz[2];
	}

	/**
	 * Pass the desired setpoints
	 * Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
	 * @param setpoint setpoints including feed-forwards to execute in update()
	 */
	void setInputSetpoint(const trajectory_setpoint_s &setpoint);

	/**
	 * Apply P-position and PID-velocity controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt time in seconds since last iteration
	 * @return true if update succeeded and output setpoint is executable, false if not
	 */
	bool update(const float dt);

	/**
	 * Get the controllers output local position setpoint
	 * These setpoints are the ones which were executed on including PID output and feed-forward.
	 * The acceleration or thrust setpoints can be used for attitude control.
	 * @param local_position_setpoint reference to struct to fill up
	 */
	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

	/**
	 * Get the controllers output attitude setpoint
	 * This attitude setpoint was generated from the resulting acceleration setpoint after position and velocity control.
	 * It needs to be executed by the attitude controller to achieve velocity and position tracking.
	 * @param attitude_setpoint reference to struct to fill up
	 */
	void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;

	/**
	 * All setpoints are set to NAN (uncontrolled). Timestampt zero.
	 */
	static const trajectory_setpoint_s empty_trajectory_setpoint;

private:

	bool _inputValid();

	// Gains
	matrix::Vector3f _weight_pos; ///< Position weight
	matrix::Vector3f _weight_vel; ///< Velocity weight
	matrix::Vector3f _weight_att; ///< Attitude weight
	matrix::Vector3f _weight_omg; ///< Angular velocity (omega) mpc weight

	// Multipliers - enable/disable subcomponents of the controller
	float _pos_en; ///< Position control enabled
	float _att_en; ///< Attitude control enabled
	float _omg_en; ///< Angular velocity control enabled

	matrix::Vector3f _weight_force; ///< Force mpc weight
	matrix::Vector3f _weight_torque; ///< Torque mpc weight

	// States
	matrix::Vector3f _pos; /**< current position */
	matrix::Vector3f _vel; /**< current velocity */
	matrix::Quatf _att; /**< current attitude */
	matrix::Vector3f _ang_vel; /**< current angular velocity */

	// Setpoints
	matrix::Vector3f _pos_sp; /**< desired position */
	matrix::Vector3f _vel_sp; /**< desired velocity */
	matrix::Quatf _att_sp; /**< desired heading */
	matrix::Vector3f _ang_vel_sp; /** desired yaw-speed */

	// Outputs
	matrix::Vector3f _thr_sp; /**< desired thrust */
	matrix::Vector3f _trq_sp; /**< desired torque */
};
