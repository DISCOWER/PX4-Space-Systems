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
/**
 * @file sc_external_wrench.c
 * Controls extra, more precise actuators for external disturbances.
 *
 * @author Pedro Roque <padr@kth.se>
 */
#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <stdio.h>
#include <math.h>
#include <poll.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/mavlink_log.h>

using namespace time_literals;

extern "C" __EXPORT int sc_external_wrench_main(int argc, char *argv[]);

class ScExternalWrench : public ModuleBase<ScExternalWrench>, public ModuleParams
{
public:
	ScExternalWrench();

	virtual ~ScExternalWrench() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static ScExternalWrench *instantiate(int argc, char *argv[]);

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

	DEFINE_PARAMETERS((ParamInt<px4::params::SYS_AUTOSTART>)_param_sys_autostart,  /**< example parameter */
			  (ParamInt<px4::params::SYS_AUTOCONFIG>)_param_sys_autoconfig /**< another parameter */
			 )

	// Flags
	static bool _timeout;
	static uint update_frequency;
	bool _enabled = false;

	// Subscriptions
	uORB::SubscriptionInterval 	_parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription 		_vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	vehicle_control_mode_s          _vehicle_control_mode {};

	uORB::Subscription _vehicle_external_thrust_sub{ORB_ID(vehicle_external_wrench_thrust)};
	uORB::Subscription _vehicle_external_torque_sub{ORB_ID(vehicle_external_wrench_torque)};
	vehicle_thrust_setpoint_s _vehicle_external_thrust {};
	vehicle_torque_setpoint_s _vehicle_external_torque {};

	uORB::Publication<vehicle_thrust_setpoint_s>  _vehicle_external_thrust_allocation_pub{ORB_ID(vehicle_external_wrench_thrust_allocation)};
	uORB::Publication<vehicle_torque_setpoint_s>  _vehicle_external_torque_allocation_pub{ORB_ID(vehicle_external_wrench_torque_allocation)};
};

bool ScExternalWrench::_timeout = true;
uint ScExternalWrench::update_frequency = 100;
