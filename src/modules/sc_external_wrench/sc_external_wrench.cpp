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

#include "sc_external_wrench.hpp"

#include <px4_platform_common/getopt.h>

#include <uORB/topics/parameter_update.h>

int ScExternalWrench::print_status()
{
	PX4_INFO("Running...");
	PX4_INFO("Enabled: %s", _enabled ? "yes" : "no");
	PX4_INFO("Timed out: %s", _timeout ? "yes" : "no");
	return 0;
}

int ScExternalWrench::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	return print_usage("unknown command");
}

int ScExternalWrench::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 1024, (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

ScExternalWrench *ScExternalWrench::instantiate(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	bool error_flag = false;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case '?':
			error_flag = true;
			break;

		case 'f':
			update_Frequency = atoi(myoptarg);
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

	ScExternalWrench *instance = new ScExternalWrench();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}


	return instance;
}

ScExternalWrench::ScExternalWrench() : ModuleParams(nullptr) {}

void ScExternalWrench::run()
{
	// initialize parameters
	parameters_update(true);

	// initialize subscriptions to default values
	_vehicle_external_thrust.xyz[0] = 0.0f;
	_vehicle_external_thrust.xyz[1] = 0.0f;
	_vehicle_external_thrust.xyz[2] = 0.0f;
	_vehicle_external_thrust.timestamp = 0;

	_vehicle_external_torque.xyz[0] = 0.0f;
	_vehicle_external_torque.xyz[1] = 0.0f;
	_vehicle_external_torque.xyz[2] = 0.0f;
	_vehicle_external_torque.timestamp = 0;

	_timeout = true;
	bool thrust_timeout = true;
	bool torque_timeout = true;

	while (!should_exit()) {
		// reset timeout flags
		thrust_timeout = true;
		thrust_timeout = true;

		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if(!_vehicle_control_mode.flag_control_allocation_enabled) {
			// if control allocation is not enabled, do not run
			_timeout = true;
			_enabled = false;
			continue;
		}
		_enabled = true;

		// check for updates on the external wrench
		vehicle_thrust_setpoint_s wrench_thrust;
		vehicle_torque_setpoint_s wrench_torque;
		if (_vehicle_external_thrust_sub.update(&wrench_thrust)) {
			// check timestamp
			if (hrt_elapsed_time(&wrench_thrust.timestamp) < 1.0 / (float)_update_frequency) {
				_vehicle_external_thrust = wrench_thrust;
				thrust_timeout = false;
			}
		}

		if (_vehicle_external_torque_sub.update(&wrench_torque)) {
			// check timestamp
			if (hrt_elapsed_time(&wrench_torque.timestamp) < 1.0 / (float)_update_frequency) {
				_vehicle_external_torque = wrench_torque;
				torque_timeout = false;
			}
		}

		// check for timeout
		_timeout = thrust_timeout || torque_timeout;
		if(!_timeout)
		{
			_vehicle_external_thrust_allocation_pub.publish(_vehicle_external_thrust);
			_vehicle_external_torque_allocation_pub.publish(_vehicle_external_torque);
		}
		else
		{
			// reset wrench
			_vehicle_external_thrust.xyz[0] = 0.0f;
			_vehicle_external_thrust.xyz[1] = 0.0f;
			_vehicle_external_thrust.xyz[2] = 0.0f;
			_vehicle_external_thrust.timestamp = 0;

			_vehicle_external_torque.xyz[0] = 0.0f;
			_vehicle_external_torque.xyz[1] = 0.0f;
			_vehicle_external_torque.xyz[2] = 0.0f;
			_vehicle_external_torque.timestamp = 0;
		}

		// check for parameter updates
		parameters_update();
	}
}

void ScExternalWrench::parameters_update(bool force)
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

int ScExternalWrench::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module provides access to receiving an external wrench that is applied to the vehicle
on different actuation channels than the main control stack. Provides capability to apply
external disturbances to the control scheme or bypass the control stack of the vehicle.

This module is active by receiving a wrench message at the starting specified rate, and
using a sc_control_allocator-enabled flight mode.

### Usage
CLI usage:
$ sc_external_wrench start -f 100

)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("sc_external_wrench", "template");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_PARAM_INT('f', 100, 0, 1000, "Target frequency of external wrench.", true);
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

int sc_external_wrench_main(int argc, char* argv[]) { return ScExternalWrench::main(argc, argv); }
