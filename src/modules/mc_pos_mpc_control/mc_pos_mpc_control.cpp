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

#include "mc_pos_mpc_control.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <lib/tinympc/tinympc.h>
#include <px4_platform_common/events.h>
#include <matrix/math.hpp>
#include <iostream>

//using namespace matrix;
using namespace time_literals;
using math::radians;

// TinyMPC variables
// Macro variables
#define DT 0.002f    // dt
#define NSTATES 12   // no. of states (error state)
#define NINPUTS 4    // no. of controls
#define NHORIZON 5   // horizon steps (NHORIZON states and NHORIZON-1 controls)

#define PUBLISH_RATE 1 // 1 for true, 0 for false. If false, publishes Torque and Thrust setpoints at 500Hz

#include "params/params_500hz.h"
#include "params/traj_fig8.h"

/* Allocate global variables for MPC */
static float f_data[NSTATES] = {0};

// Create data array, all zero initialization
static float x0_data[NSTATES] = {0.0f};       // initial state
//static float xg_data[NSTATES] = {0.0f};       // goal state (if not tracking)
static float ug_data[NINPUTS] = {0.0f};       // goal input
static float Xref_data[NSTATES * NHORIZON] = {0};
static float X_data[NSTATES * NHORIZON] = {0.0f};        // X in MPC solve
static float U_data[NINPUTS * (NHORIZON - 1)] = {0.0f};  // U in MPC solve
static float d_data[NINPUTS * (NHORIZON - 1)] = {0.0f};
static float p_data[NSTATES * NHORIZON] = {0.0f};
static float q_data[NSTATES*(NHORIZON-1)] = {0.0f};
static float r_data[NINPUTS*(NHORIZON-1)] = {0.0f};
static float r_tilde_data[NINPUTS*(NHORIZON-1)] = {0.0f};
static float Acu_data[NINPUTS * NINPUTS] = {0.0f};
static float YU_data[NINPUTS * (NHORIZON - 1)] = {0.0f};
static float umin_data[NINPUTS] = {0.0f};
static float umax_data[NINPUTS] = {0.0f};
static float temp_data[NINPUTS + 2*NINPUTS*(NHORIZON - 1)] = {0.0f};

// Created matrices
static Matrix Xref[NHORIZON];
static Matrix Uref[NHORIZON - 1];
static Matrix X[NHORIZON];
static Matrix U[NHORIZON - 1];
static Matrix d_[NHORIZON - 1];
static Matrix p[NHORIZON];
static Matrix YU[NHORIZON - 1];
static Matrix ZU[NHORIZON - 1];
static Matrix ZU_new[NHORIZON - 1];
static Matrix q_[NHORIZON-1];
static Matrix r[NHORIZON-1];
static Matrix r_tilde[NHORIZON-1];
static Matrix A;
static Matrix B;
static Matrix f;

//Create TinyMPC struct
static tiny_Model model;
static tiny_AdmmSettings stgs;
static tiny_AdmmData data;
static tiny_AdmmInfo info;
static tiny_AdmmSolution soln;
static tiny_AdmmWorkspace work;

// Helper variables
bool isInit = false;  // fix for tracking problem
uint32_t mpcTime = 0;
float u_hover = 0.67f;
int8_t result = 0;
uint32_t step = 0;
uint32_t traj_length = T_ARRAY_SIZE(X_ref_data) / 3;
uint32_t traj_idx = 0;
int8_t traj_hold = 1;  // hold current trajectory for this no of steps
bool en_traj = false;

MulticopterPosMPControl::MulticopterPosMPControl()
	: ModuleParams(nullptr),
	  WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	  _vehicle_torque_setpoint_pub(ORB_ID(vehicle_torque_setpoint)),
	  _vehicle_thrust_setpoint_pub(ORB_ID(vehicle_thrust_setpoint)),
	  _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_SPACECRAFT;
	parameters_updated();
	_controller_status_pub.advertise();


	// Initialize MPC
	tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, DT);
	tiny_InitSettings(&stgs);

	stgs.rho_init = 250.0f;  // IMPORTANT (select offline, associated with precomp.)

	tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);

	// Fill in the remaining struct
	tiny_InitWorkspaceTempData(&work, ZU, ZU_new, 0, 0, temp_data);
	tiny_InitPrimalCache(&work, Quu_inv_data, AmBKt_data, coeff_d2p_data);

	tiny_InitModelFromArray(&model, &A, &B, &f, A_data, B_data, f_data);
	tiny_InitSolnTrajFromArray(&work, X, U, X_data, U_data);
	tiny_InitSolnGainsFromArray(&work, d_, p, d_data, p_data, Kinf_data, Pinf_data);
	tiny_InitSolnDualsFromArray(&work, 0, YU, 0, YU_data, 0);

	tiny_SetInitialState(&work, x0_data);
	// tiny_SetGoalReference(&work, Xref, Uref, xg_data, ug_data);

	data.Xref = Xref;
	data.Uref = Uref;
	for (int i = 0; i < NHORIZON; ++i) {
		if (i < NHORIZON - 1) {
			Uref[i] = slap_MatrixFromArray(NINPUTS, 1, ug_data);
		}
			Xref[i] = slap_MatrixFromArray(NSTATES, 1, &X_ref_data[i * NSTATES]);
	}

	// Set up LQR cost
	tiny_InitDataQuadCostFromArray(&work, Q_data, R_data);
	slap_AddIdentity(data.R, work.rho); // \tilde{R}
	tiny_InitDataLinearCostFromArray(&work, q_, r, r_tilde, q_data, r_data, r_tilde_data);

	// Set up constraints
	tiny_SetInputBound(&work, Acu_data, umin_data, umax_data);
	slap_SetConst(data.ucu, (1 - u_hover));   // UPPER CONTROL BOUND
	slap_SetConst(data.lcu, (-u_hover));  // LOWER CONTROL BOUND

	// Initialize linear cost (for tracking)
	tiny_UpdateLinearCost(&work);

	// Solver settings
	stgs.en_cstr_goal = 0;
	stgs.en_cstr_inputs = 1;
	stgs.en_cstr_states = 0;
	stgs.max_iter = 6;           // limit this if needed
	stgs.verbose = 0;
	stgs.check_termination = 2;
	stgs.tol_abs_dual = 5e-2;
	stgs.tol_abs_prim = 5e-2;

	/* End of MPC initialization */
	step = 0;
	en_traj = true;
}

MulticopterPosMPControl::~MulticopterPosMPControl() { perf_free(_loop_perf); }

bool MulticopterPosMPControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void MulticopterPosMPControl::parameters_updated()
{
	// Set Position gains
	Q_data[0 + NSTATES * 0] = _param_sc_mpc_w_px.get();
	Q_data[1 + NSTATES * 1] = _param_sc_mpc_w_py.get();
	Q_data[2 + NSTATES * 2] = _param_sc_mpc_w_pz.get();


	// Set Attitude gains
	Q_data[3 + NSTATES * 3] = _param_sc_mpc_w_qx.get();
	Q_data[4 + NSTATES * 4] = _param_sc_mpc_w_qy.get();
	Q_data[5 + NSTATES * 5] = _param_sc_mpc_w_qz.get();
	// If using quaternions:  Q_data[i + NSTATES * i] = _param_sc_mpc_w_q	w.get();

	// Set Velocity gains
	Q_data[6 + NSTATES * 6] = _param_sc_mpc_w_vx.get();
	Q_data[7 + NSTATES * 7] = _param_sc_mpc_w_vy.get();
	Q_data[8 + NSTATES * 8] = _param_sc_mpc_w_vz.get();


	// Set Angular rate gains
	Q_data[9 + NSTATES * 9] = _param_sc_mpc_w_wx.get();
	Q_data[10 + NSTATES * 10] = _param_sc_mpc_w_wy.get();
	Q_data[11 + NSTATES * 11] = _param_sc_mpc_w_wz.get();

	// Set Input gains
	R_data[0 + NINPUTS * 0] = _param_sc_mpc_w_ufz.get();
	R_data[1 + NINPUTS * 1] = _param_sc_mpc_w_utx.get();
	R_data[2 + NINPUTS * 2] = _param_sc_mpc_w_uty.get();
	R_data[3 + NINPUTS * 3] = _param_sc_mpc_w_utz.get();
}

void MulticopterPosMPControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	// track time
	hrt_abstime timestamp = hrt_absolute_time();

	// check if vehicle is armed
	vehicle_local_position_s vehicle_local_position;
	vehicle_attitude_s v_att;
	vehicle_angular_velocity_s angular_velocity;

	if (_local_pos_sub.update(&vehicle_local_position)) {
		// const float dt =
		// 	math::constrain(((vehicle_local_position.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = vehicle_local_position.timestamp_sample;

		if (_vehicle_control_mode_sub.updated()) {
			const bool previous_position_control_enabled = _vehicle_control_mode.flag_control_position_enabled;

			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				if (!previous_position_control_enabled && _vehicle_control_mode.flag_control_position_enabled) {
					_time_position_control_enabled = _vehicle_control_mode.timestamp;

				} else if (previous_position_control_enabled && !_vehicle_control_mode.flag_control_position_enabled) {
					// clear existing setpoint when controller is no longer active
					// _setpoint = ScPositionControl::empty_trajectory_setpoint;
				}
			}
		}

		// Update setpoints, attitude and angular velocity
		_trajectory_setpoint_sub.update(&_setpoint);
		_vehicle_attitude_sub.update(&v_att);
		_vehicle_angular_velocity_sub.update(&angular_velocity);

		/* run controller on gyro changes */
		if (step % traj_hold == 0 && en_traj == true) {
		traj_idx = (int)(step / traj_hold);
			for (int i = 0; i < NHORIZON; ++i) {
				for (int j = 0; j < 3; ++j) {
					Xref_data[i*NSTATES + j] = X_ref_data[(traj_idx + i)*3+j];
				}
			}
		}

		// Set starting state (x0)
		x0_data[0] = vehicle_local_position.x;
		x0_data[1] = vehicle_local_position.y;
		x0_data[2] = vehicle_local_position.z;
		// Body velocity error, [m/s]
		x0_data[6] = vehicle_local_position.vx;
		x0_data[7] = vehicle_local_position.vy;
		x0_data[8] = vehicle_local_position.vz;
		// Angular rate error, [rad/s]
		x0_data[9]  = angular_velocity.xyz[0];
		x0_data[10] = angular_velocity.xyz[1];
		x0_data[11] = angular_velocity.xyz[2];
		// Attitude error
		matrix::Quatf quat_from_lib = matrix::Quatf(v_att.q[0], v_att.q[1], v_att.q[2], v_att.q[3]);
		matrix::Eulerf q_to_euler(quat_from_lib);
		x0_data[3] = q_to_euler.phi();
		x0_data[4] = q_to_euler.theta();
		x0_data[5] = q_to_euler.psi();


		/* MPC solve */
		tiny_UpdateLinearCost(&work);
		tiny_SolveAdmm(&work);
		hrt_abstime elapsed_time = hrt_absolute_time() - timestamp;
		std::cout << "Step: " << step << " | Status: " << info.status_val << " | Iteration: " << info.iter << " | Time:" << elapsed_time << std::endl;
		std::cout << "Inputs: " << U[0].data[0] + u_hover << " " << U[0].data[1] + u_hover  << " " << U[0].data[2] + u_hover  << "  "<< U[0].data[3] + u_hover  << std::endl;

		// publish rate setpoint
		if(PUBLISH_RATE){
			vehicle_rates_setpoint_s rates_setpoint{};

			rates_setpoint.roll = X[1].data[1];
			rates_setpoint.pitch = X[1].data[2];
			rates_setpoint.yaw = X[1].data[3];
			rates_setpoint.thrust_body[2] = U[0].data[0] + u_hover;
			rates_setpoint.timestamp = hrt_absolute_time();
			_vehicle_rates_setpoint_pub.publish(rates_setpoint);
		}else{
			// publish thrust and torque setpoints
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			// fill setpoint
			vehicle_thrust_setpoint.xyz[2] = U[0].data[0] + u_hover;
			vehicle_torque_setpoint.xyz[0] = U[0].data[1] + u_hover;
			vehicle_torque_setpoint.xyz[1] = U[0].data[2] + u_hover;
			vehicle_torque_setpoint.xyz[2] = U[0].data[3] + u_hover;

			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);
		}

		// stop trajectory at the end
		if (traj_idx >= traj_length - NHORIZON) {
			en_traj = false;
		}
		else step += 1;
	}

	perf_end(_loop_perf);
}

int MulticopterPosMPControl::task_spawn(int argc, char *argv[])
{

	MulticopterPosMPControl *instance = new MulticopterPosMPControl();

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

int MulticopterPosMPControl::custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }

int MulticopterPosMPControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the Spacecraft MPC controller. It takes position setpoints as inputs
and outputs torque and thrust setpoints.

)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("mc_pos_mpc_control", "controller");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

extern "C" __EXPORT int mc_pos_mpc_control_main(int argc, char* argv[]) {
  return MulticopterPosMPControl::main(argc, argv);
}
