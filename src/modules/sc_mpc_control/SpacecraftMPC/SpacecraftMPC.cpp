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
 * @file SpacecraftMPC.cpp
 */

#include "SpacecraftMPC.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <lib/tinympc/tinympc.h>
#include <iostream>


// TinyMPC variables
// Macro variables
#define DT 0.002f    // dt
#define NSTATES 12   // no. of states (error state)
#define NINPUTS 4    // no. of controls
#define NHORIZON 20   // horizon steps (NHORIZON states and NHORIZON-1 controls)

#include "params/params_500hz.h"
// #include "params/traj_fig8.h"

/* Allocate global variables for MPC */
static float f_data[NSTATES] = {0};

// Create data array, all zero initialization
static float x0_data[NSTATES] = {0.0f};       // initial state
static float xg_data[NSTATES] = {0.0f};       // goal state (if not tracking)
static float ug_data[NINPUTS] = {0.0f};       // goal input
// static float Xref_data[NSTATES * NHORIZON] = {0};
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
static tiny_Model mpc_model;
static tiny_AdmmSettings stgs;
static tiny_AdmmData mpc_data;
static tiny_AdmmInfo mpc_info;
static tiny_AdmmSolution mpc_soln;
static tiny_AdmmWorkspace mpc_work;

const trajectory_setpoint_s SpacecraftMPC::empty_trajectory_setpoint  = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

SpacecraftMPC::SpacecraftMPC(){
	// Initialize MPC
	tiny_InitModel(&mpc_model, NSTATES, NINPUTS, NHORIZON, 0, 0, DT);
	tiny_InitSettings(&stgs);

	stgs.rho_init = 250.0f;  // IMPORTANT (select offline, associated with precomp.)

	tiny_InitWorkspace(&mpc_work, &mpc_info, &mpc_model, &mpc_data, &mpc_soln, &stgs);

	// Fill in the remaining struct
	tiny_InitWorkspaceTempData(&mpc_work, ZU, ZU_new, 0, 0, temp_data);
	tiny_InitPrimalCache(&mpc_work, Quu_inv_data, AmBKt_data, coeff_d2p_data);

	tiny_InitModelFromArray(&mpc_model, &A, &B, &f, A_data, B_data, f_data);
	tiny_InitSolnTrajFromArray(&mpc_work, X, U, X_data, U_data);
	tiny_InitSolnGainsFromArray(&mpc_work, d_, p, d_data, p_data, Kinf_data, Pinf_data);
	tiny_InitSolnDualsFromArray(&mpc_work, 0, YU, 0, YU_data, 0);

	tiny_SetInitialState(&mpc_work, x0_data);
	tiny_SetGoalReference(&mpc_work, Xref, Uref, xg_data, ug_data);

	// Set up LQR cost
	tiny_InitDataQuadCostFromArray(&mpc_work, Q_data, R_data);
	slap_AddIdentity(mpc_data.R, mpc_work.rho); // \tilde{R}
	tiny_InitDataLinearCostFromArray(&mpc_work, q_, r, r_tilde, q_data, r_data, r_tilde_data);

	// Set up constraints
	tiny_SetInputBound(&mpc_work, Acu_data, umin_data, umax_data);
	slap_SetConst(mpc_data.ucu, (_lim_thr_max));   // UPPER CONTROL BOUND
	slap_SetConst(mpc_data.lcu, (_lim_thr_min));  // LOWER CONTROL BOUND

	// Initialize linear cost (for tracking)
	tiny_UpdateLinearCost(&mpc_work);

	// Solver settings
	stgs.en_cstr_goal = 0;
	stgs.en_cstr_inputs = 1;
	stgs.en_cstr_states = 0;
	stgs.max_iter = 6;           // limit this if needed
	stgs.verbose = 0;
	stgs.check_termination = 2;
	stgs.tol_abs_dual = 5e-2;
	stgs.tol_abs_prim = 5e-2;

	// Initialize enabled control components
	_pos_en = 0.0f;
	_att_en = 0.0f;
	_omg_en = 0.0f;
}



void SpacecraftMPC::setVelocityLimits(const float vel_max)
{

}

void SpacecraftMPC::setThrustLimit(const float max)
{

}

void SpacecraftMPC::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = matrix::Vector3f(setpoint.position);
	_vel_sp = matrix::Vector3f(setpoint.velocity);
	_att_sp = matrix::Quatf(setpoint.attitude);
	_ang_vel_sp = matrix::Vector3f(setpoint.angular_velocity);
}

bool SpacecraftMPC::update(const float dt)
{
	bool valid = _inputValid();

	if (valid) {
		// Set starting state (x0)
		x0_data[0] = _pos(0);
		x0_data[1] = _pos(1);
		x0_data[2] = _pos(2);
		// Body velocity error, [m/s]
		x0_data[6] = _vel(0);
		x0_data[7] = _vel(1);
		x0_data[8] = _vel(2);
		// Angular rate error, [rad/s]
		x0_data[9]  = _ang_vel(0);
		x0_data[10] = _ang_vel(1);
		x0_data[11] = _ang_vel(2);
		// Attitude error
		matrix::Eulerf q_to_euler(_att);
		x0_data[3] = q_to_euler.phi();
		x0_data[4] = q_to_euler.theta();
		x0_data[5] = q_to_euler.psi();

		// Set goal state (xg)
		xg_data[0] = _pos_sp(0);
		xg_data[1] = _pos_sp(1);
		xg_data[2] = _pos_sp(2);

		matrix::Eulerf q_to_euler_sp(_att_sp);
		xg_data[3] = q_to_euler_sp.phi();
		xg_data[4] = q_to_euler_sp.theta();
		xg_data[5] = q_to_euler_sp.psi();

		xg_data[6] = _vel_sp(0);
		xg_data[7] = _vel_sp(1);
		xg_data[8] = _vel_sp(2);

		// prepare for future angular velocity references
		xg_data[9] = PX4_ISFINITE(_ang_vel_sp(0)) ? _ang_vel_sp(0) : 0.f;
		xg_data[10] = PX4_ISFINITE(_ang_vel_sp(1)) ? _ang_vel_sp(1) : 0.f;
		xg_data[11] = PX4_ISFINITE(_ang_vel_sp(2)) ? _ang_vel_sp(2) : 0.f;

		tiny_SetGoalReference(&mpc_work, Xref, Uref, xg_data, ug_data);
		std::cout << "X Goal: \n" << xg_data[0] << " " << xg_data[1] << " " << xg_data[2] << " \n"
			<< xg_data[3] << " " << xg_data[4] << " " << xg_data[5] << " \n"
			<< xg_data[6] << " " << xg_data[7] << " " << xg_data[8] << " \n"
			<< xg_data[9] << " " << xg_data[10] << " " << xg_data[11] << std::endl;
		std::cout << "X0: \n" << x0_data[0] << " " << x0_data[1] << " " << x0_data[2] << " \n"
			<< x0_data[3] << " " << x0_data[4] << " " << x0_data[5] << " \n"
			<< x0_data[6] << " " << x0_data[7] << " " << x0_data[8] << " \n"
			<< x0_data[9] << " " << x0_data[10] << " " << x0_data[11] << std::endl;

		/* MPC solve */
		tiny_UpdateLinearCost(&mpc_work);
		tiny_SolveAdmm(&mpc_work);

		std::cout << "Status: " << mpc_info.status_val << " | Iteration: " << mpc_info.iter << std::endl;
		std::cout << "Inputs to Att Ctl: " << U[0].data[0] << " " << X[1].data[3] << " " << X[1].data[4]  << " " << X[1].data[5] << std::endl;
		// TODO(@Pedro-Roque, @sschoedel): Needs to be changed based on updated model - here I'm just filling garbage
		_thr_sp(0) = _thr_sp(1) = U[0].data[0];
		_thr_sp(2) = U[0].data[0];
		_att_goal = matrix::Quatf(matrix::Eulerf(X[1].data[3], X[1].data[4], X[1].data[5]));

	}

	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	return valid && _thr_sp.isAllFinite();
}

bool SpacecraftMPC::_inputValid()
{
	bool valid = true;

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1))) == PX4_ISFINITE(_pos_sp(2));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1))) == PX4_ISFINITE(_vel_sp(2));

	// For each controlled state the estimate has to be valid
	valid = valid && PX4_ISFINITE(_att_sp(0)) && PX4_ISFINITE(_att_sp(1)) && PX4_ISFINITE(_att_sp(2)) && PX4_ISFINITE(_att_sp(3));

	// Ensure that estimates are also valid
	valid = valid && PX4_ISFINITE(_pos(0)) && PX4_ISFINITE(_pos(1)) && PX4_ISFINITE(_pos(2));
	valid = valid && PX4_ISFINITE(_vel(0)) && PX4_ISFINITE(_vel(1)) && PX4_ISFINITE(_vel(2));
	valid = valid && PX4_ISFINITE(_ang_vel(0)) && PX4_ISFINITE(_ang_vel(1)) && PX4_ISFINITE(_ang_vel(2));
	valid = valid && PX4_ISFINITE(_att(0)) && PX4_ISFINITE(_att(1)) && PX4_ISFINITE(_att(2)) && PX4_ISFINITE(_att(3));

	return valid;
}

void SpacecraftMPC::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = X[1].data[5];
	local_position_setpoint.yawspeed = X[1].data[11];
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void SpacecraftMPC::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	// Set thrust setpoint
	attitude_setpoint.thrust_body[0] = _thr_sp(0);
	attitude_setpoint.thrust_body[1] = _thr_sp(1);
	attitude_setpoint.thrust_body[2] = _thr_sp(2);

	// Bypass attitude control by giving same attitude setpoint to att control
	if (PX4_ISFINITE(_att_goal(0)) && PX4_ISFINITE(_att_goal(1)) && PX4_ISFINITE(_att_goal(2)) && PX4_ISFINITE(_att_goal(3))) {
		attitude_setpoint.q_d[0] = _att_goal(0);
		attitude_setpoint.q_d[1] = _att_goal(1);
		attitude_setpoint.q_d[2] = _att_goal(2);
		attitude_setpoint.q_d[3] = _att_goal(3);
	} else {
		attitude_setpoint.q_d[0] = _att(0);
		attitude_setpoint.q_d[1] = _att(1);
		attitude_setpoint.q_d[2] = _att(2);
		attitude_setpoint.q_d[3] = _att(3);
	}
}
