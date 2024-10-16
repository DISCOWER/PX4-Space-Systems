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

/**
 * @file sc_mpc_control_params.c
 * Parameters for spacecraft attitude controller.
 *
 * @author Pedro Roque <padr@kth.se>
 */

/**
 * MPC weight on position error along X direction
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_PX, 30.0f);

/**
 * MPC weight on position error along Y direction
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_PY, 30.0f);

/**
 * MPC weight on position error along Z direction
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_PZ, 30.0f);

/**
 * MPC weight on velocity error along X direction
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_VX, 3.0f);

/**
 * MPC weight on velocity error along Y direction
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_VY, 3.0f);

/**
 * MPC weight on velocity error along Z direction
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_VZ, 3.0f);

/**
 * MPC weight on attitude quaternion error on X component
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_QX, 10.0f);

/**
 * MPC weight on attitude quaternion error on Y component
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_QY, 10.0f);

/**
 * MPC weight on attitude quaternion error on Z component
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_QZ, 10.0f);

/**
 * MPC weight on attitude quaternion error on W component
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_QW, 10.0f);

/**
 * MPC weight on angular velocity error along X direction
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_WX, 5.0f);

/**
 * MPC weight on angular velocity error along Y direction
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_WY, 5.0f);

/**
 * MPC weight on angular velocity error along Z direction
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_WZ, 5.0f);

/**
 * MPC weight on force input along X direction
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_UFX, 10.0f);

/**
 * MPC weight on force input along Y direction
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_UFY, 10.0f);

/**
 * MPC weight on force input along Z direction
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_UFZ, 10.0f);

/**
 * MPC weight on torque input along X direction
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_UTX, 10.0f);

/**
 * MPC weight on torque input along Y direction
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_UTY, 10.0f);

/**
 * MPC weight on torque input along Z direction
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Spacecraft MPC Control
 */
PARAM_DEFINE_FLOAT(SC_MPC_W_UTZ, 10.0f);
