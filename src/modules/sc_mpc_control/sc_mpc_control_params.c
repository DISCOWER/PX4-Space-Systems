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
 * MPC Enable
 *
 *
 * @min 0
 * @max 1
 * @group Spacecraft MPC
 */
PARAM_DEFINE_INT32(SPC_MPC_ENABLED, 0);

/**
 * MPC Weight on position error along X axis
 *
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_Q_POS_X, 100.0f);

/**
 * MPC Weight on position error along Y axis
 *
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_Q_POS_Y, 100.0f);

/**
 * MPC Weight on position error along Z axis
 *
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_Q_POS_Z, 100.0f);

/**
 * MPC Weight on velocity error along X axis
 *
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_Q_VEL_X, 50.0f);

/**
 * MPC Weight on velocity error along Y axis
 *
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_Q_VEL_Y, 50.0f);

/**
 * MPC Weight on velocity error along Z axis
 *
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_Q_VEL_Z, 50.0f);

/**
 * MPC Weight on attitude error along X axis
 *
 * If quaternion, acts on X component
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_Q_ATT_X, 300.0f);

/**
 * MPC Weight on attitude error along Y axis
 *
 * If quaternion, acts on Y component
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_Q_ATT_Y, 300.0f);

/**
 * MPC Weight on attitude error along Z axis
 *
 * If quaternion, acts on Z component
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_Q_ATT_Z, 300.0f);

/**
 * MPC Weight on quaternion attitude error along W axis (scale)
 *
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_Q_ATT_W, 300.0f);

/**
 * MPC Weight on angular velocity error along X axis
 *
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_Q_OMG_X, 100.0f);

/**
 * MPC Weight on angular velocity error along Y axis
 *
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_Q_OMG_Y, 100.0f);


/**
 * MPC Weight on angular velocity error along Z axis
 *
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_Q_OMG_Z, 100.0f);

/**
 * MPC Weight on force input along X axis
 *
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_R_F_X, 20.0f);

/**
 * MPC Weight on force input along Y axis
 *
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_R_F_Y, 20.0f);

/**
 * MPC Weight on force input along Z axis
 *
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_R_F_Z, 20.0f);

/**
 * MPC Weight on torque input along X axis
 *
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_R_T_X, 20.0f);

/**
 * MPC Weight on torque input along Y axis
 *
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_R_T_Y, 20.0f);

/**
 * MPC Weight on torque input along Z axis
 *
 *
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1.0
 * @group Spacecraft MPC
 */
PARAM_DEFINE_FLOAT(SPC_MPC_R_T_Z, 20.0f);
