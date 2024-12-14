#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    Copyright (c) 2022-2023 PX4 Development Team
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
    3. Neither the name PX4 nor the names of its contributors may be
    used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

File: derivation.py
Description:
    Derivation of an error-state EKF based on
    Sola, Joan. "Quaternion kinematics for the error-state Kalman filter." arXiv preprint arXiv:1711.02508 (2017).
    The derivation is directly done in discrete-time as this allows us to define the desired type of discretization
    for each element while defining the equations (easier than a continuous-time derivation followed by a block-wise discretization).
"""

import argparse

import symforce
symforce.set_epsilon_to_symbol()

import symforce.symbolic as sf
from symforce import typing as T

# Initialize parser
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action='store_true', help="interactively ask model parameters")

# Read arguments from command line
args = parser.parse_args()

if args.interactive:
    # from default_params import *
    print("Enter the system mass: ")
    mass = float(input())
    print("Enter the system inertia: ")
    inertia = float(input())
    print("Enter the maximum force for each actuator: ")
    max_force = float(input())
else:
    print("Using default parameters for system model")
    #from default_params import *

def derive_and_linearize_dynamics():
    # Define symbolic variables for state, reference state, and input
    p = sf.V3.symbolic("p")       # Position
    v = sf.V3.symbolic("v")       # Velocity
    yaw = sf.Symbol("yaw")        # Yaw angle
    w = sf.Symbol("w")            # Angular velocity
    x = sf.Matrix(8, 1, [p[0], p[1], p[2], v[0], v[1], v[2], yaw, w])

    p_ref = sf.V3.symbolic("p_ref")
    v_ref = sf.V3.symbolic("v_ref")
    yaw_ref = sf.Symbol("yaw_ref")
    w_ref = sf.Symbol("w_ref")
    x_ref = sf.Matrix(8, 1, [p_ref[0], p_ref[1], p_ref[2], v_ref[0], v_ref[1], v_ref[2], yaw_ref, w_ref])

    F = sf.V3.symbolic("F")       # Force input
    t = sf.Symbol("t")            # Torque input
    u = sf.Matrix(4, 1, [F[0], F[1], F[2], t])             # Input vector

    #z = sf.Symbol("z")            # Scalar moment of inertia
    z = 1.0
    m = 1.0

    # Rotation matrix as a function of yaw
    R = sf.M33.eye()
    R[0, 0] = sf.cos(yaw)
    R[0, 1] = -sf.sin(yaw)
    R[1, 0] = sf.sin(yaw)
    R[1, 1] = sf.cos(yaw)

    # Define dynamics
    p_dot = v
    v_dot = R.T * F / m
    yaw_dot = w
    w_dot = (1 / z) * t

    f = sf.Matrix(8, 1, [
        p_dot[0], p_dot[1], p_dot[2],
        v_dot[0], v_dot[1], v_dot[2],
        yaw_dot,
        w_dot
    ])

    # Define the error dynamics: e = x - x_ref
    e = x - x_ref

    # Error dynamics function
    e_dot = f.subs({
            p[0]: e[0], p[1]: e[1], p[2]: e[2],
            v[0]: e[3], v[1]: e[4], v[2]: e[5],
            yaw: e[6], w: e[7],
            p_ref[0]: 0, p_ref[1]: 0, p_ref[2]: 0,
            v_ref[0]: 0, v_ref[1]: 0, v_ref[2]: 0,
            yaw_ref: 0, w_ref: 0
	})
    f_error = e_dot

    # Linearize error dynamics around e = 0 and u = 0
    A = f_error.jacobian(e).subs({
        p_ref[0]: 0, p_ref[1]: 0, p_ref[2]: 0,
        v_ref[0]: 0, v_ref[1]: 0, v_ref[2]: 0, yaw: 0,
        yaw_ref: 0, w_ref: 0, F[0]: 0, F[1]: 0, F[2]: 0, t: 0
    })

    B = f_error.jacobian(u).subs({
        p_ref[0]: 0, p_ref[1]: 0, p_ref[2]: 0,
        v_ref[0]: 0, v_ref[1]: 0, v_ref[2]: 0,
        yaw_ref: 0, w_ref: 0, F[0]: 0, F[1]: 0, F[2]: 0, t: 0
    })

    print("Linearized A matrix:")
    print(A)
    print("\nLinearized B matrix:")
    print(B)

    return A, B

def generate_header_file(A, B, file_name="model.h"):
    with open(file_name, "w") as header_file:

        header_file.write("// Linearized A matrix\n")
        header_file.write("static sfloat A_data[NSTATES*NSTATES] = {\n")
        for i in range(A.shape[0]):
    	    header_file.write("   " + ", ".join([str(A[i, j]) for j in range(A.shape[1])]) + ",\n")
        header_file.write("};\n\n")

        header_file.write("// Linearized B matrix\n")
        header_file.write("static sfloat B_data[NSTATES*NINPUTS] = {\n")
        for i in range(B.shape[0]):
    	    header_file.write("   " + ", ".join([str(B[i, j]) for j in range(B.shape[1])]) + ",\n")
        header_file.write("};\n\n")

# Call the function to derive and linearize the dynamics
A, B = derive_and_linearize_dynamics()

# Generate the header file
generate_header_file(A, B)

