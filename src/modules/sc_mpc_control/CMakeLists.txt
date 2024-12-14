############################################################################
#
#   Copyright (c) 2015-2020 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

add_subdirectory(SpacecraftMPC)


option(SC_MPC_GENERATE_MODEL "spacecraft tinympc generate symforce output" ON)

# Symforce code generation - check if symforce is available
execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -m symforce.symbolic
    RESULT_VARIABLE PYTHON_SYMFORCE_EXIT_CODE
    OUTPUT_QUIET
)

set(TINYMPC_MODEL_GEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SpacecraftMPC/params)
set(TINYMPC_MODEL_GEN_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SpacecraftMPC/params)

if(SC_MPC_GENERATE_MODEL AND (${PYTHON_SYMFORCE_EXIT_CODE} EQUAL 0))
	# generate to build directory
	set(MPC_DERIVATION_DST_DIR ${CMAKE_CURRENT_BINARY_DIR}/SpacecraftMPC/params)
	file(MAKE_DIRECTORY ${MPC_DERIVATION_DST_DIR})

	set(MPC_GENERATED_FILES ${MPC_DERIVATION_DST_DIR}/model.h)
	set(MPC_GENERATED_DERIVATION_INCLUDE_PATH ${CMAKE_CURRENT_BINARY_DIR})

	set(SYMFORCE_ARGS)

	# if(NOT CONFIG_SC_MPC_GENERATE_INTERACTIVE)
	# 	message(STATUS "sc_mpc: interactive generation")
	# 	list(APPEND SYMFORCE_ARGS "--interactive")
	# endif()
	message("Calling derivation.py ...")
	add_custom_command(
		OUTPUT
			${TINYMPC_MODEL_GEN_OUTPUT_DIR}/model.h
		COMMAND
			${PYTHON_EXECUTABLE} ${TINYMPC_MODEL_GEN_DIR}/derivation.py #--interactive # ${SYMFORCE_ARGS}

		WORKING_DIRECTORY ${TINYMPC_MODEL_GEN_DIR}
		COMMENT "Symforce code generation (default)"
		USES_TERMINAL
		VERBATIM
	)
endif()

px4_add_module(
	MODULE modules__sc_mpc_control
	MAIN sc_mpc_control
	COMPILE_FLAGS
	SRCS
		SpacecraftModelPredictiveControl.cpp
		SpacecraftModelPredictiveControl.hpp
	DEPENDS
		SpacecraftMPC
		geo
		SlewRate
	)
