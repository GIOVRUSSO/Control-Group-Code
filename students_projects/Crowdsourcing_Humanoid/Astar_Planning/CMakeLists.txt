cmake_minimum_required(VERSION 3.16)

set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang++")

# INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
# Default to GLVND if available.
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)

# This line has to appear before 'PROJECT' in order to be able to disable incremental linking
set(MSVC_INCREMENTAL_DEFAULT ON)

set(Python3_FIND_VIRTUALENV ONLY)
set(Python3_FIND_STRATEGY LOCATION)

project(
  data_driven_legged_locomotion
  VERSION 0.1.0
  DESCRIPTION "Crowdsourcing algorithms for legged locomotion"
)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
message("Using Python3 executable:${Python3_EXECUTABLE}")
message("Python3 version:${Python3_VERSION}")

include(ExternalProject)
ExternalProject_Add(
  mujoco_mpc
  GIT_REPOSITORY https://github.com/AntoSave/mujoco_mpc.git
  GIT_TAG main
)

ExternalProject_Get_Property(mujoco_mpc BINARY_DIR SOURCE_DIR)
message("Building protobuf files with: ${Python3_EXECUTABLE} setup.py install -B ${BINARY_DIR}")
message("working directory: ${SOURCE_DIR}/python")
execute_process(
  COMMAND ${Python3_EXECUTABLE} setup.py install -B ${BINARY_DIR}
  WORKING_DIRECTORY ${SOURCE_DIR}/python
)