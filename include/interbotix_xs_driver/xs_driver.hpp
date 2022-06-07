// Copyright 2022 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef INTERBOTIX_XS_DRIVER__XS_DRIVER_HPP_
#define INTERBOTIX_XS_DRIVER__XS_DRIVER_HPP_

#include <interbotix_xs_driver/version.hpp>
#include <interbotix_xs_driver/xs_common.hpp>
#include <interbotix_xs_driver/xs_logging.hpp>

#include <math.h>

#include <string>
#include <bitset>
#include <vector>
#include <algorithm>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "yaml-cpp/yaml.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

namespace interbotix_xs
{
// Interbotix Core Class to build any type of Dynamixel-based robot
class InterbotixDriverXS
{
public:
  /// @brief Constructor for the InterbotixDriverXS
  /// @param filepath_motor_configs absolute filepah to the motor configuration file
  /// @param filepath_mode_configs absolute filepah to the modes configuration file
  /// @param write_eeprom_on_startup whether or not to write configs to EEPROM on startup
  /// @throws runtime_error if constructor was not initialized properly due to bad configuration
  ///   file, inability to open specified serial port, inability to ping all specified motors, or
  ///   inability to write startup configuration to all motors' EEPROMs
  explicit InterbotixDriverXS(
    std::string filepath_motor_configs,
    std::string filepath_mode_configs,
    bool write_eeprom_on_startup);

  /// @brief Destructor for the InterbotixDriverXS
  ~InterbotixDriverXS();

  /// @brief Set the operating mode for a specific group of motors or a single motor
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if changing the operating mode for a group of motors
  ///   or 'CMD_TYPE_SINGLE' if changing the operating mode for a single motor
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param mode desired operating mode (either 'position', 'linear_position', 'ext_position',
  ///   'velocity', 'pwm', 'current', or 'current_based_position')
  /// @param profile_type set to 'velocity' for a Velocity-based Profile or 'time' for a Time-based
  ///   Profile (modifies Bit 2 of the 'Drive_Mode' register)
  /// @param profile_velocity passthrough to the Profile_Velocity register on the motor
  /// @param profile_acceleration passthrough to the Profile_Acceleration register on the motor
  bool robot_set_operating_modes(
    const std::string cmd_type,
    const std::string & name,
    const std::string & mode,
    const std::string profile_type = DEFAULT_PROF_TYPE,
    const int32_t profile_velocity = DEFAULT_PROF_VEL,
    const int32_t profile_acceleration = DEFAULT_PROF_ACC);

  /// @brief Helper function used to set the operating mode for a single motor
  /// @param name desired motor name
  /// @param mode desired operating mode (either 'position', 'linear_position', 'ext_position',
  ///   'velocity', 'pwm', 'current', or 'current_based_position')
  /// @param profile_type set to 'velocity' for a Velocity-based Profile or 'time' for a Time-based
  ///   Profile (modifies Bit 2 of the 'Drive_Mode' register)
  /// @param profile_velocity passthrough to the Profile_Velocity register on the motor
  /// @param profile_acceleration passthrough to the Profile_Acceleration register on the motor
  bool robot_set_joint_operating_mode(
    const std::string & name,
    const std::string & mode,
    const std::string profile_type = DEFAULT_PROF_TYPE,
    const int32_t profile_velocity = DEFAULT_PROF_VEL,
    const int32_t profile_acceleration = DEFAULT_PROF_ACC);

  /// @brief Torque On/Off a specific group of motors or a single motor
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if torquing off a group of motors or
  ///   'CMD_TYPE_SINGLE' if torquing off a single motor
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param enable set to True to torque on or False to torque off
  bool robot_torque_enable(
    const std::string cmd_type,
    const std::string & name,
    const bool & enable);

  /// @brief Reboot a specific group of motors or a single motor
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if rebooting a group of motors or 'CMD_TYPE_SINGLE'
  ///   if rebooting a single motor
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param torque_enable set to True to torque on or False to torque off after rebooting
  /// @param smart_reboot set to True to only reboot motor(s) in a specified group that have gone
  ///   into an error state
  bool robot_reboot_motors(
    const std::string cmd_type,
    const std::string & name,
    const bool & enable,
    const bool & smart_reboot);

  /// @brief Command a desired group of motors with the specified commands
  /// @param name desired motor group name
  /// @param commands vector of commands (order matches the order specified in the 'groups'
  ///   section in the motor config file)
  /// @details commands are processed differently based on the operating mode specified for the
  ///   motor group
  bool robot_write_commands(
    const std::string & name,
    std::vector<float> commands);

  /// @brief Command a desired motor with the specified command
  /// @param name desired motor name
  /// @param command motor command
  /// @details the command is processed differently based on the operating mode specified for the
  ///   motor
  bool robot_write_joint_command(
    const std::string & name,
    float command);

  /// @brief Set motor firmware PID gains
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if changing the PID gains for a group of motors or
  ///   'CMD_TYPE_SINGLE' if changing the PID gains for a single motor
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param gains vector containing the desired PID gains - order is as shown in the function
  bool robot_set_motor_pid_gains(
    const std::string cmd_type,
    const std::string & name,
    const std::vector<int32_t> & gains);

  /// @brief Set a register value to multiple motors
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if setting register values for a group of motors or
  ///   'CMD_TYPE_SINGLE' if setting a single register value
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param value desired register value
  bool robot_set_motor_registers(
    const std::string cmd_type,
    const std::string & name,
    const std::string & reg,
    const int32_t & value);

  /// @brief Get a register value from multiple motors
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if getting register values from a group of motors or
  ///   'CMD_TYPE_SINGLE' if getting a single register value
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param values [out] vector of register values
  bool robot_get_motor_registers(
    const std::string cmd_type,
    const std::string & name,
    const std::string & reg,
    std::vector<int32_t> & values);

  /// @brief Get states for a group of joints
  /// @param name desired joint group name
  /// @param positions [out] vector of current joint positions [rad]
  /// @param velocities [out] vector of current joint velocities [rad/s]
  /// @param effort [out] vector of current joint effort [mA]
  bool robot_get_joint_states(
    const std::string & name,
    std::vector<float> * positions = NULL,
    std::vector<float> * velocities = NULL,
    std::vector<float> * effort = NULL);

  /// @brief Get states for a group of joints
  /// @param name desired joint group name
  /// @param positions [out] vector of current joint positions [rad]
  /// @param velocities [out] vector of current joint velocities [rad/s]
  /// @param effort [out] vector of current joint effort [mA]
  bool robot_get_joint_states(
    const std::string & name,
    std::vector<double> * positions = NULL,
    std::vector<double> * velocities = NULL,
    std::vector<double> * effort = NULL);

  /// @brief Get states for a single joint
  /// @param name desired joint name
  /// @param position [out] current joint position [rad]
  /// @param velocity [out] current joint velocity [rad/s]
  /// @param effort [out] current joint effort [mA]
  bool robot_get_joint_state(
    const std::string & name,
    float * position = NULL,
    float * velocity = NULL,
    float * effort = NULL);

  /// @brief Converts linear distance between two gripper fingers into angular position
  /// @param name name of the gripper servo to command
  /// @param linear_position desired distance [m] between the two gripper fingers
  /// @param <float> [out] angular position [rad] that achieves the desired linear distance
  float robot_convert_linear_position_to_radian(
    const std::string & name,
    const float & linear_position);

  /// @brief Converts angular position into the linear distance from one gripper finger to the
  ///   center of the gripper servo horn
  /// @param name name of the gripper sevo to command
  /// @param angular_position desired gripper angular position [rad]
  /// @param <float> [out] linear position [m] from a gripper finger to the center of the gripper
  ///   servo horn
  float robot_convert_angular_position_to_linear(
    const std::string & name,
    const float & angular_position);

  /// @brief Reads current states from all the motors
  void robot_read_joint_states();

  void get_joint_names(std::vector<std::string> & names);

  float get_sleep_position(const std::string & name);

  std::unordered_map<std::string, JointGroup> * get_group_map();

  std::unordered_map<std::string, MotorState> * get_motor_map();

  JointGroup * get_group_info(const std::string & name);

  MotorState * get_motor_info(const std::string & name);

  Gripper * get_gripper_info(const std::string & name);

  std::vector<std::string> * get_gripper_order();

  bool is_motor_gripper(const std::string & name);

private:
  // Boolean that changes value when a JointTrajectoryCommand begins and ends execution
  bool execute_joint_traj;

  // Pointer to the 'all' group (makes updating joint states more efficient)
  JointGroup * all_ptr;

  // DynamixelWorkbench object used to easily communicate with any Dynamixel servo
  DynamixelWorkbench dxl_wb;

  // Holds all the information in the motor_configs YAML file
  YAML::Node motor_configs;

  // Holds all the information in the mode_configs YAML file
  YAML::Node mode_configs;

  // Inficating whether to set up relative time.
  bool set_up_start_time;

  // Trajectory counter
  size_t cntr;

  // Holds the USB port name that connects to the U2D2
  std::string port = DEFAULT_PORT;

  // Vector containing all the desired EEPROM register values to command the motors at startup
  std::vector<MotorRegVal> motor_info_vec;

  // Vector containing the order in which multiple grippers (if present) are published in the
  // JointState message
  std::vector<std::string> gripper_order;

  // Dictionary mapping register names to information about them (like 'address' and expected 'data
  // length')
  std::unordered_map<std::string, const ControlItem *> control_items;

  // Dictionary mapping a joint's name with its 'sleep' position
  std::unordered_map<std::string, float> sleep_map;

  // Dictionary mapping a joint group's name with information about it (as defined in the
  // JointGroup struct)
  std::unordered_map<std::string, JointGroup> group_map;

  // Dictionary mapping a motor's name with information about it (as defined in the MotorState
  // struct)
  std::unordered_map<std::string, MotorState> motor_map;

  // Dictionary mapping a motor's name with the names of its shadows - including itself
  std::unordered_map<std::string, std::vector<std::string>> shadow_map;

  // Dictionary mapping the name of either servo in a 2in1 motor with the other one (henceforth
  // known as 'sister')
  std::unordered_map<std::string, std::vector<std::string>> sister_map;

  // Dictionary mapping the name of a gripper motor with information about it (as defined in the
  // Gripper struct)
  std::unordered_map<std::string, Gripper> gripper_map;

  // Dictionary mapping the name of a joint with its position in the JointState 'name' list
  std::unordered_map<std::string, size_t> js_index_map;

  std::vector<float> robot_positions;
  std::vector<float> robot_velocities;
  std::vector<float> robot_efforts;

  std::string filepath_motor_configs;
  std::string filepath_mode_configs;

  bool write_eeprom_on_startup;

  // Mutex for updating / getting joint states
  std::mutex _mutex_js;

  /// @brief Loads a robot-specific 'motor_configs' yaml file and populates class variables with
  ///   its contents
  bool robot_retrieve_motor_configs(
    std::string filepath_motor_configs,
    std::string filepath_mode_configs);

  /// @brief Initializes the port to communicate with the Dynamixel servos
  /// @param <bool> [out] True if the port was successfully opened; False otherwise
  bool robot_init_port();

  /// @brief Pings all motors to make sure they can be found
  /// @param <bool> [out] True if all motors were found; False otherwise
  bool robot_ping_motors();

  /// @brief Writes some 'startup' EEPROM register values to the Dynamixel servos
  /// @param <bool> [out] True if all register values were written successfully; False otherwise
  bool robot_load_motor_configs();

  /// @brief Retrieves information about 'Goal_XXX' and 'Present_XXX' registers
  /// @details Info includes a register's name, address, and data length
  bool robot_init_controlItems();

  /// @brief Creates SyncWrite and SyncRead Handlers to write/read data to multiple motors
  ///   simultaneously
  bool robot_init_workbench_handlers();

  /// @brief Loads a 'mode_configs' yaml file containing desired operating modes and sets up the
  ///   motors accordingly
  void robot_init_operating_modes();
};

}  // namespace interbotix_xs

#endif  // INTERBOTIX_XS_DRIVER__XS_DRIVER_HPP_
