// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <map>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>

#include "dynamixel_hardware/visiblity_control.h"
#include "rclcpp/macros.hpp"

using hardware_interface::return_type;

namespace dynamixel_hardware
{
struct JointValues
{
  constexpr static auto quiet_NaN = std::numeric_limits<double>::quiet_NaN();
  double position{quiet_NaN};
  double velocity{quiet_NaN};
  double effort{quiet_NaN};
};

struct Joint
{
  Joint(uint8_t id) : id{id} {}
  uint8_t id;
  bool inverted{false};
  JointValues state{};
  JointValues command{};
  JointValues prev_command{};
  // Mode roue protocole 1 : position integree depuis la vitesse, et regulateur PI de vitesse
  double velocity_integral{0.0};
  double velocity_output{0.0};
  // Zone morte du capteur (60 degres sur 360) : lectures de position et de vitesse fausses
  int32_t last_raw_position{-1};
  bool in_dead_zone{false};
  int valid_streak{0};
  int stuck_streak{0};
  // Derniere lecture valide du capteur, pour recaler la position a la sortie de la zone morte
  bool reference_valid{false};
  bool reference_trusted{false};
  int32_t reference_raw{0};
  double reference_angle{0.0};
  double reference_position{0.0};
  // Vitesse calculee depuis le capteur de position, filtree
  bool previous_sample_valid{false};
  int32_t trust_progress{0};
  bool dead_zone_needs_edge{false};
  double filtered_velocity{0.0};
};

// Gains du regulateur PI de vitesse (mode roue protocole 1)
struct VelocityPI
{
  bool enabled{true};
  double kp{1.0};
  double ki{4.0};
  double integral_limit{3.0};  // rad/s
};

enum class ControlMode { none, position, velocity };

class DynamixelHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelHardware)

  DYNAMIXEL_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  void set_joints_info();
  bool is_stub_used();
  bool init_dynamixel_workbench();
  bool load_dynamixels();
  bool init_dynamixels();
  bool init_control_items();
  bool add_sdk_handler();
  bool enable_torque();
  bool disable_torque();
  bool set_position_control_mode();
  bool set_velocity_control_mode();
  bool restore_torque(bool torque_was_enabled);
  void reset_command();
  bool set_up_all_dynamixels_components();
  void set_all_states_when_stub_is_used();
  uint16_t control_items_data_length();
  bool read_current_states(
    uint8_t id, uint16_t data_length, std::vector<uint32_t> & position_velocity_current);
  JointValues convert_joint_values(
    uint8_t id, const std::vector<uint32_t> & position_velocity_current);
  std::vector<uint8_t> get_ids() const;
  bool is_joint_interface(const std::string & interface_name) const;
  bool has_position_command_changed() const;
  bool has_velocity_command_changed() const;
  std::vector<int32_t> get_velocity_commands();
  int32_t convert_velocity_command(uint8_t id, double velocity);
  int32_t convert_protocol1_moving_speed_command(uint8_t id, double velocity);
  bool uses_protocol1_moving_speed_velocity();
  hardware_interface::return_type write_velocity_commands(
    std::vector<uint8_t> ids, std::vector<int32_t> commands);
  std::vector<int32_t> get_position_commands();
  hardware_interface::return_type write_position_commands(
    std::vector<uint8_t> ids, std::vector<int32_t> commands);
  void set_command_to_position();
  bool set_control_item(const char * control_item_name);
  bool set_control_item(const char * control_item_name, const char * secondary_control_item_name);
  void log_current_joint_position();
  bool uses_wheel_mode() const;
  void read_velocity_pi_parameters();
  void update_velocity_outputs(double dt);
  bool update_dead_zone(
    Joint & joint, int32_t raw_position, int32_t raw_speed, double velocity, double dt);

  ControlMode control_mode_{};
  std::vector<Joint> joints_info_;
  bool use_stub_{};
  bool torque_enabled_{};
  bool protocol1_wheel_mode_{};
  VelocityPI velocity_pi_{};
  DynamixelWorkbench dynamixel_workbench_;
  std::map<const char * const, const ControlItem *> control_items_;
};

}  // namespace dynamixel_hardware
