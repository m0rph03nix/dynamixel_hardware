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

#include "dynamixel_hardware/dynamixel_hardware.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)

namespace
{
constexpr const char * NAME_OF_HARDWARE_INTERFACE = "Dynamixel Hardware";
constexpr const char * GOAL_POSITION = "Goal_Position";
constexpr const char * GOAL_VELOCITY = "Goal_Velocity";
constexpr const char * MOVING_SPEED = "Moving_Speed";
constexpr const char * PRESENT_POSITION = "Present_Position";
constexpr const char * PRESENT_VELOCITY = "Present_Velocity";
constexpr const char * PRESENT_CURRENT = "Present_Current";
constexpr const char * PRESENT_SPEED = "Present_Speed";
constexpr const char * PRESENT_LOAD = "Present_Load";
constexpr float PROTOCOL_1 = 1.0f;
constexpr float RPM_TO_RAD_PER_SEC = 0.104719755f;
constexpr int32_t PROTOCOL1_MOVING_SPEED_MAX = 1023;
constexpr int32_t PROTOCOL1_MOVING_SPEED_CW_OFFSET = 1024;
// Au-dela, une vitesse lue est une trame corrompue (AX-12A : environ 6 rad/s a vide)
constexpr double MAX_PLAUSIBLE_VELOCITY = 10.0;
// Mode roue protocole 1 : le capteur de position couvre 0-1023 sur 300 degres. Dans les 60 degres
// restants, la position lue vaut 0, 1023 ou une valeur figee, et la vitesse lue est fausse.
constexpr int32_t RAW_POSITION_MIN_VALID = 4;
constexpr int32_t RAW_POSITION_MAX_VALID = 1019;
constexpr int32_t RAW_POSITION_MAX_STEP = 40;    // ~12 degres par cycle, bien au-dela de 6 rad/s a 50 Hz
constexpr int32_t RAW_SPEED_MAX = 2047;
constexpr int VALID_SAMPLES_TO_LEAVE_DEAD_ZONE = 3;
// Au-dela de 0,6 rad/s, une roue qui tourne avance d'au moins 2 pas de capteur par cycle :
// une position figee pendant 3 cycles ne peut venir que de la zone morte.
constexpr double STUCK_MIN_COMMAND = 0.6;        // rad/s
constexpr int STUCK_SAMPLES_TO_ENTER_DEAD_ZONE = 3;
constexpr double STUCK_PROGRESS_RATIO = 0.4;     // avance minimale par cycle / avance attendue
constexpr int32_t RAW_POSITION_EXIT_MARGIN = 150;  // pas depuis le bord, a la sortie de la zone
constexpr int32_t TRUST_PROGRESS = 20;           // pas d'avance continue pour croire le capteur
constexpr double RAW_TO_RAD = 300.0 / 1023.0 * M_PI / 180.0;  // 1023 pas sur 300 degres
// La vitesse lue (Present_Speed) surestime la vitesse reelle de 5 a 9 % sur l'AX-12A : la vitesse
// est calculee depuis la position, et filtree sur environ 0,1 s (1 pas de capteur = 0,005 rad)
constexpr double VELOCITY_FILTER_TIME_CONSTANT = 0.1;  // s
constexpr double MAX_VELOCITY_ERROR = 2.0;       // rad/s

bool has_interface_type(const std::string & interface_name, const char * interface_type)
{
  const std::string suffix = "/" + std::string(interface_type);
  return interface_name.size() >= suffix.size() and
         interface_name.compare(interface_name.size() - suffix.size(), suffix.size(), suffix) == 0;
}
}  // namespace

namespace dynamixel_hardware
{

hardware_interface::CallbackReturn DynamixelHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  set_joints_info();
  read_velocity_pi_parameters();

  if (is_stub_used() or set_up_all_dynamixels_components()) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  return hardware_interface::CallbackReturn::ERROR;
}

bool DynamixelHardware::set_up_all_dynamixels_components()
{
  if (
    init_dynamixel_workbench() and load_dynamixels() and disable_torque() and init_dynamixels() and
    set_position_control_mode() and enable_torque() and init_control_items() and
    add_sdk_handler()) {
    return true;
  }
  return false;
}

void DynamixelHardware::set_joints_info()
{
  for (const auto & joint : info_.joints) {
    joints_info_.push_back(Joint(std::stoi(joint.parameters.at("id"))));
    auto & j = joints_info_.back();
    auto it = joint.parameters.find("invert");
    if (it != joint.parameters.cend()) {
      std::string val = it->second;
      std::transform(val.begin(), val.end(), val.begin(),
        [](unsigned char c) { return std::tolower(c); });
      j.inverted = (val == "true");
    }
    RCLCPP_INFO(
      rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE),
      "joint_id %d inverted=%s", j.id, j.inverted ? "true" : "false");
  }
}

void DynamixelHardware::read_velocity_pi_parameters()
{
  const auto & params = info_.hardware_parameters;
  auto get = [&](const char * name, double & value) {
      const auto it = params.find(name);
      if (it != params.cend()) {value = std::stod(it->second);}
    };
  const auto it = params.find("velocity_pi");
  if (it != params.cend()) {velocity_pi_.enabled = (it->second == "true");}
  get("velocity_kp", velocity_pi_.kp);
  get("velocity_ki", velocity_pi_.ki);
  get("velocity_integral_limit", velocity_pi_.integral_limit);
}

bool DynamixelHardware::is_stub_used()
{
  if (
    info_.hardware_parameters.find("use_stub") != info_.hardware_parameters.cend() and
    info_.hardware_parameters.at("use_stub") == "true") {
    use_stub_ = true;
    RCLCPP_INFO(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "stub is used");
  }
  return use_stub_;
}

bool DynamixelHardware::init_dynamixel_workbench()
{
  const std::string usb_port = info_.hardware_parameters.at("usb_port");
  const int baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  const char * log{};

  RCLCPP_INFO(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "baud_rate: %d", baud_rate);
  if (not dynamixel_workbench_.init(usb_port.c_str(), baud_rate, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
    return false;
  }
  return true;
}

bool DynamixelHardware::load_dynamixels()
{
  const char * log{};
  for (const auto & joint : joints_info_) {
    uint16_t model_number = 0;

    if (not dynamixel_workbench_.ping(joint.id, &model_number, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
      return false;
    }
  }
  return true;
}

bool DynamixelHardware::init_control_items()
{
  if (
    set_control_item(GOAL_POSITION) and set_control_item(GOAL_VELOCITY, MOVING_SPEED) and
    set_control_item(PRESENT_POSITION) and set_control_item(PRESENT_VELOCITY, PRESENT_SPEED) and
    set_control_item(PRESENT_CURRENT, PRESENT_LOAD)) {
    return true;
  }
  return false;
}

bool DynamixelHardware::set_control_item(const char * control_item_name)
{
  const uint8_t first_joint_id = joints_info_[0].id;
  const ControlItem * control_item =
    dynamixel_workbench_.getItemInfo(first_joint_id, control_item_name);
  if (not control_item) {
    return false;
  }
  control_items_[control_item_name] = control_item;
  return true;
}

bool DynamixelHardware::set_control_item(
  const char * control_item_name, const char * secondary_control_item_name)
{
  const uint8_t first_joint_id = joints_info_[0].id;
  const ControlItem * control_item =
    dynamixel_workbench_.getItemInfo(first_joint_id, control_item_name);
  if (not control_item) {
    control_item = dynamixel_workbench_.getItemInfo(first_joint_id, secondary_control_item_name);
  }
  if (not control_item) {
    return false;
  }
  control_items_[control_item_name] = control_item;
  return true;
}

bool DynamixelHardware::add_sdk_handler()
{
  const char * log{};
  if (not dynamixel_workbench_.addSyncWriteHandler(
        control_items_[GOAL_POSITION]->address, control_items_[GOAL_POSITION]->data_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
    return false;
  };

  if (not dynamixel_workbench_.addSyncWriteHandler(
        control_items_[GOAL_VELOCITY]->address, control_items_[GOAL_VELOCITY]->data_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
    return false;
  }
  return true;
}

bool DynamixelHardware::enable_torque()
{
  const char * log{};
  for (const auto & joint : joints_info_) {
    if (not dynamixel_workbench_.torqueOn(joint.id, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
      return false;
    }
  }
  torque_enabled_ = true;
  return true;
}

bool DynamixelHardware::disable_torque()
{
  const char * log{};
  for (const auto & joint : joints_info_) {
    if (not dynamixel_workbench_.torqueOff(joint.id, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
      return false;
    }
  }
  torque_enabled_ = false;
  return true;
}

bool DynamixelHardware::init_dynamixels()
{
  const char * log{};
  for (const auto & joint_info : info_.joints) {
    const uint8_t id = std::stoi(joint_info.parameters.at("id"));
    for (const auto & [name, value] : joint_info.parameters) {
      if (name != "id" && name != "invert") {
        if (not dynamixel_workbench_.itemWrite(id, name.c_str(), std::stoi(value), &log)) {
          return false;
        }
        RCLCPP_INFO(
rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "Write item with success %s %d", name.c_str(), std::stoi(value));
      }
    }
  }
  return true;
}

bool DynamixelHardware::set_position_control_mode()
{
  const char * log{};
  if (control_mode_ != ControlMode::position) {
    const bool torque_was_enabled = torque_enabled_;
    if (torque_was_enabled and not disable_torque()) {
      return false;
    }
    for (const auto & joint : joints_info_) {
      if (not dynamixel_workbench_.setPositionControlMode(joint.id, &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
        restore_torque(torque_was_enabled);
        return false;
      }
    }
    if (not restore_torque(torque_was_enabled)) {
      return false;
    }
    RCLCPP_INFO(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "Position control");
    control_mode_ = ControlMode::position;
  }
  return true;
}
bool DynamixelHardware::set_velocity_control_mode()
{
  const char * log{};
  if (control_mode_ != ControlMode::velocity) {
    const bool torque_was_enabled = torque_enabled_;
    if (torque_was_enabled and not disable_torque()) {
      return false;
    }
    for (const auto & joint : joints_info_) {
      if (not dynamixel_workbench_.setVelocityControlMode(joint.id, &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
        restore_torque(torque_was_enabled);
        return false;
      }
    }
    if (not restore_torque(torque_was_enabled)) {
      return false;
    }
    control_mode_ = ControlMode::velocity;
    const bool entering_wheel_mode = uses_protocol1_moving_speed_velocity();
    for (auto & joint : joints_info_) {
      joint.velocity_integral = 0.0;
      // En mode roue, la position est integree et donc relative : elle part de 0, comme en
      // simulation. Sinon diff_drive_controller compterait la position absolue du capteur
      // comme un deplacement a sa premiere mise a jour.
      if (entering_wheel_mode) {
        joint.state.position = 0.0;
        joint.last_raw_position = -1;
        joint.in_dead_zone = false;
        joint.stuck_streak = 0;
        joint.reference_valid = false;
        joint.reference_trusted = false;
        joint.previous_sample_valid = false;
        joint.trust_progress = 0;
      }
    }
  }
  protocol1_wheel_mode_ = uses_protocol1_moving_speed_velocity();
  if (protocol1_wheel_mode_ and velocity_pi_.enabled) {
    RCLCPP_INFO(
      rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE),
      "Velocity control (wheel mode, PI kp=%.2f ki=%.2f)", velocity_pi_.kp, velocity_pi_.ki);
  } else {
    RCLCPP_INFO(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "Velocity control");
  }
  return true;
}

bool DynamixelHardware::restore_torque(bool torque_was_enabled)
{
  if (torque_was_enabled) {
    return enable_torque();
  }
  return true;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_info_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_info_[i].state.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_info_[i].state.effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_info_[i].command.position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_info_[i].command.velocity));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DynamixelHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "start");
  set_all_states_when_stub_is_used();

  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  log_current_joint_position();
  reset_command();
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));
  return hardware_interface::CallbackReturn::SUCCESS;
}
void DynamixelHardware::log_current_joint_position()
{
  for (auto & joint : joints_info_) {
    RCLCPP_INFO(
      rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "joint id %d, position %f", joint.id,
      joint.state.position);
  }
}

void DynamixelHardware::set_all_states_when_stub_is_used()
{
  for (auto & joint : joints_info_) {
    if (use_stub_ and std::isnan(joint.state.position)) {
      joint.state = {0.0, 0.0, 0.0};
    }
  }
}

hardware_interface::CallbackReturn DynamixelHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "stop");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DynamixelHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> &)
{
  bool starts_position = false;
  bool starts_velocity = false;

  for (const auto & interface_name : start_interfaces) {
    if (not is_joint_interface(interface_name)) {
      continue;
    }

    if (has_interface_type(interface_name, hardware_interface::HW_IF_POSITION)) {
      starts_position = true;
    } else if (has_interface_type(interface_name, hardware_interface::HW_IF_VELOCITY)) {
      starts_velocity = true;
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "Unsupported command interface: %s",
        interface_name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }

  if (starts_position and starts_velocity) {
    RCLCPP_ERROR(
      rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE),
      "Position and velocity command interfaces cannot be active at the same time");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> &)
{
  for (const auto & interface_name : start_interfaces) {
    if (not is_joint_interface(interface_name)) {
      continue;
    }

    if (has_interface_type(interface_name, hardware_interface::HW_IF_POSITION)) {
      if (use_stub_) {
        control_mode_ = ControlMode::position;
        return hardware_interface::return_type::OK;
      }
      return set_position_control_mode() ? hardware_interface::return_type::OK
                                         : hardware_interface::return_type::ERROR;
    }
    if (has_interface_type(interface_name, hardware_interface::HW_IF_VELOCITY)) {
      if (use_stub_) {
        control_mode_ = ControlMode::velocity;
        return hardware_interface::return_type::OK;
      }
      return set_velocity_control_mode() ? hardware_interface::return_type::OK
                                         : hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (use_stub_) {
    return hardware_interface::return_type::OK;
  }
  const uint16_t data_length = control_items_data_length();
  std::vector<uint32_t> position_velocity_current(data_length);
  const double dt = period.seconds();

  for (auto & joint : joints_info_) {
    if (not read_current_states(joint.id, data_length, position_velocity_current)) {
      return hardware_interface::return_type::ERROR;
    }
    const JointValues previous = joint.state;
    JointValues values = convert_joint_values(joint.id, position_velocity_current);
    if (joint.inverted) {
      values.position = -values.position;
      values.velocity = -values.velocity;
      values.effort   = -values.effort;
    }
    // Trame corrompue : on garde la vitesse precedente
    if (std::abs(values.velocity) > MAX_PLAUSIBLE_VELOCITY and not std::isnan(previous.velocity)) {
      values.velocity = previous.velocity;
    }
    if (uses_wheel_mode() and not std::isnan(previous.position)) {
      const int32_t raw_position =
        DXL_MAKEWORD(position_velocity_current[0], position_velocity_current[1]);
      const int32_t raw_speed =
        DXL_MAKEWORD(position_velocity_current[2], position_velocity_current[3]);
      // Dans la zone morte, la meilleure estimation de la vitesse est la consigne
      const bool dead_zone = update_dead_zone(joint, raw_position, raw_speed, values.velocity, dt);
      if (dead_zone) {
        values.velocity = joint.command.velocity;
        joint.filtered_velocity = values.velocity;
      }
      if (joint.in_dead_zone) {
        joint.previous_sample_valid = false;
      }
      // A l'arret, la vitesse lue n'est fiable que si le capteur l'est
      if (joint.command.velocity == 0.0 and
        (joint.in_dead_zone or not joint.reference_trusted))
      {
        values.velocity = 0.0;
      }
      // Le capteur revient a zero a chaque tour : la position est d'abord integree
      values.position = previous.position + values.velocity * dt;
      if (not joint.in_dead_zone) {
        // Hors zone morte, la position lue est exacte : l'angle parcouru depuis la derniere lecture
        // valide, modulo un tour, recale la position, y compris apres un passage en zone morte.
        // La reference n'est fiable qu'une fois la roue vue en mouvement : a l'arret dans la zone
        // morte, la valeur lue peut etre figee et fausse.
        const double angle = raw_position * RAW_TO_RAD;
        if (joint.reference_valid and joint.reference_trusted) {
          // L'angle lu est connu modulo un tour : on garde le tour le plus proche de la position
          // estimee, qui ne s'ecarte pas d'un demi-tour meme apres une longue zone morte
          const double sign = joint.inverted ? -1.0 : 1.0;
          const double measured = angle - joint.reference_angle;
          const double predicted = sign * (values.position - joint.reference_position);
          const double delta =
            measured + 2.0 * M_PI * std::round((predicted - measured) / (2.0 * M_PI));
          values.position = joint.reference_position + sign * delta;
          if (joint.previous_sample_valid and dt > 0.0) {
            const double alpha = dt / (VELOCITY_FILTER_TIME_CONSTANT + dt);
            joint.filtered_velocity += alpha * ((values.position - joint.reference_position) / dt -
              joint.filtered_velocity);
            values.velocity = joint.filtered_velocity;
          } else {
            joint.filtered_velocity = values.velocity;
          }
          joint.previous_sample_valid = true;
        } else if (joint.reference_valid) {
          // Une valeur figee tremble sans avancer : on attend une avance continue dans un sens
          const int32_t step = raw_position - joint.reference_raw;
          joint.trust_progress = (step == 0 or (step > 0) == (joint.trust_progress > 0)) ?
            joint.trust_progress + step : step;
          if (std::abs(joint.trust_progress) >= TRUST_PROGRESS) {
            joint.reference_trusted = true;
          }
        }
        joint.reference_valid = true;
        joint.reference_raw = raw_position;
        joint.reference_angle = angle;
        joint.reference_position = values.position;
      }
    } else {
      joint.last_raw_position = -1;
      joint.in_dead_zone = false;
    }
    joint.state = values;
  }
  return hardware_interface::return_type::OK;
}

bool DynamixelHardware::update_dead_zone(
  Joint & joint, int32_t raw_position, int32_t raw_speed, double velocity, double dt)
{
  const int32_t step = joint.last_raw_position >= 0 ? raw_position - joint.last_raw_position : 0;
  const bool out_of_range =
    raw_position < RAW_POSITION_MIN_VALID or raw_position > RAW_POSITION_MAX_VALID;
  const bool jump = std::abs(step) > RAW_POSITION_MAX_STEP;
  joint.last_raw_position = raw_position;

  // Sens de variation attendu de la position brute, et pas attendu par cycle
  const double command = joint.command.velocity;
  const int direction = command == 0.0 ? 0 : ((command > 0.0) != joint.inverted ? 1 : -1);
  const double expected_step = std::abs(command) * dt / RAW_TO_RAD;

  // Assez vite pour juger de l'avance a chaque cycle : une valeur figee (qui tremble de quelques
  // pas) n'avance pas comme la consigne le demande
  const bool fast = std::abs(command) > STUCK_MIN_COMMAND and dt > 0.0;
  const bool progressing = not fast or direction * step >= STUCK_PROGRESS_RATIO * expected_step;
  joint.stuck_streak = progressing ? 0 : joint.stuck_streak + 1;
  const bool frozen = joint.stuck_streak >= STUCK_SAMPLES_TO_ENTER_DEAD_ZONE;

  // Une roue suivie par le PI ne s'ecarte jamais autant de sa consigne
  const bool implausible = std::abs(velocity - command) > MAX_VELOCITY_ERROR;

  // A la sortie de la zone morte, la position lue repart du bord correspondant au sens de rotation
  const bool near_exit_edge =
    direction > 0 ? raw_position <= RAW_POSITION_EXIT_MARGIN :
    direction < 0 ? raw_position >= 1023 - RAW_POSITION_EXIT_MARGIN : true;

  const bool bad_reading = out_of_range or jump or raw_speed > RAW_SPEED_MAX or implausible;
  if (bad_reading or frozen) {
    // Lecture aberrante : la roue traverse la zone morte, elle en sortira par un bord.
    // Position figee seule : il suffit que la roue avance de nouveau (demarrage, blocage).
    joint.dead_zone_needs_edge = (joint.in_dead_zone and joint.dead_zone_needs_edge) or bad_reading;
    joint.in_dead_zone = true;
    joint.valid_streak = 0;
  } else if (joint.in_dead_zone) {
    const bool exit_ok = progressing and (near_exit_edge or not joint.dead_zone_needs_edge);
    joint.valid_streak = exit_ok ? joint.valid_streak + 1 : 0;
    if (joint.valid_streak >= VALID_SAMPLES_TO_LEAVE_DEAD_ZONE) {
      joint.in_dead_zone = false;
    }
  }

  return joint.in_dead_zone;
}

bool DynamixelHardware::uses_wheel_mode() const
{
  return control_mode_ == ControlMode::velocity and protocol1_wheel_mode_;
}

JointValues DynamixelHardware::convert_joint_values(
  uint8_t id, const std::vector<uint32_t> & position_velocity_current)
{
  JointValues joint_values;
  joint_values.effort = dynamixel_workbench_.convertValue2Current(
    id, DXL_MAKEWORD(position_velocity_current[4], position_velocity_current[5]));
  joint_values.velocity = dynamixel_workbench_.convertValue2Velocity(
    id, DXL_MAKEWORD(position_velocity_current[2], position_velocity_current[3]));
  joint_values.position = dynamixel_workbench_.convertValue2Radian(
    id, DXL_MAKEWORD(position_velocity_current[0], position_velocity_current[1]));
  RCLCPP_DEBUG(
    rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "joint id %d, position %f", id,
    joint_values.position);
  return joint_values;
}

uint16_t DynamixelHardware::control_items_data_length()
{
  return control_items_[PRESENT_POSITION]->data_length +
         control_items_[PRESENT_VELOCITY]->data_length +
         control_items_[PRESENT_CURRENT]->data_length;
}

bool DynamixelHardware::read_current_states(
  uint8_t id, uint16_t data_length, std::vector<uint32_t> & position_velocity_current)
{
  int nb_attempt = 3;
  const char * log{};

  while(nb_attempt > 0)
  {
    if (dynamixel_workbench_.readRegister(
          id, control_items_[PRESENT_POSITION]->address, data_length,
          position_velocity_current.data(), &log)) {
      return true;
    }

    RCLCPP_WARN(
      rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "Read current values failed! %s (Attempt %d)", log, (4-nb_attempt));
    
    for(int i=0;i<30000;i++) {;}

    nb_attempt--;
  }

  RCLCPP_ERROR(
    rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "Read current values failed! %s", log);

  return false;
}

hardware_interface::return_type DynamixelHardware::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (use_stub_) {
    set_command_to_position();
    for (auto & joint_info : joints_info_) {
      joint_info.prev_command = joint_info.command;
    }
    return hardware_interface::return_type::OK;
  }
  std::vector<uint8_t> ids{get_ids()};
  update_velocity_outputs(period.seconds());

  if (has_velocity_command_changed()) {
    if (not set_velocity_control_mode()) {
      return hardware_interface::return_type::ERROR;
    }
    return write_velocity_commands(ids, get_velocity_commands());
  }

  if (has_position_command_changed()) {
    if (not set_position_control_mode()) {
      return hardware_interface::return_type::ERROR;
    }
    return write_position_commands(ids, get_position_commands());
  }

  if (control_mode_ == ControlMode::velocity) {
    return write_velocity_commands(ids, get_velocity_commands());
  }

  if (control_mode_ == ControlMode::position) {
    return write_position_commands(ids, get_position_commands());
  }

  RCLCPP_ERROR(
    rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "Only position/velocity control are implemented");
  return hardware_interface::return_type::ERROR;
}

std::vector<uint8_t> DynamixelHardware::get_ids() const
{
  std::vector<uint8_t> ids;
  std::transform(
    joints_info_.cbegin(), joints_info_.cend(), std::back_inserter(ids),
    [](const auto & joint_info) { return joint_info.id; });
  return ids;
}

bool DynamixelHardware::is_joint_interface(const std::string & interface_name) const
{
  return std::any_of(info_.joints.cbegin(), info_.joints.cend(), [&](const auto & joint) {
    const std::string prefix = joint.name + "/";
    return interface_name.compare(0, prefix.size(), prefix) == 0;
  });
}

bool DynamixelHardware::has_position_command_changed() const
{
  return std::any_of(joints_info_.cbegin(), joints_info_.cend(), [](const auto & joint_info) {
    return joint_info.command.position != joint_info.prev_command.position;
  });
}

bool DynamixelHardware::has_velocity_command_changed() const
{
  return std::any_of(joints_info_.cbegin(), joints_info_.cend(), [](const auto & joint_info) {
    return joint_info.command.velocity != joint_info.prev_command.velocity;
  });
}

void DynamixelHardware::update_velocity_outputs(double dt)
{
  for (auto & joint : joints_info_) {
    const double command = joint.command.velocity;
    // Le mode roue protocole 1 regle une puissance, pas une vitesse : un PI corrige l'ecart
    if (not (uses_wheel_mode() and velocity_pi_.enabled) or std::isnan(command) or
      std::isnan(joint.state.velocity))
    {
      joint.velocity_output = command;
      continue;
    }
    if (command == 0.0) {
      joint.velocity_integral = 0.0;
      joint.velocity_output = 0.0;
      continue;
    }
    const double error = command - joint.state.velocity;
    if (dt > 0.0 and not joint.in_dead_zone) {
      joint.velocity_integral = std::clamp(
        joint.velocity_integral + velocity_pi_.ki * error * dt,
        -velocity_pi_.integral_limit, velocity_pi_.integral_limit);
    }
    joint.velocity_output = command + velocity_pi_.kp * error + joint.velocity_integral;
  }
}

std::vector<int32_t> DynamixelHardware::get_velocity_commands()
{
  std::vector<int32_t> commands;
  for (const auto & joint_info : joints_info_) {
    const double vel = joint_info.inverted ? -joint_info.velocity_output : joint_info.velocity_output;
    commands.push_back(convert_velocity_command(joint_info.id, vel));
  }
  return commands;
}

int32_t DynamixelHardware::convert_velocity_command(uint8_t id, double velocity)
{
  if (uses_protocol1_moving_speed_velocity()) {
    return convert_protocol1_moving_speed_command(id, velocity);
  }
  return dynamixel_workbench_.convertVelocity2Value(id, static_cast<float>(velocity));
}

int32_t DynamixelHardware::convert_protocol1_moving_speed_command(uint8_t id, double velocity)
{
  if (velocity == 0.0) {
    return 0;
  }

  const ModelInfo * model_info = dynamixel_workbench_.getModelInfo(id);
  if (model_info == nullptr or model_info->rpm == 0.0f) {
    return dynamixel_workbench_.convertVelocity2Value(id, static_cast<float>(velocity));
  }

  // Protocol 1.0 wheel mode stores direction in bit 10 of Moving_Speed.
  int32_t value = static_cast<int32_t>(
    std::abs(velocity) / static_cast<double>(model_info->rpm * RPM_TO_RAD_PER_SEC));
  value = std::min(value, PROTOCOL1_MOVING_SPEED_MAX);

  if (velocity < 0.0) {
    value += PROTOCOL1_MOVING_SPEED_CW_OFFSET;
  }
  return value;
}

bool DynamixelHardware::uses_protocol1_moving_speed_velocity()
{
  const auto control_item = control_items_.find(GOAL_VELOCITY);
  if (control_item == control_items_.cend() or control_item->second == nullptr) {
    return false;
  }

  return std::abs(dynamixel_workbench_.getProtocolVersion() - PROTOCOL_1) < 0.01f and
         std::strcmp(control_item->second->item_name, MOVING_SPEED) == 0;
}

std::vector<int32_t> DynamixelHardware::get_position_commands()
{
  std::vector<int32_t> commands;
  for (const auto & joint_info : joints_info_) {
    const double pos = joint_info.inverted ? -joint_info.command.position : joint_info.command.position;
    commands.push_back(dynamixel_workbench_.convertRadian2Value(joint_info.id, pos));
  }
  return commands;
}

hardware_interface::return_type DynamixelHardware::write_velocity_commands(
  std::vector<uint8_t> ids, std::vector<int32_t> commands)
{
  constexpr uint8_t GOAL_VELOCITYIndex = 1;
  const char * log{};
  if (not dynamixel_workbench_.syncWrite(
        GOAL_VELOCITYIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
    return hardware_interface::return_type::ERROR;
  }
  for (auto & joint_info : joints_info_) {
    joint_info.prev_command.velocity = joint_info.command.velocity;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardware::write_position_commands(
  std::vector<uint8_t> ids, std::vector<int32_t> commands)
{
  constexpr uint8_t GOAL_POSITIONIndex = 0;
  const char * log{};
  if (not dynamixel_workbench_.syncWrite(
        GOAL_POSITIONIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
    return hardware_interface::return_type::ERROR;
  }
  for (auto & joint_info : joints_info_) {
    joint_info.prev_command.position = joint_info.command.position;
  }
  return hardware_interface::return_type::OK;
}
void DynamixelHardware::set_command_to_position()
{
  for (auto & joint : joints_info_) {
    joint.state.position = joint.command.position;
    joint.state.velocity = joint.command.velocity;
  }
}
void DynamixelHardware::reset_command()
{
  for (auto & joint_info : joints_info_) {
    joint_info.command.position = joint_info.state.position;
    joint_info.command.velocity = 0.0;
    joint_info.command.effort = 0.0;
    joint_info.prev_command = joint_info.command;
  }
}

}  // namespace dynamixel_hardware
