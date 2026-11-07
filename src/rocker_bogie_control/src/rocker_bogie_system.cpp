#include "rocker_bogie_control/rocker_bogie_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <numbers>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>

using namespace ctre::phoenix6;

namespace rocker_bogie_hardware
{

hardware_interface::CallbackReturn RockerBogieSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::array<std::array<const char *, 3>, 2> wheel_names{{
    {{"front_left_wheel_joint", "mid_left_wheel_joint", "back_left_wheel_joint"}},
    {{"front_right_wheel_joint", "mid_right_wheel_joint", "back_right_wheel_joint"}}
  }};

  hw_commands_.assign(info_.joints.size(), 0.0);
  hw_positions_.assign(info_.joints.size(), 0.0);
  hw_velocities_.assign(info_.joints.size(), 0.0);

  for (size_t side = 0; side < wheel_names.size(); ++side) {
    for (size_t wheel = 0; wheel < wheel_names[side].size(); ++wheel) {
      const auto joint = std::find_if(
        info_.joints.begin(), info_.joints.end(),
        [&wheel_names, side, wheel](const auto & candidate) {
          return candidate.name == wheel_names[side][wheel];
        });
      if (joint == info_.joints.end()) {
        RCLCPP_ERROR(
          rclcpp::get_logger("RockerBogieSystemHardware"),
          "Required wheel joint '%s' is missing from ros2_control",
          wheel_names[side][wheel]);
        return hardware_interface::CallbackReturn::ERROR;
      }

      const auto joint_index = static_cast<size_t>(std::distance(info_.joints.begin(), joint));
      side_joint_indices_[side].push_back(joint_index);
    }
    drive_joint_indices_[side] = side_joint_indices_[side].front();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RockerBogieSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RockerBogieSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto & side_joint_indices : side_joint_indices_) {
    for (const auto joint_index : side_joint_indices) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[joint_index].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[joint_index]));
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RockerBogieSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("RockerBogieSystemHardware"), "Activating hardware...");

  for (size_t side = 0; side < motors_.size(); ++side) {
    motors_[side] = std::make_shared<ctre::phoenix6::hardware::TalonFX>(
      motor_can_ids_[side], canbus_name_);

    // ctre::phoenix6::configs::TalonFXConfiguration config{};
    // config.Slot0.kP = 0.1;
    // config.Slot0.kI = 0.0;
    // config.Slot0.kD = 0.0;
    // config.Slot0.kV = 0.12;
    // const auto status = motors_[side]->GetConfigurator().Apply(config);
    // if (!status.IsOK()) {
    //   RCLCPP_ERROR(
    //     rclcpp::get_logger("RockerBogieSystemHardware"),
    //     "Failed to configure TalonFX CAN ID %d: %s",
    //     motor_can_ids_[side], status.GetName());
    //   return hardware_interface::CallbackReturn::ERROR;
    // }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RockerBogieSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("RockerBogieSystemHardware"), "Deactivating hardware...");
  for (size_t side = 0; side < motors_.size(); ++side) {
    if (motors_[side]) {
      motors_[side]->SetControl(velocity_requests_[side].WithVelocity(
        units::turns_per_second_t(0.0)));
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RockerBogieSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  for (size_t side = 0; side < motors_.size(); ++side) {
    if (!motors_[side]) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RockerBogieSystemHardware"),
        "Cannot read wheel feedback before hardware activation");
      return hardware_interface::return_type::ERROR;
    }

    const double position_radians = motors_[side]->GetPosition().GetValueAsDouble() * 2.0 *
      std::numbers::pi;
    const double velocity_radians_per_second =
      motors_[side]->GetVelocity().GetValueAsDouble() * 2.0 * std::numbers::pi;
    for (const auto joint_index : side_joint_indices_[side]) {
      hw_positions_[joint_index] = position_radians;
      hw_velocities_[joint_index] = velocity_radians_per_second;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RockerBogieSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t side = 0; side < motors_.size(); ++side) {
    if (!motors_[side]) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RockerBogieSystemHardware"),
        "Cannot write wheel commands before hardware activation");
      return hardware_interface::return_type::ERROR;
    }

    ctre::phoenix::unmanaged::FeedEnable(10);
    const double target_rotations_per_second =
      hw_commands_[drive_joint_indices_[side]] / (2.0 * std::numbers::pi);
    motors_[side]->SetControl(controls::VelocityVoltage(target_rotations_per_second * 10_tps));
    
    // controls::DutyCycleOut test{0.5};
    // motors_[side]->SetControl(test);
  }
  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rocker_bogie_hardware::RockerBogieSystemHardware, hardware_interface::SystemInterface)