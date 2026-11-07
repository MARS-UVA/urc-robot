#ifndef ROCKER_BOGIE_HARDWARE__ROCKER_BOGIE_SYSTEM_HPP_
#define ROCKER_BOGIE_HARDWARE__ROCKER_BOGIE_SYSTEM_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <ctre/phoenix6/TalonFX.hpp>

#define FRONT_LEFT_WHEEL_ID 13
#define MID_LEFT_WHEEL_ID 14
#define BACK_LEFT_WHEEL_ID 15
#define FRONT_RIGHT_WHEEL_ID 10
#define MID_RIGHT_WHEEL_ID 11
#define BACK_RIGHT_WHEEL_ID 12

namespace rocker_bogie_hardware
{

class RockerBogieSystemHardware : public hardware_interface::SystemInterface
{
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(RockerBogieSystemHardware)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo & info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        std::vector<double> hw_commands_;
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::array<size_t, 2> drive_joint_indices_{};
        std::array<std::vector<size_t>, 2> side_joint_indices_;

        std::array<std::array<std::shared_ptr<ctre::phoenix6::hardware::TalonFX>, 3>, 2> motors_;
        std::array<ctre::phoenix6::controls::VelocityVoltage, 2> velocity_requests_{
            ctre::phoenix6::controls::VelocityVoltage(
              units::turns_per_second_t(0.0)),
            ctre::phoenix6::controls::VelocityVoltage(
              units::turns_per_second_t(0.0))};
        
        const std::array<std::array<int, 3>, 2> motor_can_ids_{{
            {{FRONT_LEFT_WHEEL_ID, MID_LEFT_WHEEL_ID, BACK_LEFT_WHEEL_ID}},
            {{FRONT_RIGHT_WHEEL_ID, MID_RIGHT_WHEEL_ID, BACK_RIGHT_WHEEL_ID}}
        }};
        const std::string canbus_name_ = "*"; // Name of your CANivore network
};

}

#endif