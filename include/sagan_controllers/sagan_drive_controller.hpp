#ifndef SAGAN_CONTROLLERS_SAGAN_DRIVE_CONTROLLER_HPP_
#define SAGAN_CONTROLLERS_SAGAN_DRIVE_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"

namespace sagan_controllers
{
class SaganDriverController : public controller_interface::ControllerInterface
{
public: 
  SaganDriverController();
    
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

}

} // namespace sagan_controllers

#endif // SAGAN_CONTROLLERS_SAGAN_DRIVE_CONTROLLER_HPP_