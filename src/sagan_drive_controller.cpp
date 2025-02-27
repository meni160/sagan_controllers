#ifndef SAGAN_CONTROLLERS_SAGAN_DRIVE_CONTROLLER_CPP_
#define SAGAN_CONTROLLERS_SAGAN_DRIVE_CONTROLLER_CPP_

#include <sagan_controllers/sagan_drive_controller.hpp>

namespace sagan_controllers
{

controller_interface::CallbackReturn SaganDriverController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
}

controller_interface::CallbackReturn SaganDriverController::on_configure(
  const rclcpp_lifecycle::State &)    
{
  auto logger = get_node()->get_logger();

  // update parameters if they have changed
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }
}

controller_interface::InterfaceConfiguration
InterfaceConfiguration SaganDriverController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : params_.left_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.right_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration
InterfaceConfiguration SaganDriverController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : params_.left_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + feedback_type());
  }
  for (const auto & joint_name : params_.right_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + feedback_type());
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn SaganDriverController::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SaganDriverController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Deactivating");
}

controller_interface::return_type SaganDriverController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = this->get_node()->get_logger();
  // update dynamic parameters
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
  }
}

} // namespace sagan_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  sagan_controllers::SaganDriverController, controller_interface::ControllerInterface)
  
#endif //SAGAN_CONTROLLERS_SAGAN_DRIVE_CONTROLLER_CPP_