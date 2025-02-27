#ifndef SAGAN_CONTROLLERS_SAGAN_DRIVE_CONTROLLER_CPP_
#define SAGAN_CONTROLLERS_SAGAN_DRIVE_CONTROLLER_CPP_

#include <sagan_controllers/sagan_drive_controller.hpp>

namespace sagan_controllers
{

controller_interface::CallbackReturn SaganDriverController::on_init()
{
  try
  {
    auto_declare<std::vector<std::string>>("joints", joint_names_);
    auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
    auto_declare<std::vector<double>>("joints_references", {});
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

  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  if (command_interface_types_.empty())
  {
    command_interface_types_ = get_node()->get_parameter("command_interfaces").as_string_array();
  }

  if (command_interface_types_.empty())
  {
    RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  for (const auto &interface : command_interface_types_)
  {
    auto it =
      std::find(allowed_command_interface_types_.begin(), allowed_command_interface_types_.end(), interface);
    if (it == allowed_command_interface_types_.end())
    {
      RCLCPP_ERROR(logger, "Command interface type '%s' not allowed! Only effort type is allowed!", interface.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  joint_command_interface_.resize(allowed_command_interface_types_.size());

  // State interface checking
  state_interface_types_ = get_node()->get_parameter("state_interfaces").as_string_array();

  if (state_interface_types_.empty())
  {
  RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty.");
  return CallbackReturn::FAILURE;
  }

  joint_state_interface_.resize(allowed_state_interface_types_.size());

  // Pritig format
  auto get_interface_list = [](const std::vector<std::string> &interface_types)
  {
    std::stringstream ss_interfaces;
    for (size_t index = 0; index < interface_types.size(); ++index)
    {
      if (index != 0)
     {
      ss_interfaces << " ";
      }
      ss_interfaces << interface_types[index];
    }
    return ss_interfaces.str();
  };

  RCLCPP_INFO(
    logger, "Command interfaces are [%s] and and state interfaces are [%s].",
    get_interface_list(command_interface_types_).c_str(),
    get_interface_list(state_interface_types_).c_str());

  return CallbackReturn::SUCCESS;

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