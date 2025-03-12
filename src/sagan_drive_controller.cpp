#ifndef sagan_drive_controller__SAGAN_DRIVE_CONTROLLER_CPP_
#define sagan_drive_controller__SAGAN_DRIVE_CONTROLLER_CPP_

#include <sagan_drive_controller/sagan_drive_controller.hpp>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace sagan_drive_controller
{

SaganDriverController::SaganDriverController()
  : controller_interface::ControllerInterface(),
    joint_names_({})
{
  fprintf(stderr, "Contruction done");
}

controller_interface::CallbackReturn SaganDriverController::on_init()
{
  try
  {
    auto_declare<std::vector<std::string>>("joints", joint_names_);
    auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
    auto_declare<std::vector<double>>("joints_references", {});


    auto_declare<std::vector<std::string>>("front_left_wheel_joint", fl_wheel_joint);
    auto_declare<std::vector<std::string>>("front_right_wheel_joint", fr_wheel_joint);
    auto_declare<std::vector<std::string>>("rear_left_wheel_joint", rl_wheel_joint);
    auto_declare<std::vector<std::string>>("rear_right_wheel_joint", rr_wheel_joint);

    auto_declare<std::vector<std::string>>("front_left_steering_joint", fl_steering_joint);
    auto_declare<std::vector<std::string>>("front_right_steering_joint", fr_steering_joint);
    auto_declare<std::vector<std::string>>("rear_left_steering_joint", rl_steering_joint);
    auto_declare<std::vector<std::string>>("rear_right_steering_joint", rr_steering_joint);

  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SaganDriverController::on_configure(
  const rclcpp_lifecycle::State &)    
{
  auto logger = get_node()->get_logger();

  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  fl_wheel_joint = get_node()->get_parameter("front_left_wheel_joint").as_string_array();
  fr_wheel_joint = get_node()->get_parameter("front_right_wheel_joint").as_string_array();
  rl_wheel_joint = get_node()->get_parameter("rear_left_wheel_joint").as_string_array();
  rr_wheel_joint = get_node()->get_parameter("rear_right_wheel_joint").as_string_array();

  fl_wheel_joint = get_node()->get_parameter("front_left_steering_joint").as_string_array();
  fr_wheel_joint = get_node()->get_parameter("front_right_steering_joint").as_string_array();
  rl_wheel_joint = get_node()->get_parameter("rear_left_steering_joint").as_string_array();
  rr_wheel_joint = get_node()->get_parameter("rear_right_steering_joint").as_string_array();


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
SaganDriverController::command_interface_configuration() const
{
  // controller_interface::InterfaceConfiguration command_interfaces_config;
  // command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // command_interfaces_config.names.reserve(joint_names_.size() * command_interface_types_.size());
  // for (const auto &joint : joint_names_)
  // {
  //   for (const auto &interface_type : command_interface_types_)
  //   {
  //     command_interfaces_config.names.push_back(joint + "/" + interface_type);
  //   }
  // }

  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.reserve(4 * command_interface_types_.size());

  for (const auto &interface_type : command_interface_types_)
  {
    command_interfaces_config.names.push_back(fl_wheel_joint[0] + "/" + interface_type);
  }

  for (const auto &interface_type : command_interface_types_)
  {
    command_interfaces_config.names.push_back(fr_wheel_joint[0] + "/" + interface_type);
  }

  for (const auto &interface_type : command_interface_types_)
  {
    command_interfaces_config.names.push_back(rl_wheel_joint[0] + "/" + interface_type);
  }

  for (const auto &interface_type : command_interface_types_)
  {
    command_interfaces_config.names.push_back(rr_wheel_joint[0] + "/" + interface_type);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
SaganDriverController::state_interface_configuration() const
{
  // controller_interface::InterfaceConfiguration state_interfaces_config;
  // state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // state_interfaces_config.names.reserve(joint_names_.size() * state_interface_types_.size());
  // for (const auto &joint_name : joint_names_)
  // {
  //   for (const auto &interface_type : state_interface_types_)
  //   {
  //     state_interfaces_config.names.push_back(joint_name + "/" + interface_type);
  //   }
  // }

  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names.reserve(4 * state_interface_types_.size());

  for (const auto &interface_type : state_interface_types_)
  {
    state_interfaces_config.names.push_back(fl_wheel_joint[0] + "/" + interface_type);
  }

  for (const auto &interface_type : state_interface_types_)
  {
    state_interfaces_config.names.push_back(fr_wheel_joint[0] + "/" + interface_type);
  }

  for (const auto &interface_type : state_interface_types_)
  {
    state_interfaces_config.names.push_back(rl_wheel_joint[0] + "/" + interface_type);
  }

  for (const auto &interface_type : state_interface_types_)
  {
    state_interfaces_config.names.push_back(rr_wheel_joint[0] + "/" + interface_type);
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn SaganDriverController::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Activating");

  std::vector<std::string> wheel_joint_names_ = {fl_wheel_joint[0], fr_wheel_joint[0], rl_wheel_joint[0], rr_wheel_joint[0]};

  for (const auto &interface : command_interface_types_)
  {
    auto it =
      std::find(allowed_command_interface_types_.begin(), allowed_command_interface_types_.end(), interface);
    auto index = std::distance(allowed_command_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(command_interfaces_, wheel_joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", joint_names_.size(),
        interface.c_str(), joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  
  for (const auto &interface : state_interface_types_)
  {
    auto it =
      std::find(allowed_state_interface_types_.begin(), allowed_state_interface_types_.end(), interface);
    auto index = std::distance(allowed_state_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(state_interfaces_, wheel_joint_names_, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", joint_names_.size(),
        interface.c_str(), joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SaganDriverController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Deactivating");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SaganDriverController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto logger = get_node()->get_logger();
  // update dynamic parameters

    for (auto index = 0; index < 4; index++)
    {
      double value = 20.0;
      (void)joint_command_interface_[0][index].get().set_value(value);
    }

  return controller_interface::return_type::OK;
}

} // namespace sagan_drive_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  sagan_drive_controller::SaganDriverController, 
  controller_interface::ControllerInterface)
  
#endif //sagan_drive_controller_SAGAN_DRIVE_CONTROLLER_CPP_