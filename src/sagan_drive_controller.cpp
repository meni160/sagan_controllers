#ifndef SAGAN_CONTROLLERS_SAGAN_DRIVE_CONTROLLER_CPP_
#define SAGAN_CONTROLLERS_SAGAN_DRIVE_CONTROLLER_CPP_

#include <sagan_controllers/sagan_drive_controller.hpp>

namespace sagan_controllers
{

controller_interface::CallbackReturn SaganDriveController::on_init()
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

controller_interface::CallbackReturn SaganDriveController::on_configure(
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

} // namespace sagan_controllers

#endif //SAGAN_CONTROLLERS_SAGAN_DRIVE_CONTROLLER_CPP_