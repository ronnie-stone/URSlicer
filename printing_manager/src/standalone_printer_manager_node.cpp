#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "printer_manager/printer_manager_component.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({ { "use_sim_time", true } });

  auto printer_manager_node = std::make_shared<printer_manager::PrinterManagerComponent>(options);

  // Create a multi-threaded executor for better performance
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(printer_manager_node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
