#ifndef PRINTER_MANAGER_COMPONENT_HPP
#define PRINTER_MANAGER_COMPONENT_HPP

#include <functional>
#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>

#include "std_msgs/msg/float32.hpp"

#include "ur_slicer_interfaces/action/prepare_printer.hpp"
#include "ur_slicer_interfaces/action/slicer.hpp"
#include "ur_slicer_interfaces/msg/nozzle_temperature.hpp"
#include "ur_slicer_interfaces/msg/path.hpp"
#include "ur_slicer_interfaces/msg/slicer_settings.hpp"
#include "ur_slicer_interfaces/srv/extruder_control.hpp"
#include "ur_slicer_interfaces/srv/heater_control.hpp"

namespace printer_manager
{

class PrinterManagerComponent : public rclcpp::Node
{
public:
  explicit PrinterManagerComponent(const rclcpp::NodeOptions& options);

private:
  void initialize_communications();

  // Action server callbacks
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID& uuid,
              std::shared_ptr<const ur_slicer_interfaces::action::PreparePrinter::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_slicer_interfaces::action::PreparePrinter>> handle);

  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_slicer_interfaces::action::PreparePrinter>> handle);

  // Slicing Related Functions
  void slicing_request(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_slicer_interfaces::action::PreparePrinter>> handle);

  void
  slicing_confirmed(const rclcpp_action::ClientGoalHandle<ur_slicer_interfaces::action::Slicer>::SharedPtr goal_handle);

  void
  slicing_feedback(const rclcpp_action::ClientGoalHandle<ur_slicer_interfaces::action::Slicer>::SharedPtr goal_handle,
                   const std::shared_ptr<const ur_slicer_interfaces::action::Slicer::Feedback> feedback);

  void
  slicing_result(const rclcpp_action::ClientGoalHandle<ur_slicer_interfaces::action::Slicer>::WrappedResult& result);

  void update_temp(const ur_slicer_interfaces::msg::NozzleTemperature::SharedPtr msg);

  // Heater Control Functions
  void begin_heating();
  void stop_heating();
  void extruder_on();
  void extruder_off();

  // Settings Functions
  void slicer_settings_callback(const ur_slicer_interfaces::msg::SlicerSettings::SharedPtr msg);
  void begin_printing_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void bed_height_callback(const std_msgs::msg::Float32::SharedPtr msg);

  // Member variables
  // MoveIt variables
  const moveit::core::JointModelGroup* joint_model_group;

  // Action server
  rclcpp_action::Server<ur_slicer_interfaces::action::PreparePrinter>::SharedPtr printer_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_slicer_interfaces::action::PreparePrinter>>
      current_server_goal_handle_;

  // Clients
  rclcpp_action::Client<ur_slicer_interfaces::action::Slicer>::SharedPtr slicer_client_;
  rclcpp::Client<ur_slicer_interfaces::srv::HeaterControl>::SharedPtr heater_client_;
  rclcpp::Client<ur_slicer_interfaces::srv::ExtruderControl>::SharedPtr extruder_client_;

  // Pubs and subs
  rclcpp::Subscription<ur_slicer_interfaces::msg::SlicerSettings>::SharedPtr slicer_settings_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr begin_printing_sub_;
  rclcpp::Subscription<ur_slicer_interfaces::msg::NozzleTemperature>::SharedPtr nozzle_temp_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr bed_height_sub_;

  // Slicing Settings
  int nozzle_temp_ = 200;     // Default nozzle temperature
  int infill_percent_ = 20;   // Default infill percentage
  float layer_height_ = 0.2;  // Default layer height
  float print_speed_ = 50.0;  // Default print speed

  // Temp Tracker
  int current_temp_ = 0;      // Current temperature of the nozzle
  bool extruder_on_ = false;  // Extruder state

  // Bed settings
  geometry_msgs::msg::Point bed_origin_;
  float bed_height_ = 0.0;  // Height of the bed

  // Moveit Settings
  const double jump_threshold = 0.0;
  const double eef_step = 0.00001;

  // Callback groups for multithreading
  rclcpp::CallbackGroup::SharedPtr slicing_callback_group_;
  rclcpp::CallbackGroup::SharedPtr temp_callback_group_;
  rclcpp::CallbackGroup::SharedPtr settings_callback_group_;
};

}  // namespace printer_manager

#endif  // PRINTER_MANAGER_COMPONENT_HPP
