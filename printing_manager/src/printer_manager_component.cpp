#include "printer_manager/printer_manager_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <functional>
#include <memory>
#include <vector>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace printer_manager
{

PrinterManagerComponent::PrinterManagerComponent(const rclcpp::NodeOptions& options) : Node("printer_manager", options)
{
  // Create callback groups for multithreading
  slicing_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  temp_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  settings_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  initialize_communications();
}

void PrinterManagerComponent::initialize_communications()
{
  RCLCPP_INFO(get_logger(), "Initializing communications...");
  using namespace std::placeholders;

  // Action server for printer preparation
  printer_server_ = rclcpp_action::create_server<ur_slicer_interfaces::action::PreparePrinter>(
      this, "ur5e_printer", std::bind(&PrinterManagerComponent::handle_goal, this, _1, _2),
      std::bind(&PrinterManagerComponent::handle_cancel, this, _1),
      std::bind(&PrinterManagerComponent::handle_accepted, this, _1), rcl_action_server_get_default_options(),
      slicing_callback_group_);

  RCLCPP_INFO(get_logger(), "Printer preparation action server is ready.");

  // Subscriber for nozzle temperature with a callback group
  rclcpp::SubscriptionOptions temp_options;
  temp_options.callback_group = temp_callback_group_;
  nozzle_temp_sub_ = create_subscription<ur_slicer_interfaces::msg::NozzleTemperature>(
      "nozzle_temperature", 10, std::bind(&PrinterManagerComponent::update_temp, this, std::placeholders::_1),
      temp_options);

  // Subscriber for slicer settings with a callback group
  rclcpp::SubscriptionOptions settings_options;
  settings_options.callback_group = settings_callback_group_;
  slicer_settings_sub_ = create_subscription<ur_slicer_interfaces::msg::SlicerSettings>(
      "slicer_settings", 10, std::bind(&PrinterManagerComponent::slicer_settings_callback, this, std::placeholders::_1),
      settings_options);

  // Subscriber for beginning printing with a callback group
  begin_printing_sub_ = create_subscription<std_msgs::msg::Bool>(
      "begin_printing", 10, std::bind(&PrinterManagerComponent::begin_printing_callback, this, std::placeholders::_1),
      settings_options);

  // Subscriber for bed height with a callback group
  bed_height_sub_ = create_subscription<std_msgs::msg::Float32>(
      "bed_height", 10, std::bind(&PrinterManagerComponent::bed_height_callback, this, std::placeholders::_1),
      settings_options);

  // Action client for slicing with a callback group
  slicer_client_ = rclcpp_action::create_client<ur_slicer_interfaces::action::Slicer>(this, "slicer_action",
                                                                                      slicing_callback_group_);

  // Wait for the action server to be available
  while (!slicer_client_->wait_for_action_server(std::chrono::seconds(1)))
  {
    RCLCPP_INFO(get_logger(), "Waiting for slicer action server...");
  }
  RCLCPP_INFO(get_logger(), "Slicer action server is available.");

  // Service client for extruder control with a callback group
  extruder_client_ = create_client<ur_slicer_interfaces::srv::ExtruderControl>(
      "extruder_control", rmw_qos_profile_default, slicing_callback_group_);

  // Wait for the service to be available
  while (!extruder_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_INFO(get_logger(), "Waiting for extruder control service...");
  }
  RCLCPP_INFO(get_logger(), "Extruder control service is available.");

  // Service client for heater control with a callback group
  heater_client_ = create_client<ur_slicer_interfaces::srv::HeaterControl>("heater_control", rmw_qos_profile_default,
                                                                           slicing_callback_group_);

  // Wait for the service to be available
  while (!heater_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_INFO(get_logger(), "Waiting for heater control service...");
  }
  RCLCPP_INFO(get_logger(), "Heater control service is available.");
}

// Action server callbacks
rclcpp_action::GoalResponse PrinterManagerComponent::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ur_slicer_interfaces::action::PreparePrinter::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received goal request with file: %s", goal->filepath.c_str());

  if (goal->preheat)
  {
    begin_heating();
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PrinterManagerComponent::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_slicer_interfaces::action::PreparePrinter>> handle)
{
  RCLCPP_INFO(get_logger(), "Received cancel request");

  // Handle cancel request
  stop_heating();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PrinterManagerComponent::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_slicer_interfaces::action::PreparePrinter>> handle)
{
  // Instead of creating a new thread, we'll use the executor's thread pool by submitting the work
  // directly to the callback group via a timer that fires immediately
  RCLCPP_INFO(get_logger(), "Starting new thread");
  current_server_goal_handle_ = handle;
  std::thread{ std::bind(&PrinterManagerComponent::slicing_request, this, std::placeholders::_1), handle }.detach();
}

// Slicing Related Functions
void PrinterManagerComponent::slicing_request(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_slicer_interfaces::action::PreparePrinter>> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<ur_slicer_interfaces::action::PreparePrinter::Result>();

  // Check if the goal is valid
  if (goal->filepath.empty())
  {
    RCLCPP_ERROR(get_logger(), "Invalid slicing request: empty file path");
    handle->abort(result);
    return;
  }

  // Prepare the slicing goal
  auto slicing_goal = ur_slicer_interfaces::action::Slicer::Goal();
  slicing_goal.filepath = goal->filepath;
  slicing_goal.origin_pose = goal->origin_pose;

  // Send slicing request to the action server and assign callback functions
  auto send_goal_options = rclcpp_action::Client<ur_slicer_interfaces::action::Slicer>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&PrinterManagerComponent::slicing_confirmed, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&PrinterManagerComponent::slicing_feedback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&PrinterManagerComponent::slicing_result, this, std::placeholders::_1);

  auto result_future = slicer_client_->async_send_goal(slicing_goal, send_goal_options);
  RCLCPP_INFO(get_logger(), "Slicing request sent successfully");
}

void PrinterManagerComponent::slicing_confirmed(
    const rclcpp_action::ClientGoalHandle<ur_slicer_interfaces::action::Slicer>::SharedPtr goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(get_logger(), "Slicing request was rejected");
    return;
  }
  RCLCPP_INFO(get_logger(), "Slicing request accepted");
}

void PrinterManagerComponent::slicing_feedback(
    const rclcpp_action::ClientGoalHandle<ur_slicer_interfaces::action::Slicer>::SharedPtr goal_handle,
    const std::shared_ptr<const ur_slicer_interfaces::action::Slicer::Feedback> feedback)
{
  // Displays slicing progress in "N layers of M layers sliced"
  RCLCPP_INFO(get_logger(), "Slicing progress: %d of %d layers sliced", feedback->layers_finished,
              feedback->total_layers);
}

void PrinterManagerComponent::slicing_result(
    const rclcpp_action::ClientGoalHandle<ur_slicer_interfaces::action::Slicer>::WrappedResult& wrapped_result)
{
  if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO(get_logger(), "Slicing completed successfully");
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Slicing failed");
    return;
  }

  rclcpp::NodeOptions node_options;
  // node_options.automatically_declare_parameters_from_overrides(true).parameter_overrides({ { "use_sim_time", true }
  // }); auto move_group_node = rclcpp::Node::make_shared("printer_move_group", node_options);

  static const std::string PLANNING_GROUP = "ur_arm";

  moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), PLANNING_GROUP);
  joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group.setPlannerId("LIN");

  // Go to home pose
  bed_origin_.x = 0.0;
  bed_origin_.y = -0.55;
  bed_origin_.z = -0.419;

  geometry_msgs::msg::Pose home_pose;
  home_pose.position.x = bed_origin_.x;
  home_pose.position.y = bed_origin_.y;
  home_pose.position.z = bed_origin_.z + 0.25;
  home_pose.orientation.w = 0.7071068;
  home_pose.orientation.x = 0.7071068;
  home_pose.orientation.y = 0.0;
  home_pose.orientation.z = 0.0;

  moveit::planning_interface::MoveGroupInterface::Plan current_plan;

  move_group.setPoseTarget(home_pose);

  bool success = (move_group.plan(current_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    move_group.execute(current_plan);
    RCLCPP_INFO(get_logger(), "Moving to home pose");
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Failed to move to home pose in LIN planner, attempting PTP");
    move_group.setPlannerId("PTP");
    move_group.setPoseTarget(home_pose);
    success = (move_group.plan(current_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
      move_group.execute(current_plan);
      RCLCPP_INFO(get_logger(), "Moving to home pose");
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to move to home pose in PTP planner");
      return;
    }
  }

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(this->shared_from_this(), "base_link", "stl_markers",
                                                      move_group.getRobotModel());

  visual_tools.loadRemoteControl();

  // Handle the result
  auto sliced_list = wrapped_result.result->path_list;
  for (const auto& path : sliced_list)
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // Loop through the points in the path
    std::vector<geometry_msgs::msg::Pose> waypoints;

    for (const auto& point : path.path)
    {
      // Create a Pose object for the point
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = point.x + bed_origin_.x;
      target_pose.position.y = point.y + bed_origin_.y;
      target_pose.position.z = point.z + bed_origin_.z;
      target_pose.orientation.w = 0.7071068;
      target_pose.orientation.x = 0.7071068;
      target_pose.orientation.y = 0.0;
      target_pose.orientation.z = 0.0;

      RCLCPP_INFO(get_logger(), "Adding point to waypoints: x: %f, y: %f, z: %f", target_pose.position.x,
                  target_pose.position.y, target_pose.position.z);

      waypoints.push_back(target_pose);
    }

    std::vector<geometry_msgs::msg::Pose> start_pose = { waypoints.front() };

    // Wait for nozzle heater to be ready
    while (abs(current_temp_ < nozzle_temp_) >= 5)
    {
      RCLCPP_INFO(get_logger(), "Waiting for nozzle heater to reach %d degrees Celsius", nozzle_temp_);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // Move to the start pose
    double start_fraction = move_group.computeCartesianPath(start_pose, eef_step, jump_threshold, trajectory);

    if (start_fraction < 0.99)
    {
      RCLCPP_ERROR(get_logger(), "Failed to move to start pose (%.0f%% coverage)", start_fraction * 100);
      return;
    }

    // Plan motion
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    // Execute if successful and nozzle turns on
    if (fraction >= 0.99)
    {
      plan.trajectory_ = trajectory;

      // Use RVIZ visual tools gui to confirm path
      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

      extruder_on();
      // if (extruder_on_)
      if (true)
      {
        RCLCPP_INFO(get_logger(), "Extruder turned on.");
        move_group.execute(plan);
        extruder_off();
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Failed to turn on extruder.");
        return;
      }
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Cartesian path planning failed (%.0f%% coverage)", fraction * 100);
    }
  }

  move_group.setJointValueTarget(home_pose);

  success = (move_group.plan(current_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    move_group.execute(current_plan);
    RCLCPP_INFO(get_logger(), "Moving to home pose");
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Failed to move to home pose");
    return;
  }

  auto result = std::make_shared<ur_slicer_interfaces::action::PreparePrinter::Result>();

  result->heater_ready = true;
  result->motion_path_ready = true;
  stop_heating();
  RCLCPP_INFO(get_logger(), "Slicing and motion path printing completed successfully");
  current_server_goal_handle_->succeed(result);
  return;
}

void PrinterManagerComponent::update_temp(const ur_slicer_interfaces::msg::NozzleTemperature::SharedPtr msg)
{
  // Handle nozzle temperature update
  RCLCPP_INFO(get_logger(), "Nozzle temperature updated: %d", msg->nozzle_temperature);
  // Check if the temperature is within the desired range
  current_temp_ = msg->nozzle_temperature;
}

// Heater Control Functions
void PrinterManagerComponent::begin_heating()
{
  auto request = std::make_shared<ur_slicer_interfaces::srv::HeaterControl::Request>();
  request->heater_on = true;
  request->heater_off = false;
  request->desired_temperature = nozzle_temp_;

  heater_client_->async_send_request(
      request, [this](rclcpp::Client<ur_slicer_interfaces::srv::HeaterControl>::SharedFuture future) {
        try
        {
          auto response = future.get();
          if (response->control_confirmed)
          {
            RCLCPP_INFO(get_logger(), "MAN: Heater turned on.");
          }
          else
          {
            RCLCPP_ERROR(get_logger(), "MAN: Heater failed to turn on.");
          }
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR(get_logger(), "MAN: Service call failed: %s", e.what());
        }
      });
}

void PrinterManagerComponent::stop_heating()
{
  auto request = std::make_shared<ur_slicer_interfaces::srv::HeaterControl::Request>();
  request->heater_on = false;
  request->heater_off = true;
  request->desired_temperature = nozzle_temp_;

  heater_client_->async_send_request(
      request, [this](rclcpp::Client<ur_slicer_interfaces::srv::HeaterControl>::SharedFuture future) {
        try
        {
          auto response = future.get();
          if (response->control_confirmed)
          {
            RCLCPP_INFO(get_logger(), "MAN: Heater turned off.");
          }
          else
          {
            RCLCPP_ERROR(get_logger(), "MAN: Heater failed to turn off.");
          }
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR(get_logger(), "MAN: Service call failed: %s", e.what());
        }
      });
}

void PrinterManagerComponent::extruder_on()
{
  auto request = std::make_shared<ur_slicer_interfaces::srv::ExtruderControl::Request>();
  request->on_extruder = true;
  request->off_extruder = false;

  auto future = extruder_client_->async_send_request(request);
}

void PrinterManagerComponent::extruder_off()
{
  auto request = std::make_shared<ur_slicer_interfaces::srv::ExtruderControl::Request>();
  request->on_extruder = false;
  request->off_extruder = true;

  auto future = extruder_client_->async_send_request(request);
}

// Settings Functions
void PrinterManagerComponent::slicer_settings_callback(const ur_slicer_interfaces::msg::SlicerSettings::SharedPtr msg)
{
  // Handle slicer settings
  nozzle_temp_ = msg->nozzle_temperature;
  infill_percent_ = msg->infill_density;
  layer_height_ = msg->layer_height;
  print_speed_ = msg->print_speed;
  // Log the settings
  RCLCPP_INFO(get_logger(), "Received slicer settings:");
  RCLCPP_INFO(get_logger(), "Infill Percent: %d", infill_percent_);
  RCLCPP_INFO(get_logger(), "Layer Height: %f", layer_height_);
  RCLCPP_INFO(get_logger(), "Print Speed: %f", print_speed_);
}

void PrinterManagerComponent::begin_printing_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data)
  {
    RCLCPP_INFO(get_logger(), "Begin printing signal received.");
    // Start the printing process
  }
}

void PrinterManagerComponent::bed_height_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  // Handle bed height update
  bed_height_ = msg->data;
  RCLCPP_INFO(get_logger(), "Bed height updated: %f", bed_height_);
}

}  // namespace printer_manager

// Register the component with class loader
RCLCPP_COMPONENTS_REGISTER_NODE(printer_manager::PrinterManagerComponent)
