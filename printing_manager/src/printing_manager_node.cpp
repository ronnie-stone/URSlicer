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

#include "ur_slicer_interfaces/action/prepare_printer.hpp"
#include "ur_slicer_interfaces/action/slicer.hpp"
#include "ur_slicer_interfaces/msg/nozzle_temperature.hpp"
#include "ur_slicer_interfaces/msg/path.hpp"
#include "ur_slicer_interfaces/msg/slicer_settings.hpp"
#include "ur_slicer_interfaces/srv/extruder_control.hpp"
#include "ur_slicer_interfaces/srv/heater_control.hpp"

using namespace std::chrono_literals;

class PrinterManager : public rclcpp::Node
{
public:
  PrinterManager(const std::string& node_name, const rclcpp::NodeOptions& options) : Node(node_name, options)
  {
    initialize_communications();
  }

private:
  void initialize_communications()
  {
    RCLCPP_INFO(get_logger(), "Initializing communications...");
    using namespace std::placeholders;

    // Action server for printer preparation
    printer_server_ = rclcpp_action::create_server<ur_slicer_interfaces::action::PreparePrinter>(
        this, "ur_printer", std::bind(&PrinterManager::handle_goal, this, _1, _2),
        std::bind(&PrinterManager::handle_cancel, this, _1), std::bind(&PrinterManager::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "Printer preparation action server is ready.");

    // Subscriber for nozzle temperature
    nozzle_temp_sub_ = create_subscription<ur_slicer_interfaces::msg::NozzleTemperature>(
        "nozzle_temperature", 10, std::bind(&PrinterManager::update_temp, this, std::placeholders::_1));

    // Subscriber for slicer settings
    slicer_settings_sub_ = create_subscription<ur_slicer_interfaces::msg::SlicerSettings>(
        "slicer_settings", 10, std::bind(&PrinterManager::slicer_settings_callback, this, std::placeholders::_1));

    // Subscriber for beginning printing
    begin_printing_sub_ = create_subscription<std_msgs::msg::Bool>(
        "begin_printing", 10, std::bind(&PrinterManager::begin_printing_callback, this, std::placeholders::_1));

    // Action client for slicing
    slicer_client_ = rclcpp_action::create_client<ur_slicer_interfaces::action::Slicer>(this, "slicer_action");
    // Wait for the action server to be available
    while (!slicer_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(get_logger(), "Waiting for slicer action server...");
    }
    RCLCPP_INFO(get_logger(), "Slicer action server is available.");

    // Service client for extruder control
    extruder_client_ = create_client<ur_slicer_interfaces::srv::ExtruderControl>("extruder_control");
    // Wait for the service to be available
    while (!extruder_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(get_logger(), "Waiting for extruder control service...");
    }
    RCLCPP_INFO(get_logger(), "Extruder control service is available.");

    // Service client for heater control
    heater_client_ = create_client<ur_slicer_interfaces::srv::HeaterControl>("heater_control");
    // Wait for the service to be available
    while (!heater_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(get_logger(), "Waiting for heater control service...");
    }
    RCLCPP_INFO(get_logger(), "Heater control service is available.");
  }

  // Action server callbacks
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID& uuid,
              std::shared_ptr<const ur_slicer_interfaces::action::PreparePrinter::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with file: %s", goal->filepath.c_str());
    // Check if the goal is valid
    // if (goal->filepath.empty())
    // {
    //   RCLCPP_ERROR(get_logger(), "Invalid slicing request: empty file path");
    //   return rclcpp_action::GoalResponse::REJECT;
    // }
    if (goal->preheat)
    {
      begin_heating();
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_slicer_interfaces::action::PreparePrinter>> handle)
  {
    RCLCPP_INFO(get_logger(), "Received cancel request");

    // Handle cancel request
    stop_heating();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_slicer_interfaces::action::PreparePrinter>> handle)
  {
    RCLCPP_INFO(get_logger(), "Starting new thread");
    std::thread{ std::bind(&PrinterManager::slicing_request, this, std::placeholders::_1), handle }.detach();
  }

  // Slicing Related Functions

  void slicing_request(
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
        std::bind(&PrinterManager::slicing_confirmed, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&PrinterManager::slicing_feedback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&PrinterManager::slicing_result, this, std::placeholders::_1);

    auto result_future = slicer_client_->async_send_goal(slicing_goal, send_goal_options);
    // if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
    // {
    //   RCLCPP_ERROR(get_logger(), "Failed to send slicing request");
    //   return;
    // }
    RCLCPP_INFO(get_logger(), "Slicing request sent successfully");
  }

  void
  slicing_confirmed(const rclcpp_action::ClientGoalHandle<ur_slicer_interfaces::action::Slicer>::SharedPtr goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(get_logger(), "Slicing request was rejected");
      return;
    }
    RCLCPP_INFO(get_logger(), "Slicing request accepted");
  }

  void
  slicing_feedback(const rclcpp_action::ClientGoalHandle<ur_slicer_interfaces::action::Slicer>::SharedPtr goal_handle,
                   const std::shared_ptr<const ur_slicer_interfaces::action::Slicer::Feedback> feedback)
  {
    // Displays slicing progress in "N layers of M layers sliced"
    RCLCPP_INFO(get_logger(), "Slicing progress: %d of %d layers sliced", feedback->layers_finished,
                feedback->total_layers);
  }

  void
  slicing_result(const rclcpp_action::ClientGoalHandle<ur_slicer_interfaces::action::Slicer>::WrappedResult& result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(get_logger(), "Slicing completed successfully");
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Slicing failed");
      return;
    }

    namespace rvt = rviz_visual_tools;

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true).parameter_overrides(
        { { "use_sim_time", true } });
    auto move_group_node = rclcpp::Node::make_shared("printer_move_group", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "ur_arm";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Go to home pose

    bed_origin_.x = 0.5;
    bed_origin_.y = 0.0;
    bed_origin_.z = -0.25;

    geometry_msgs::msg::Pose home_pose;
    home_pose.position.x = bed_origin_.x;
    home_pose.position.y = bed_origin_.y;
    home_pose.position.z = bed_origin_.z + 0.25;
    home_pose.orientation.w = 0.5;
    home_pose.orientation.x = 0.5;
    home_pose.orientation.y = 0.50;
    home_pose.orientation.z = 0.50;

    moveit::planning_interface::MoveGroupInterface::Plan current_plan;

    move_group.setJointValueTarget(home_pose);

    bool success = (move_group.plan(current_plan) == moveit::core::MoveItErrorCode::SUCCESS);

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

    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "stl_markers",
                                                        move_group.getRobotModel());

    visual_tools.loadRemoteControl();

    // Handle the result
    auto sliced_list = result.result->path_list;
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
        target_pose.orientation.w = 0.5;
        target_pose.orientation.x = 0.5;
        target_pose.orientation.y = 0.5;
        target_pose.orientation.z = 0.5;

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
    return;
  }

  void update_temp(const ur_slicer_interfaces::msg::NozzleTemperature::SharedPtr msg)
  {
    // Handle nozzle temperature update
    RCLCPP_INFO(get_logger(), "Nozzle temperature updated: %d", msg->nozzle_temperature);
    // Check if the temperature is within the desired range
    current_temp_ = msg->nozzle_temperature;
  }

  // Heater Control Functions

  void begin_heating()
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

  void stop_heating()
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

  void extruder_on()
  {
    auto request = std::make_shared<ur_slicer_interfaces::srv::ExtruderControl::Request>();
    request->on_extruder = true;
    request->off_extruder = false;

    auto future = extruder_client_->async_send_request(request);
  }

  void extruder_off()
  {
    auto request = std::make_shared<ur_slicer_interfaces::srv::ExtruderControl::Request>();
    request->on_extruder = false;
    request->off_extruder = true;

    extruder_client_->async_send_request(
        request, [this](rclcpp::Client<ur_slicer_interfaces::srv::ExtruderControl>::SharedFuture future) {
          try
          {
            auto response = future.get();
            if (response->control_confirmed)
            {
              RCLCPP_INFO(get_logger(), "MAN: Extruder turned off.");
              extruder_on_ = false;
            }
            else
            {
              RCLCPP_ERROR(get_logger(), "MAN: Extruder failed to turn off.");
            }
          }
          catch (const std::exception& e)
          {
            RCLCPP_ERROR(get_logger(), "MAN: Service call failed: %s", e.what());
          }
        });
  }

  // Settings Functions
  void slicer_settings_callback(const ur_slicer_interfaces::msg::SlicerSettings::SharedPtr msg)
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

  void begin_printing_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      RCLCPP_INFO(get_logger(), "Begin printing signal received.");
      // Start the printing process
    }
  }

  // Member variables

  // MoveIt variables
  const moveit::core::JointModelGroup* joint_model_group;
  bool mgi_initialized = false;

  // Action server
  rclcpp_action::Server<ur_slicer_interfaces::action::PreparePrinter>::SharedPtr printer_server_;
  // Clients
  rclcpp_action::Client<ur_slicer_interfaces::action::Slicer>::SharedPtr slicer_client_;
  rclcpp::Client<ur_slicer_interfaces::srv::HeaterControl>::SharedPtr heater_client_;
  rclcpp::Client<ur_slicer_interfaces::srv::ExtruderControl>::SharedPtr extruder_client_;
  // Pubs and subs
  rclcpp::Subscription<ur_slicer_interfaces::msg::SlicerSettings>::SharedPtr slicer_settings_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr begin_printing_sub_;
  rclcpp::Subscription<ur_slicer_interfaces::msg::NozzleTemperature>::SharedPtr nozzle_temp_sub_;

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

  // Moveit Settings
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PrinterManager>(
      "ur5e_manager_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).parameter_overrides(
          { { "use_sim_time", true } }));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
