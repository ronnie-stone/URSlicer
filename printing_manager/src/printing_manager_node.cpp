#include <functional>
#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
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
  PrinterManager()
    : Node("ur5e_manager_node",
           rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).parameter_overrides(
               { { "use_sim_time", true } }))
  {
    // Initialize MoveIt interface
    RCLCPP_INFO(get_logger(), "Initializing MoveIt interface...");

    // Initialize MoveIt
    // auto const moveit_node = std::make_shared<rclcpp::Node>(
    //     "ur5e_moveit_node",
    //     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).parameter_overrides(
    //         { { "use_sim_time", true } }));

    // auto move_group_ = moveit::planning_interface::MoveGroupInterface(moveit_node, "ur_manipulator");

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),  // Use current
                                                                                                        // node context
                                                                                   "ur_manipulator");

    RCLCPP_INFO(get_logger(), "MoveIt interface initialized.");

    configure_moveit();
    initialize_communications();
  }

private:
  void configure_moveit()
  {
    RCLCPP_INFO(get_logger(), "Configuring MoveIt...");
    move_group_->setPlanningTime(10.0);
    move_group_->setMaxVelocityScalingFactor(0.1);
    move_group_->setMaxAccelerationScalingFactor(0.1);
    move_group_->setPoseReferenceFrame("base_link");

    RCLCPP_INFO(get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
  }

  void initialize_communications()
  {
    RCLCPP_INFO(get_logger(), "Initializing communications...");
    using namespace std::placeholders;
    // Action server for printer preparation
    printer_server_ = rclcpp_action::create_server<ur_slicer_interfaces::action::PreparePrinter>(
        this, "ur_printer", std::bind(&PrinterManager::handle_goal, this, _1, _2),
        std::bind(&PrinterManager::handle_cancel, this, _1), std::bind(&PrinterManager::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "Printer preparation action server is ready.");

    // Action client for slicing
    slicer_client_ = rclcpp_action::create_client<ur_slicer_interfaces::action::Slicer>(this, "slicer_action");
    // Wait for the action server to be available
    // while (!slicer_client_->wait_for_action_server(std::chrono::seconds(1)))
    // {
    //   RCLCPP_INFO(get_logger(), "Waiting for slicer action server...");
    // }
    RCLCPP_INFO(get_logger(), "Slicer action server is available.");

    // Service client for extruder control
    extruder_client_ = create_client<ur_slicer_interfaces::srv::ExtruderControl>("extruder_control");
    // Wait for the service to be available
    // while (!extruder_client_->wait_for_service(std::chrono::seconds(1)))
    // {
    //   RCLCPP_INFO(get_logger(), "Waiting for extruder control service...");
    // }
    RCLCPP_INFO(get_logger(), "Extruder control service is available.");

    // Service client for heater control
    heater_client_ = create_client<ur_slicer_interfaces::srv::HeaterControl>("heater_control");
    // Wait for the service to be available
    // while (!heater_client_->wait_for_service(std::chrono::seconds(1)))
    // {
    //   RCLCPP_INFO(get_logger(), "Waiting for heater control service...");
    // }
    RCLCPP_INFO(get_logger(), "Heater control service is available.");

    // Publisher for nozzle temperature
    nozzle_temp_pub_ = create_publisher<ur_slicer_interfaces::msg::NozzleTemperature>("nozzle_temperature", 10);

    // Subscriber for slicer settings
    slicer_settings_sub_ = create_subscription<ur_slicer_interfaces::msg::SlicerSettings>(
        "slicer_settings", 10, std::bind(&PrinterManager::slicer_settings_callback, this, std::placeholders::_1));

    // Subscriber for beginning printing
    begin_printing_sub_ = create_subscription<std_msgs::msg::Bool>(
        "begin_printing", 10, std::bind(&PrinterManager::begin_printing_callback, this, std::placeholders::_1));
  }

  // Action server callbacks
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID& uuid,
              std::shared_ptr<const ur_slicer_interfaces::action::PreparePrinter::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with file: %s", goal->filepath.c_str());
    // Check if the goal is valid
    if (goal->filepath.empty())
    {
      RCLCPP_ERROR(get_logger(), "Invalid slicing request: empty file path");
      return rclcpp_action::GoalResponse::REJECT;
    }
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
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send slicing request");
      return;
    }
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
    }

    // Handle the result
    ur_slicer_interfaces::msg::Path sliced_list = result.result->path_list;
    for (const auto& path : sliced_list)
    {
      // First loop through Path objects
      print_path(path);  // Now passing Path instead of Point

      // Then loop through points within each Path (if needed)
      for (const auto& point : path.points)
      {
        // Process individual points
      }
    }
  }

  std::vector<std::vector<geometry_msgs::msg::Pose>>
  request_path_from_server(const std::string& filepath, const geometry_msgs::msg::Pose& origin_pose)
  {
    // Implementation for server communication
    return {};  // Return actual path data
  }

  // Heater Control Functions

  void begin_heating()
  {
    auto request = std::make_shared<ur_slicer_interfaces::srv::HeaterControl::Request>();
    request->heater_on = true;
    request->heater_off = false;
    request->check_status = false;
    auto response = heater_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS)
    {
      if (response.get()->control_confirmed)
      {
        RCLCPP_INFO(get_logger(), "Heater turned on.");
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Heater failed to turn on.");
      }
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Heater service failed to respond");
    }
    return;
  }

  void stop_heating()
  {
    auto request = std::make_shared<ur_slicer_interfaces::srv::HeaterControl::Request>();
    request->heater_on = false;
    request->heater_off = true;
    request->check_status = false;
    auto response = heater_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS)
    {
      if (response.get()->control_confirmed)
      {
        RCLCPP_INFO(get_logger(), "Heater turned off.");
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Heater failed to turn off.");
      }
      if (response.get()->warning_hot)
      {
        RCLCPP_WARN(get_logger(), "Heater turned off. HEATER STILL HOT.");
      }
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Heater service failed to respond");
    }
    return;
  }

  // Extruder Control Functions
  bool extruder_on()
  {
    auto request = std::make_shared<ur_slicer_interfaces::srv::ExtruderControl::Request>();
    request->on_extruder = true;
    request->on_extruder = false;
    auto response = extruder_client_->async_send_request(request);
    // Wait for the response
    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS)
    {
      if (response.get()->motion_allowed)
      {
        RCLCPP_INFO(get_logger(), "Extruder turned on.");
        return true;
      }
    }
    RCLCPP_ERROR(get_logger(), "Extruder failed to turn on.");
    return false;
  }

  void extruder_off()
  {
    auto request = std::make_shared<ur_slicer_interfaces::srv::ExtruderControl::Request>();
    request->on_extruder = false;
    request->on_extruder = true;
    auto response = extruder_client_->async_send_request(request);
    // Wait for the response
    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS)
    {
      if (response.get()->motion_allowed)
      {
        RCLCPP_INFO(get_logger(), "Extruder turned off.");
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Extruder failed to turn off.");
      }
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Extruder service failed to respond");
    }
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
    RCLCPP_INFO(get_logger(), "Nozzle Temp: %d", nozzle_temp_);
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

  // MoveIt
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  // Action server
  rclcpp_action::Server<ur_slicer_interfaces::action::PreparePrinter>::SharedPtr printer_server_;
  // Clients
  rclcpp_action::Client<ur_slicer_interfaces::action::Slicer>::SharedPtr slicer_client_;
  rclcpp::Client<ur_slicer_interfaces::srv::HeaterControl>::SharedPtr heater_client_;
  rclcpp::Client<ur_slicer_interfaces::srv::ExtruderControl>::SharedPtr extruder_client_;
  // Pubs and subs
  rclcpp::Subscription<ur_slicer_interfaces::msg::SlicerSettings>::SharedPtr slicer_settings_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr begin_printing_sub_;
  rclcpp::Publisher<ur_slicer_interfaces::msg::NozzleTemperature>::SharedPtr nozzle_temp_pub_;

  // Slicing Settings
  int nozzle_temp_ = 200;     // Default nozzle temperature
  int infill_percent_ = 20;   // Default infill percentage
  float layer_height_ = 0.2;  // Default layer height
  float print_speed_ = 50.0;  // Default print speed
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PrinterManager>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
