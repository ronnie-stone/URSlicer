#include "slicer_panel/panel.hpp"
#include <QMessageBox>
#include <QDoubleValidator>

namespace slicer_panel
{

Slicer::Slicer(QWidget* parent) : rviz_common::Panel(parent), rclcpp::Node("slicer_panel")
{
  QTabWidget* tabWidget = new QTabWidget(this);

  // Object Selection Tab
  QWidget* slicingWidget = new QWidget();
  QVBoxLayout* mainLayout = new QVBoxLayout;

  // Hotend status
  QHBoxLayout* hotendLayout = new QHBoxLayout;
  hotend_status_label = new QLabel("Hotend Status: OFF");
  hotend_status_label->setStyleSheet("color: green");
  hotendLayout->addWidget(hotend_status_label);
  mainLayout->addLayout(hotendLayout);

  // File Selection
  QHBoxLayout* layout = new QHBoxLayout;
  file_button_ = new QPushButton("Select File");
  layout->addWidget(file_button_);
  path_label_ = new QLabel("No file selected");
  layout->addWidget(path_label_);
  mainLayout->addLayout(layout);

  mainLayout->addStretch();

  // Slicing preheat and clear workspace buttons
  QVBoxLayout* buttonLayout = new QVBoxLayout;
  slice_button_ = new QPushButton("Slice");
  buttonLayout->addWidget(slice_button_);
  slice_button_->setEnabled(false);
  visualize_button_ = new QPushButton("Visualize");
  buttonLayout->addWidget(visualize_button_);
  visualize_button_->setEnabled(false);
  preheat_button_ = new QPushButton("Preheat Hotend");
  buttonLayout->addWidget(preheat_button_);
  preheat_button_->setEnabled(true);
  clear_ws_button_ = new QPushButton("Clear Workspace");
  buttonLayout->addWidget(clear_ws_button_);
  clear_ws_button_->setEnabled(true);

  mainLayout->addLayout(buttonLayout);
  slicingWidget->setLayout(mainLayout);

  // Slicing Settings Tab
  QWidget* settingsTabWidget = new QWidget();
  QVBoxLayout* advancedLayout = new QVBoxLayout;

  // Validators
  QDoubleValidator* decimalValidator = new QDoubleValidator(0.0, 1000.0, 2, this);
  decimalValidator->setNotation(QDoubleValidator::StandardNotation);

  QIntValidator* intValidator = new QIntValidator(0, 100, this);

  // Layer Height
  layer_height_label_ = new QLabel("Layer Height (mm)");
  advancedLayout->addWidget(layer_height_label_);
  layer_height_input_ = new QLineEdit("0.4");
  layer_height_input_->setValidator(decimalValidator);
  connect(layer_height_input_, &QLineEdit::editingFinished, this, &Slicer::validateNumericInput);
  advancedLayout->addWidget(layer_height_input_);

  // Infill Density
  infill_density_label_ = new QLabel("Infill Density (%)");
  advancedLayout->addWidget(infill_density_label_);
  infill_density_input_ = new QLineEdit("100");
  infill_density_input_->setValidator(intValidator);
  connect(infill_density_input_, &QLineEdit::editingFinished, this, &Slicer::validateNumericInput);
  advancedLayout->addWidget(infill_density_input_);

  // Temperature
  temperature_label_ = new QLabel("Temperature (°C)");
  advancedLayout->addWidget(temperature_label_);
  temperature_input_ = new QLineEdit("255");
  temperature_input_->setValidator(intValidator);
  connect(temperature_input_, &QLineEdit::editingFinished, this, &Slicer::validateNumericInput);
  advancedLayout->addWidget(temperature_input_);

  // Print Speed
  print_speed_label_ = new QLabel("Print Speed (mm/s)");
  advancedLayout->addWidget(print_speed_label_);
  print_speed_input_ = new QLineEdit("50.0");
  print_speed_input_->setValidator(decimalValidator);
  connect(print_speed_input_, &QLineEdit::editingFinished, this, &Slicer::validateNumericInput);
  advancedLayout->addWidget(print_speed_input_);

  // Print Bed Adhesion
  bed_adhesion_label_ = new QLabel("Print Bed Adhesion");
  advancedLayout->addWidget(bed_adhesion_label_);
  bed_adhesion_combo_ = new QComboBox();
  bed_adhesion_combo_->addItems({ "None", "Skirt", "Brim", "Raft" });
  advancedLayout->addWidget(bed_adhesion_combo_);

  // Infill Pattern
  infill_pattern_label_ = new QLabel("Infill Pattern");
  advancedLayout->addWidget(infill_pattern_label_);
  infill_pattern_combo_ = new QComboBox();
  infill_pattern_combo_->addItems({ "Triangle", "Gyroid", "Cubic" });
  advancedLayout->addWidget(infill_pattern_combo_);

  settingsTabWidget->setLayout(advancedLayout);
  tabWidget->addTab(slicingWidget, "Slicing");
  tabWidget->addTab(settingsTabWidget, "Settings");

  QVBoxLayout* panelLayout = new QVBoxLayout;
  panelLayout->addWidget(tabWidget);
  setLayout(panelLayout);

  connect(file_button_, &QPushButton::clicked, this, &Slicer::selectFileClicked);
  connect(slice_button_, SIGNAL(clicked()), this, SLOT(sliceClicked()));
  connect(visualize_button_, SIGNAL(clicked()), this, SLOT(visualizeClicked()));
  connect(preheat_button_, SIGNAL(clicked()), this, SLOT(preheatClicked()));
  connect(clear_ws_button_, SIGNAL(clicked()), this, SLOT(clearWSClicked()));
}

void Slicer::onInitialize()
{
  // Interactive marker server for STL objects
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "stl_marker", this->get_node_base_interface(), this->get_node_clock_interface(),
      this->get_node_logging_interface(), this->get_node_topics_interface(), this->get_node_services_interface());

  bed_subscriber_ = this->create_subscription<ur_slicer_interfaces::msg::BedCorners>(
      "/bed_corners", 1, std::bind(&Slicer::rectangleBedCreation, this, std::placeholders::_1));

  // Printer Manager Action Client
  printer_client_ = rclcpp_action::create_client<ur_slicer_interfaces::action::PreparePrinter>(this, "/ur5e_printer");

  // Heater Control Service Client
  heater_client_ = create_client<ur_slicer_interfaces::srv::HeaterControl>("heater_control");

  // Settings Publisher
  settings_pub_ = this->create_publisher<ur_slicer_interfaces::msg::SlicerSettings>("/slicer_settings", 1);

  // Set up timer for spinning the node
  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, &Slicer::spin);
  spin_timer_->start(10);  // Spin every 10ms
  // Publish default settings
  publishSettings();
}

void Slicer::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

void Slicer::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void Slicer::spin()
{
  rclcpp::spin_some(this->get_node_base_interface());
}

// Bed Creation Functions

void Slicer::testBedCreation()
{
  deleteBed();
  geometry_msgs::msg::Point p1, p2, p3, p4;

  RCLCPP_INFO(this->get_logger(), "Received bed corners");

  p1.x = 0.25;
  p1.y = -0.25;
  p1.z = -0.415;

  p2.x = 0.25;
  p2.y = -0.75;
  p2.z = -0.415;

  p3.x = -0.25;
  p3.y = -0.75;
  p3.z = -0.415;

  p4.x = -0.25;
  p4.y = -0.25;
  p4.z = -0.415;

  float bed_height = 0.1;  // Arbitrary bed height for visualization

  // Calculate center position for bed object
  geometry_msgs::msg::Point center;
  center.x = (p1.x + p2.x + p3.x + p4.x) / 4;
  center.y = (p1.y + p2.y + p3.y + p4.y) / 4;
  center.z = (p1.z + p2.z + p3.z + p4.z) / 4 - bed_height / 2;  // Ensures top surface at correct height

  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "ur5e_base_link";  // UR5e base frame
  int_marker.pose.position = center;
  int_marker.scale = 1.0;
  int_marker.name = "printer_bed";

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;

  // Create bed visualization
  visualization_msgs::msg::Marker bed_visual;
  bed_visual.type = visualization_msgs::msg::Marker::CUBE;
  bed_visual.color.r = 0.8;
  bed_visual.color.g = 0.8;
  bed_visual.color.b = 0.8;
  bed_visual.color.a = 1;

  // Calculate bed dimensions
  bed_visual.scale.x =
      std::max({ std::abs(p1.x - p2.x), std::abs(p2.x - p3.x), std::abs(p3.x - p4.x), std::abs(p4.x - p1.x) });
  bed_visual.scale.y =
      std::max({ std::abs(p1.y - p2.y), std::abs(p2.y - p3.y), std::abs(p3.y - p4.y), std::abs(p4.y - p1.y) });
  bed_visual.scale.z = bed_height;

  control.markers.push_back(bed_visual);
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->applyChanges();
}

void Slicer::rectangleBedCreation(const ur_slicer_interfaces::msg::BedCorners::SharedPtr msg)
{
  deleteBed();
  geometry_msgs::msg::Point p1, p2, p3, p4;

  RCLCPP_INFO(this->get_logger(), "Received bed corners");

  p1 = msg->corners[0];
  p2 = msg->corners[1];
  p3 = msg->corners[2];
  p4 = msg->corners[3];

  float bed_height = 0.1;  // Arbitrary bed height for visualization

  // Calculate center position for bed object
  geometry_msgs::msg::Point center;
  center.x = (p1.x + p2.x + p3.x + p4.x) / 4;
  center.y = (p1.y + p2.y + p3.y + p4.y) / 4;
  center.z = (p1.z + p2.z + p3.z + p4.z) / 4 - bed_height / 2;  // Ensures top surface at correct height

  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "ur5e_base_link";  // UR5e base frame
  int_marker.pose.position = center;
  int_marker.scale = 1.0;
  int_marker.name = "printer_bed";

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;

  // Create bed visualization
  visualization_msgs::msg::Marker bed_visual;
  bed_visual.type = visualization_msgs::msg::Marker::CUBE;
  bed_visual.color.r = 0.8;
  bed_visual.color.g = 0.8;
  bed_visual.color.b = 0.8;
  bed_visual.color.a = 1;

  // Calculate bed dimensions
  bed_visual.scale.x =
      std::max({ std::abs(p1.x - p2.x), std::abs(p2.x - p3.x), std::abs(p3.x - p4.x), std::abs(p4.x - p1.x) });
  bed_visual.scale.y =
      std::max({ std::abs(p1.y - p2.y), std::abs(p2.y - p3.y), std::abs(p3.y - p4.y), std::abs(p4.y - p1.y) });
  bed_visual.scale.z = bed_height;

  control.markers.push_back(bed_visual);
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->applyChanges();
}

void Slicer::deleteBed()
{
  server_->erase("printer_bed");
  server_->applyChanges();
}

// STL Marker Functions

void Slicer::createSTLMarker()
{
  deleteSTLMarker();
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "ur5e_base_link";
  int_marker.name = "stl_marker";
  int_marker.description = "";
  int_marker.scale = 1.0;

  // Create stl marker
  visualization_msgs::msg::Marker stl;
  stl.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  stl.mesh_resource = "file://" + path_label_->text().toStdString();
  stl.ns = "stl_marker";
  // stl.type = visualization_msgs::msg::Marker::ARROW;
  stl.scale.x = 0.001;
  stl.scale.y = 0.001;
  stl.scale.z = 0.001;
  stl.color.r = 0.5;
  stl.color.g = 0.5;
  stl.color.b = 0.5;
  stl.color.a = 1.0;
  stl.id = 8;

  // Create a control for the stl
  visualization_msgs::msg::InteractiveMarkerControl stl_control;
  stl_control.always_visible = true;
  stl_control.markers.push_back(stl);
  int_marker.controls.push_back(stl_control);

  // Create controls for movement and rotation
  visualization_msgs::msg::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server_->insert(int_marker, std::bind(&Slicer::processSTLFeedback, this, std::placeholders::_1));
  server_->applyChanges();
  testBedCreation();
}

void Slicer::deleteSTLMarker()
{
  server_->erase("stl_marker");
  server_->applyChanges();
}

void Slicer::processSTLFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  switch (feedback->event_type)
  {
    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
      RCLCPP_INFO(
          this->get_logger(), "STL marker moved to position (%.2f, %.2f, %.2f) and orientation(%.2f, %.2f, %.2f, %.2f)",
          feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z, feedback->pose.orientation.x,
          feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w);
      server_->setPose(feedback->marker_name, feedback->pose);
      server_->applyChanges();
      break;
  }
}

// Main UI Callback Functions

void Slicer::selectFileClicked()
{
  clearWSClicked();
  QString path = QFileDialog::getOpenFileName(this,                // Parent widget
                                              "Select STL File",   // Dialog title
                                              QDir::homePath(),    // Default directory
                                              "STL Files (*.stl)"  // Filters
  );

  if (!path.isEmpty())
  {
    path_label_->setText(path);
    // Add custom logic to process the file
    slice_button_->setEnabled(true);
    createSTLMarker();
  }
}

void Slicer::sliceClicked()
{
  // Slicing Code
  visualize_button_->setEnabled(true);

  // Publish current settings
  auto settings = ur_slicer_interfaces::msg::SlicerSettings();
  settings.nozzle_temperature = 200;
  settings.infill_density = 20;
  settings.layer_height = 0.2;
  settings.print_speed = 50.0;
  settings_pub_->publish(settings);
  RCLCPP_INFO(this->get_logger(), "Slicing settings published");

  if (!printer_client_->wait_for_action_server(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(this->get_logger(), "Printer action server not available");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Printer action server available");
  auto goal = ur_slicer_interfaces::action::PreparePrinter::Goal();
  goal.filepath = path_label_->text().toStdString();
  goal.preheat = true;
  goal.origin_pose = geometry_msgs::msg::Pose();

  auto send_goal_options = rclcpp_action::Client<ur_slicer_interfaces::action::PreparePrinter>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&Slicer::slicing_confirmed, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&Slicer::slicing_feedback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&Slicer::slicing_result, this, std::placeholders::_1);
  auto result_future = printer_client_->async_send_goal(goal, send_goal_options);
  // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
  //     rclcpp::FutureReturnCode::SUCCESS)
  // {
  //   RCLCPP_ERROR(this->get_logger(), "Failed to send slicing request");
  //   return;
  // }
  RCLCPP_INFO(this->get_logger(), "Slicing request sent");
}

void Slicer::visualizeClicked()
{
  // Show motion planning once slicing has been done
  RCLCPP_INFO(this->get_logger(), "Visualizing motion planning");
}

void Slicer::preheatClicked()
{
  // Check Heater Server
  if (!heater_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(this->get_logger(), "Heater service not available");
    return;
  }

  if (heater_on_)
  {
    RCLCPP_INFO(this->get_logger(), "Turning off heater");
    auto request = std::make_shared<ur_slicer_interfaces::srv::HeaterControl::Request>();
    request->heater_on = false;
    request->heater_off = true;
    request->desired_temperature = 0;
    auto response = heater_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), response) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      if (response.get()->control_confirmed)
      {
        heater_on_ = false;
        RCLCPP_INFO(this->get_logger(), "Heater turned off");
        hotend_status_label->setText("Hotend Status: OFF");
        hotend_status_label->setStyleSheet("color: green");
        preheat_button_->setText("Preheat Hotend");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to turn off heater");
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Heater service failed to respond");
    }
  }
  else
  {
    auto request = std::make_shared<ur_slicer_interfaces::srv::HeaterControl::Request>();
    request->heater_on = true;
    request->heater_off = false;
    request->desired_temperature = temperature_input_->text().toInt();
    auto response = heater_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), response) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      if (response.get()->control_confirmed)
      {
        heater_on_ = true;
        RCLCPP_INFO(this->get_logger(), "Heater turned on");
        hotend_status_label->setText("Hotend Status: ON");
        hotend_status_label->setStyleSheet("color: red");
        preheat_button_->setText("Turn Off Hotend");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to turn on heater");
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Heater service failed to respond");
    }
  }
}

void Slicer::clearWSClicked()
{
  // Clear file selection
  path_label_->setText("No file selected");

  // Disable Buttons
  slice_button_->setEnabled(false);
  visualize_button_->setEnabled(false);

  // Remove interactive marker object to set up creation of new one
  deleteSTLMarker();
}

// Settings Related Functions
void Slicer::validateNumericInput()
{
  // Validate numeric input
  bool valid = true;
  if (layer_height_input_->text().isEmpty() || layer_height_input_->text().toDouble(&valid) <= 0.0 ||
      layer_height_input_->text().toDouble(&valid) > 1.0)
  {
    QMessageBox::warning(this, "Invalid Input", "Layer height must be a positive number less than 1.");
    layer_height_input_->clear();
    return;
  }
  if (infill_density_input_->text().isEmpty() || infill_density_input_->text().toInt(&valid) < 0 ||
      infill_density_input_->text().toInt(&valid) > 100)
  {
    QMessageBox::warning(this, "Invalid Input", "Infill density must be between 0 and 100.");
    infill_density_input_->clear();
    return;
  }
  if (temperature_input_->text().isEmpty() || temperature_input_->text().toInt(&valid) <= 0 ||
      temperature_input_->text().toInt(&valid) > 300)
  {
    QMessageBox::warning(this, "Invalid Input", "Temperature must be a positive number less than 300.");
    temperature_input_->clear();
    return;
  }
  if (print_speed_input_->text().isEmpty() || print_speed_input_->text().toDouble(&valid) <= 0.0 ||
      print_speed_input_->text().toDouble(&valid) > 100.0)
  {
    QMessageBox::warning(this, "Invalid Input", "Print speed must be a positive number less than 100.");
    print_speed_input_->clear();
    return;
  }
  publishSettings();
  RCLCPP_INFO(this->get_logger(), "Settings validated and published");
}

void Slicer::publishSettings()
{
  // Publish the settings to the topic
  ur_slicer_interfaces::msg::SlicerSettings settings;
  settings.nozzle_temperature = temperature_input_->text().toInt();
  settings.infill_density = infill_density_input_->text().toInt();
  settings.layer_height = layer_height_input_->text().toDouble();
  settings.print_speed = print_speed_input_->text().toDouble();
  settings_pub_->publish(settings);
}

// Printing Manager Action Response Functions

void Slicer::slicing_confirmed(
    const rclcpp_action::ClientGoalHandle<ur_slicer_interfaces::action::PreparePrinter>::SharedPtr goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Slicing request was rejected");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Slicing request accepted");
}

void Slicer::slicing_feedback(
    const rclcpp_action::ClientGoalHandle<ur_slicer_interfaces::action::PreparePrinter>::SharedPtr goal_handle,
    const std::shared_ptr<const ur_slicer_interfaces::action::PreparePrinter::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Slicing progress:");
}

void Slicer::slicing_result(
    const rclcpp_action::ClientGoalHandle<ur_slicer_interfaces::action::PreparePrinter>::WrappedResult& result)
{
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO(this->get_logger(), "Slicing completed successfully");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Slicing failed");
  }
}

std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

}  // namespace slicer_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(slicer_panel::Slicer, rviz_common::Panel)
