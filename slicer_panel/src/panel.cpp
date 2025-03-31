#include "slicer_panel/panel.hpp"

namespace slicer_panel
{

  

Slicer::Slicer(QWidget* parent) : rviz_common::Panel(parent), rclcpp::Node("slicer_panel")
{
  QTabWidget* tabWidget = new QTabWidget(this);

  // Object Selection Tab
  QWidget* slicingWidget = new QWidget();
  QVBoxLayout* mainLayout = new QVBoxLayout;

  QVBoxLayout* dynamicContentLayout = new QVBoxLayout;
  dynamicContentLayout->setSpacing(10);  // Set spacing between dynamic boxes

  // File selection
  QHBoxLayout* layout = new QHBoxLayout;
  file_button_ = new QPushButton("Select File");
  layout->addWidget(file_button_);
  path_label_ = new QLabel("No file selected");
  layout->addWidget(path_label_);
  mainLayout->addLayout(layout);
  
  // Layer height input
  layer_height_label_ = new QLabel("Layer Height (mm)");
  layer_height_input_ = new QLineEdit;
  layer_height_input_->setPlaceholderText("Layer Height (mm)");
  mainLayout->addWidget(layer_height_input_);


  // Infill density input
  infill_density_input_ = new QLineEdit;
  infill_density_input_->setPlaceholderText("Infill Density (%)");
  mainLayout->addWidget(infill_density_input_);


  // Temperature settings input
  temperature_input_ = new QLineEdit;
  temperature_input_->setPlaceholderText("Temperature (°C)");
  mainLayout->addWidget(temperature_input_);


  // Print bed adhesion dropdown
  bed_adhesion_combo_ = new QComboBox;
  bed_adhesion_combo_->addItems({"None", "Brim", "Raft", "Skirt"});
  mainLayout->addWidget(bed_adhesion_combo_);


  // Infill pattern dropdown
  infill_pattern_combo_ = new QComboBox;
  infill_pattern_combo_->addItems({"Grid", "Honeycomb", "Lines", "Cubic"});
  mainLayout->addWidget(infill_pattern_combo_);


  // Print speed input
  print_speed_input_ = new QLineEdit;
  print_speed_input_->setPlaceholderText("Print Speed (mm/s)");
  mainLayout->addWidget(print_speed_input_);

  // Spacer to push buttons to the bottom

  mainLayout->addStretch();

  // Slice, Visualize, Export, Clear Buttons

  QVBoxLayout* buttonLayout = new QVBoxLayout;
  slice_button_ = new QPushButton("Slice");
  buttonLayout->addWidget(slice_button_);
  slice_button_->setEnabled(false);
  visualize_button_ = new QPushButton("Visualize");
  buttonLayout->addWidget(visualize_button_);
  visualize_button_->setEnabled(false);
  export_button_ = new QPushButton("Export");
  buttonLayout->addWidget(export_button_);
  export_button_->setEnabled(false);
  clear_ws_button_ = new QPushButton("Clear Workspace");
  buttonLayout->addWidget(clear_ws_button_);
  clear_ws_button_->setEnabled(true);

  mainLayout->addLayout(buttonLayout);

  slicingWidget->setLayout(mainLayout);

  // Slicing Settings Tab

  QWidget* settingsTabWidget = new QWidget();
  QVBoxLayout* advancedLayout = new QVBoxLayout;

  settingsTabWidget->setLayout(advancedLayout);

  tabWidget->addTab(slicingWidget, "Slicing");
  tabWidget->addTab(settingsTabWidget, "Settings");

  QVBoxLayout* panelLayout = new QVBoxLayout;
  panelLayout->addWidget(tabWidget);
  setLayout(panelLayout);

  connect(file_button_, &QPushButton::clicked, this, &Slicer::selectFileClicked);
  connect(slice_button_, SIGNAL(clicked()), this, SLOT(sliceClicked()));
  connect(visualize_button_, SIGNAL(clicked()), this, SLOT(visualizeClicked()));
  connect(export_button_, SIGNAL(clicked()), this, SLOT(exportClicked()));
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

  // Set up timer for spinning the node
  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, &Slicer::spin);
  spin_timer_->start(10);  // Spin every 10ms
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

void Slicer::rectangleBedCreation(const ur_slicer_interfaces::msg::BedCorners::SharedPtr msg)
{
  deleteBed();
  geometry_msgs::msg::Point p1, p2, p3, p4;

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
  int_marker.header.frame_id = "base_link";  // UR5e base frame
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
  int_marker.header.frame_id = "base_link";
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

// UI Callback Functions

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
  export_button_->setEnabled(true);
}

void Slicer::visualizeClicked()
{
  // Show motion planning once slicing has been done
}

void Slicer::exportClicked()
{
  // Export however is determined
}

void Slicer::clearWSClicked()
{
  // Clear file selection
  path_label_->setText("No file selected");

  // Disable Buttons
  slice_button_->setEnabled(false);
  visualize_button_->setEnabled(false);
  export_button_->setEnabled(false);

  // Remove interactive marker object to set up creation of new one
  deleteSTLMarker();
}

std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

}  // namespace slicer_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(slicer_panel::Slicer, rviz_common::Panel)
