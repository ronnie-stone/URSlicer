#ifndef RVIZ_UI_PANEL_PANEL_HPP_
#define RVIZ_UI_PANEL_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <memory>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "ur_slicer_interfaces/msg/bed_corners.hpp"
#include "ur_slicer_interfaces/msg/path.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QLineEdit>
#include <QString>
#include <QTimer>
#include <QFileDialog>
#include <QDir>
#include <vector>
#include <string>

class QLabel;
class QComboBox;
class QPushButton;
class QLineEdit;
class QTimer;
class QFileDialog;

namespace slicer_panel
{

class Slicer : public rviz_common::Panel, public rclcpp::Node
{
  Q_OBJECT
public:
  explicit Slicer(QWidget* parent = nullptr);

  virtual void onInitialize() override;

  virtual void load(const rviz_common::Config& config) override;
  virtual void save(rviz_common::Config config) const override;

protected Q_SLOTS:
  void spin();
  void rectangleBedCreation(const ur_slicer_interfaces::msg::BedCorners::SharedPtr msg);
  void deleteBed();
  void createSTLMarker();
  void deleteSTLMarker();
  void processSTLFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);
  void selectFileClicked();
  void sliceClicked();
  void visualizeClicked();
  void exportClicked();
  void clearWSClicked();

private:
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;  // Interactive marker server for STL objects
  rclcpp::Subscription<ur_slicer_interfaces::msg::BedCorners>::SharedPtr bed_subscriber_;  // Subscriber for bed corners

  QTimer* spin_timer_;

  QPushButton* file_button_;
  QLabel* path_label_;
  QPushButton* slice_button_;
  QPushButton* visualize_button_;
  QPushButton* export_button_;
  QPushButton* clear_ws_button_;
};

}  // namespace slicer_panel

#endif  // RVIZ_UI_PANEL_PANEL_HPP_
