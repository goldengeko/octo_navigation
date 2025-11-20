#pragma once

#include <rviz_common/panel.hpp>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QTimer>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace alert_nav_plugins
{

class AlertPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit AlertPanel(QWidget* parent = nullptr);
  virtual ~AlertPanel();

  // load and save config
  virtual void load(const rviz_common::Config& config) override;
  virtual void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onToggleOctomap();
  void onFactorChanged(double v);
  void onRadiusChanged(double v);
  void updateButtonUI(bool enabled);

private:
  QPushButton* toggle_btn_ = nullptr;
  QDoubleSpinBox* factor_spin_ = nullptr;
  QDoubleSpinBox* radius_spin_ = nullptr;

  // rclcpp node and parameter client
  // rclcpp node and parameter client
  rclcpp::Node::SharedPtr rcl_node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;
  // planner node (kept for other parameter operations), and remote node to control
  std::string planner_node_name_ = "octo_planner";
  // node and parameter key used for toggling octomap updates in the robot stack
  std::string param_node_name_ = "/move_base_flex";
  std::string param_key_ = "octo_planner.enable_octomap_updates";
  bool octomap_enabled_ = true;
  QTimer* spin_timer_ = nullptr;
  // true if this panel called rclcpp::init()
  bool we_initialized_rclcpp_ = false;
};

} // namespace alert_nav_plugins
