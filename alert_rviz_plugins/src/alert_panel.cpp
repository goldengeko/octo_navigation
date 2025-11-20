#include "alert_nav_plugins/alert_panel.hpp"
#include <rviz_common/config.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace alert_nav_plugins
{

AlertPanel::AlertPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
  auto v = new QVBoxLayout();

  toggle_btn_ = new QPushButton("Toggle Octomap Updates");
  v->addWidget(toggle_btn_);
  connect(toggle_btn_, &QPushButton::clicked, this, &AlertPanel::onToggleOctomap);

  auto h1 = new QHBoxLayout();
  h1->addWidget(new QLabel("Penalty spread factor"));
  factor_spin_ = new QDoubleSpinBox();
  factor_spin_->setRange(0.0, 1.0);
  factor_spin_->setSingleStep(0.01);
  factor_spin_->setValue(0.25);
  h1->addWidget(factor_spin_);
  connect(factor_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &AlertPanel::onFactorChanged);
  v->addLayout(h1);

  auto h2 = new QHBoxLayout();
  h2->addWidget(new QLabel("Penalty spread radius (m)"));
  radius_spin_ = new QDoubleSpinBox();
  radius_spin_->setRange(0.0, 5.0);
  radius_spin_->setSingleStep(0.01);
  radius_spin_->setValue(0.6);
  h2->addWidget(radius_spin_);
  connect(radius_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &AlertPanel::onRadiusChanged);
  v->addLayout(h2);

  setLayout(v);

  // create rcl node for parameter updates
  try {
    rclcpp::init(0, nullptr);
    we_initialized_rclcpp_ = true;
  } catch (const std::exception & e) {
    // already initialized by rviz / parent process
    we_initialized_rclcpp_ = false;
  }
  rcl_node_ = std::make_shared<rclcpp::Node>("alert_nav_rvizpanel_node");
  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(rcl_node_, param_node_name_);

  // Log so we can see in the rviz terminal that the panel was constructed
  try {
    RCLCPP_INFO(rcl_node_->get_logger(), "AlertPanel constructed and parameter client created for '%s'", param_node_name_.c_str());
  } catch(...) {
    // fallback to cerr if logging subsystem isn't fully up
    std::cerr << "AlertPanel constructed for planner: " << param_node_name_ << std::endl;
  }

  // periodically spin the rcl node in the background to process calls
  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, [this]() { rclcpp::spin_some(rcl_node_); });
  spin_timer_->start(100);
  // read current parameter value asynchronously so we don't block the GUI thread
  std::thread([this]() {
    try {
      auto fut = param_client_->get_parameters({param_key_});
      if (fut.valid()) {
        auto vec = fut.get();
        if (!vec.empty() && vec[0].get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
          octomap_enabled_ = vec[0].as_bool();
        }
      }
    } catch(...) {}
    // update UI on GUI thread
    QMetaObject::invokeMethod(this, "updateButtonUI", Qt::QueuedConnection, Q_ARG(bool, octomap_enabled_));
  }).detach();
}

AlertPanel::~AlertPanel()
{
  try {
    if (we_initialized_rclcpp_) rclcpp::shutdown();
  } catch(...) {}
}

void AlertPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

void AlertPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void AlertPanel::onToggleOctomap()
{
  if (!param_client_) return;
  octomap_enabled_ = !octomap_enabled_;
  std::vector<rclcpp::Parameter> params;
  params.emplace_back(param_key_, octomap_enabled_);
  param_client_->set_parameters(params);
  // update UI immediately
  updateButtonUI(octomap_enabled_);
}

void AlertPanel::onFactorChanged(double v)
{
  if (!param_client_) return;
  std::vector<rclcpp::Parameter> params;
  params.emplace_back(planner_node_name_ + ".penalty_spread_factor", v);
  param_client_->set_parameters(params);
}

void AlertPanel::onRadiusChanged(double v)
{
  if (!param_client_) return;
  std::vector<rclcpp::Parameter> params;
  params.emplace_back(planner_node_name_ + ".penalty_spread_radius", v);
  param_client_->set_parameters(params);
}

void AlertPanel::updateButtonUI(bool enabled)
{
  if (!toggle_btn_) return;
  if (enabled) {
    toggle_btn_->setText("Octomap updates ON");
    toggle_btn_->setStyleSheet("color: white; background-color: green;");
  } else {
    toggle_btn_->setText("Octomap updates OFF");
    toggle_btn_->setStyleSheet("color: white; background-color: red;");
  }
}

} // namespace alert_nav_plugins

PLUGINLIB_EXPORT_CLASS(alert_nav_plugins::AlertPanel, rviz_common::Panel);
