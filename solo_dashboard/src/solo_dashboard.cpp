// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include <QStringList>

#include <memory>
#include <thread>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "solo_dashboard/solo_dashboard.hpp"

namespace solo_dashboard
{
SoloDashboard::SoloDashboard()
: rqt_gui_cpp::Plugin(),
  widget_(0),
  solo_dashboard_node_(std::make_shared<solo_dashboard::SoloDashboardNode>())
{
  setObjectName("Solo Dashboard");
}

void SoloDashboard::initPlugin(qt_gui_cpp::PluginContext & context)
{
  // Access standalone command line arguments
  QStringList argv = context.argv();
  // Create QWidget
  widget_ = new QWidget();
  // Extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // Add widget to the user interface
  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(
      widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  // Set states
  connect(ui_.pub_on_button, SIGNAL(clicked(bool)), this, SLOT(set_pub_on()));
  connect(ui_.pub_off_button, SIGNAL(clicked(bool)), this, SLOT(set_pub_off()));
  connect(ui_.sub_on_button, SIGNAL(clicked(bool)), this, SLOT(set_sub_on()));
  connect(ui_.sub_off_button, SIGNAL(clicked(bool)), this, SLOT(set_sub_off()));

  connect(ui_.button_1_1, SIGNAL(clicked(bool)), this, SLOT(press_button_1_1()));
  connect(ui_.button_1_2, SIGNAL(clicked(bool)), this, SLOT(press_button_1_2()));
  connect(ui_.button_2_1, SIGNAL(clicked(bool)), this, SLOT(press_button_2_1()));
  connect(ui_.button_2_2, SIGNAL(clicked(bool)), this, SLOT(press_button_2_2()));
  connect(ui_.button_3_1, SIGNAL(clicked(bool)), this, SLOT(press_button_3_1()));
  connect(ui_.button_3_2, SIGNAL(clicked(bool)), this, SLOT(press_button_3_2()));
  connect(ui_.button_4_1, SIGNAL(clicked(bool)), this, SLOT(press_button_4_1()));
  connect(ui_.button_4_2, SIGNAL(clicked(bool)), this, SLOT(press_button_4_2()));
  connect(ui_.button_5_1, SIGNAL(clicked(bool)), this, SLOT(press_button_5_1()));
  connect(ui_.button_5_2, SIGNAL(clicked(bool)), this, SLOT(press_button_5_2()));
  connect(ui_.button_6_1, SIGNAL(clicked(bool)), this, SLOT(press_button_6_1()));
  connect(ui_.button_6_2, SIGNAL(clicked(bool)), this, SLOT(press_button_6_2()));
  connect(ui_.button_7_1, SIGNAL(clicked(bool)), this, SLOT(press_button_7_1()));
  connect(ui_.button_7_2, SIGNAL(clicked(bool)), this, SLOT(press_button_7_2()));

  connect(ui_.f_button, SIGNAL(clicked(bool)), this, SLOT(press_f_button()));
  connect(ui_.b_button, SIGNAL(clicked(bool)), this, SLOT(press_b_button()));
  connect(ui_.s_button, SIGNAL(clicked(bool)), this, SLOT(press_s_button()));
  connect(ui_.l_button, SIGNAL(clicked(bool)), this, SLOT(press_l_button()));
  connect(ui_.r_button, SIGNAL(clicked(bool)), this, SLOT(press_r_button()));

  // Run ros spin
  ros_timer_ = new QTimer(this);
  connect(ros_timer_, SIGNAL(timeout()), this, SLOT(ros_timer_callback()));
  ros_timer_->start(10);

  // Get & Display states
  display_timer_ = new QTimer(this);
  connect(display_timer_, SIGNAL(timeout()), this, SLOT(display_timer_callback()));
  display_timer_->start(10);
}

void SoloDashboard::shutdownPlugin()
{
  // TODO(my_username): unregister all publishers here
}

void SoloDashboard::saveSettings(
  qt_gui_cpp::Settings & plugin_settings,
  qt_gui_cpp::Settings & instance_settings) const
{
  // TODO(my_username): save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
  (void)plugin_settings;
  (void)instance_settings;
}

void SoloDashboard::restoreSettings(
  const qt_gui_cpp::Settings & plugin_settings,
  const qt_gui_cpp::Settings & instance_settings)
{
  // TODO(my_username): restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
  (void)plugin_settings;
  (void)instance_settings;
}

void SoloDashboard::ros_timer_callback()
{
  rclcpp::spin_some(solo_dashboard_node_);
}

void SoloDashboard::display_timer_callback()
{
  ui_.pub_onoff_state->setText(get_pub_onff());
  ui_.sub_onoff_state->setText(get_sub_onff());
}

QString SoloDashboard::get_pub_onff()
{
  QString q_str;
  if (solo_dashboard_node_->pub_onoff_ == true) {
    q_str = "on";
  } else {
    q_str = "off";
  }

  return q_str;
}

QString SoloDashboard::get_sub_onff()
{
  QString q_str;
  if (solo_dashboard_node_->sub_onoff_ == true) {
    q_str = "on";
  } else {
    q_str = "off";
  }

  return q_str;
}

void SoloDashboard::set_pub_on()
{
  solo_dashboard_node_->pub_onoff_ = true;
}

void SoloDashboard::set_pub_off()
{
  solo_dashboard_node_->pub_onoff_ = false;
}

void SoloDashboard::set_sub_on()
{
  solo_dashboard_node_->sub_onoff_ = true;
}

void SoloDashboard::set_sub_off()
{
  solo_dashboard_node_->sub_onoff_ = false;
}

void SoloDashboard::press_f_button()
{
  solo_dashboard_node_->lin_vel_ = 0.01;
}

void SoloDashboard::press_b_button()
{
  solo_dashboard_node_->lin_vel_ = -0.01;
}

void SoloDashboard::press_s_button()
{
  solo_dashboard_node_->lin_vel_ = 0.0;
  solo_dashboard_node_->ang_vel_ = 0.0;
}

void SoloDashboard::press_l_button()
{
  solo_dashboard_node_->ang_vel_ = 0.01;
}

void SoloDashboard::press_r_button()
{
  solo_dashboard_node_->ang_vel_ = -0.01;
}
}  // namespace solo_dashboard
PLUGINLIB_EXPORT_CLASS(solo_dashboard::SoloDashboard, rqt_gui_cpp::Plugin)
