// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#ifndef SOLO_DASHBOARD__SOLO_DASHBOARD_HPP_
#define SOLO_DASHBOARD__SOLO_DASHBOARD_HPP_

#include <rqt_gui_cpp/plugin.h>
#include "ui_solo_dashboard.h"  // TODO(JaehyunShim): doesn't work...
#include <QWidget>
#include <QTimer>

#include <memory>

#include "solo_dashboard/rosnode.hpp"

namespace solo_dashboard
{
class SoloDashboard : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  SoloDashboard();

  virtual void initPlugin(qt_gui_cpp::PluginContext & context);
  virtual void shutdownPlugin();
  virtual void saveSettings(
    qt_gui_cpp::Settings & plugin_settings,
    qt_gui_cpp::Settings & instance_settings) const;
  virtual void restoreSettings(
    const qt_gui_cpp::Settings & plugin_settings,
    const qt_gui_cpp::Settings & instance_settings);

private:
  Ui::SoloDashboardWidget ui_;
  QWidget * widget_;
  std::shared_ptr<solo_dashboard::RosNode> solo_dashboard_node_;
  QTimer * ros_timer_;
  QTimer * display_timer_;

  QString get_pub_onff();
  QString get_sub_onff();

private slots:
  void ros_timer_callback();
  void display_timer_callback();

  void set_pub_on();
  void set_pub_off();
  void set_sub_on();
  void set_sub_off();

  void press_button_1_1();
  void press_button_1_2();
  void press_button_2_1();
  void press_button_2_2();
  void press_button_3_1();
  void press_button_3_2();
  void press_button_4_1();
  void press_button_4_2();
  void press_button_5_1();
  void press_button_5_2();
  void press_button_6_1();
  void press_button_6_2();
  void press_button_7_1();
  void press_button_7_2();

  void press_f_button();
  void press_b_button();
  void press_s_button();
  void press_l_button();
  void press_r_button();
};
}  // namespace solo_dashboard
#endif  // SOLO_DASHBOARD__SOLO_DASHBOARD_HPP_
