// Copyright 2021 University of Edinburgh
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef SOLO_DASHBOARD__SOLO_DASHBOARD_HPP_
#define SOLO_DASHBOARD__SOLO_DASHBOARD_HPP_

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <QTimer>

#include <memory>

#include "../solo_dashboard/ui_solo_dashboard.h"
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
  virtual void saveSettings(qt_gui_cpp::Settings & plugin_settings,
                            qt_gui_cpp::Settings & instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings & plugin_settings,
                               const qt_gui_cpp::Settings & instance_settings);

private:
  Ui::SoloDashboardWidget ui_;
  QWidget * widget_;
  std::shared_ptr<solo_dashboard::Rosnode> solo_dashboard_node_;
  QTimer * ros_timer_;
  QTimer * display_timer_;

  QString get_pub_onoff();
  QString get_sub_onoff();
  QString get_solo_test_onoff();

private slots:
  void ros_timer_callback();
  void display_timer_callback();

  void set_pub_on();
  void set_pub_off();
  void set_sub_on();
  void set_sub_off();
  void set_solo_test_on();
  void set_solo_test_off();
  void press_button_2_1();
  void press_button_2_2();

  void press_f_button();
  void press_b_button();
  void press_s_button();
  void press_l_button();
  void press_r_button();
};
}  // namespace solo_dashboard
#endif  // SOLO_DASHBOARD__SOLO_DASHBOARD_HPP_
