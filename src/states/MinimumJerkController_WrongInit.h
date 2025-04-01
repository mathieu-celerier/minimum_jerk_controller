#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/OrientationTask.h>

struct MinimumJerkController_WrongInit : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  Eigen::Vector3d initPos_;
  bool init_;
  bool gui_switch_;

  std::shared_ptr<mc_tasks::OrientationTask> oriTask_;
};
