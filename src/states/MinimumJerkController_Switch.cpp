#include "MinimumJerkController_Switch.h"

#include "../MinimumJerkController.h"
#include <Eigen/src/Geometry/Quaternion.h>

void MinimumJerkController_Switch::configure(const mc_rtc::Configuration & config) {}

void MinimumJerkController_Switch::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MinimumJerkController &>(ctl_);
  auto & robot = ctl.robot();

  initPos_ = robot.bodyPosW("FT_sensor_wrench").translation();

  Eigen::Vector3d LQR_Q;
  // W_1 << 1e7, 1e7, 1e7, 1e6, 1e6, 1e6, 1e4, 1e4, 1e4;
  // W_1 << 1e7, 1e7, 1e7, 1e6, 1e6, 1e6, 1e2, 1e2, 1e2;
  // W_1 << 1e6, 1e6, 1e6, 1e5, 1e5, 1e5, 1e2, 1e2, 1e2;
  // Working in Position control
  LQR_Q << 1e7, 1e5, 1e3;

  // W_2 << 1e1, 1e1, 1e1, 1e1, 1e-3, 1e1, 1e1, 1e1;
  // W_2 << 1e0, 1e0, 1e0, 2 * 1e3, 1.2 * 1e3, 3.5 * 1e2, 3.5 * 1e2, 3.5 * 1e2;
  // W_2 << 1e0, 1e0, 1e0, 5e1, 2e1, 1e1, 1e1, 1e1;
  // Working in Position control

  ctl.minJerkTask->LQR_Q(LQR_Q);
  ctl.minJerkTask->LQR_R(10);
  ctl.minJerkTask->W_e(Eigen::Vector3d({1, 1, 1}));
  ctl.minJerkTask->W_u(Eigen::Vector4d(1, 50000, 10000, 10000));
  ctl.minJerkTask->fitts_b(0.32);
  ctl.minJerkTask->fitts_a(-0.09);
  ctl.minJerkTask->react_time(0.0);

  ctl.minJerkTask->setTarget(initPos_ + Eigen::Vector3d(0.1, 0.2, 0.0));
  ctl.compPostureTask->stiffness(100.0);
  ctl.compPostureTask->makeCompliant(false);

  ctl.solver().addTask(ctl.minJerkTask);

  oriTask_ = std::make_shared<mc_tasks::OrientationTask>("FT_sensor_wrench", ctl.robots(), ctl.robot().robotIndex(),
                                                         20.0, 1000.0);
  oriTask_->orientation(Eigen::Quaterniond(-0.5, 0.5, 0.5, 0.5).toRotationMatrix());
  ctl.solver().addTask(oriTask_);

  init_ = true;
  // if(not ctl.datastore().call<bool>("EF_Estimator::isActive"))
  // {
  //   ctl.datastore().call("EF_Estimator::toggleActive");
  // }

  ctl.gui()->addElement({"Controller"}, mc_rtc::gui::Checkbox("Trigger next target", gui_switch_));

  ctl.datastore().assign<std::string>("ControlMode", "Torque");
  mc_rtc::log::success("[MinJerkCtrl] Switched to Switch state - {} controlled",
                       ctl.datastore().get<std::string>("ControlMode"));
}

bool MinimumJerkController_Switch::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MinimumJerkController &>(ctl_);
  if(ctl.minJerkTask->eval().norm() < 0.03 and ctl.minJerkTask->speed().norm() < 0.01 and gui_switch_)
  {
    mc_rtc::log::info("Target reached switching targe | eval = {}, speed = {}", ctl.minJerkTask->eval().norm(),
                      ctl.minJerkTask->speed().norm());
    if(init_)
    {
      ctl.minJerkTask->setTarget(initPos_ + Eigen::Vector3d(0.1, -0.2, 0.0));
      init_ = false;
    }
    else
    {
      ctl.minJerkTask->setTarget(initPos_ + Eigen::Vector3d(0.1, 0.2, 0.0));
      init_ = true;
    }
  }
  else
  {
    // mc_rtc::log::info("Target not reached yet");
  }

  return false;
}

void MinimumJerkController_Switch::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MinimumJerkController &>(ctl_);
}

EXPORT_SINGLE_STATE("MinimumJerkController_Switch", MinimumJerkController_Switch)
