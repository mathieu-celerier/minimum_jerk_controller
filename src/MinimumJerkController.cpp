#include "MinimumJerkController.h"

MinimumJerkController::MinimumJerkController(mc_rbdyn::RobotModulePtr rm,
                                             double dt,
                                             const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  solver().removeConstraintSet(dynamicsConstraint);
  // Initialize the constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.8, 70.0});
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, 0.0, 1.8, 70.0}, 0.9, true));
  solver().addConstraintSet(dynamicsConstraint);

  solver().removeTask(getPostureTask(robot().name()));
  compPostureTask = std::make_shared<mc_tasks::CompliantPostureTask>(solver(), robot().robotIndex(), 1, 1);
  compPostureTask->reset();
  compPostureTask->stiffness(1.0);
  compPostureTask->damping(2.0);
  solver().addTask(compPostureTask);

  // // Check of the Inertia matrix with Rotor Inertia
  // rbd::ForwardDynamics FD(robot().mb());
  // FD.computeH(robot().mb(), robot().mbc());
  // auto HIr = FD.HIr();
  // auto H = FD.H();
  //
  // Eigen::IOFormat format(2, 0, "& ", "\\\\\n", "", "", "", "");
  // std::cout << "---------------------------------\n";
  // std::cout << HIr.format(format) << "\n";
  // std::cout << "---------------------------------\n";
  // std::cout << H.format(format) << "\n";
  // std::cout << "---------------------------------\n";
  // // End

  minJerkTask = std::make_shared<mc_tasks::MinimumJerkTask>("DS4_tool", robots(), robot().robotIndex(), 10000.0);
  // ctl.solver().addTask(ctl.minJerkTask);

  datastore().make<std::string>("ControlMode", "Position");
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return compPostureTask; });

  // Add log entries
  logger().addLogEntry("ControlMode",
                       [this]()
                       {
                         auto mode = datastore().get<std::string>("ControlMode");
                         if(mode.compare("") == 0) return 0;
                         if(mode.compare("Position") == 0) return 1;
                         if(mode.compare("Velocity") == 0) return 2;
                         if(mode.compare("Torque") == 0) return 3;
                         return 0;
                       });

  mc_rtc::log::success("MinimumJerkController init done ");
}

bool MinimumJerkController::run()
{
  auto ctrl_mode = datastore().get<std::string>("ControlMode");

  if(ctrl_mode.compare("Position") == 0)
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop);
  }
  else
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }

  return false;
}

void MinimumJerkController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

void MinimumJerkController::getPostureTarget(void)
{
  posture_target_log = rbd::dofToVector(robot().mb(), compPostureTask->posture());
}
