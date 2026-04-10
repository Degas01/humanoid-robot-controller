#include "MyController.h"

MyController::MyController(mc_rbdyn::RobotModulePtr rm, double dt,
                           const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);

  solver().addTask(postureTask);

  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 10.0, 1000.0);
  solver().addTask(comTask);

  rhTask = std::make_shared<mc_tasks::EndEffectorTask>(
      "R_WRIST_Y_S", robots(), 0, 5.0, 1000.0);
  solver().addTask(rhTask);

  solver().setContacts({});
  mc_rtc::log::success("MyController init done");
}

bool MyController::run()
{
  t_ += timeStep;
  auto pose = rhTask->get_ef_pose();
  pose.translation().z() = 0.85 + 0.10 * std::sin(t_);
  rhTask->set_ef_pose(pose);

  return mc_control::MCController::run();
}

void MyController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("MyController", MyController)
