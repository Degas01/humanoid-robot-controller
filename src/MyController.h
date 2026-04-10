#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/EndEffectorTask.h>
#include <cmath>

struct MyController : public mc_control::MCController
{
  MyController(mc_rbdyn::RobotModulePtr rm, double dt,
               const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

private:
  std::shared_ptr<mc_tasks::CoMTask> comTask;
  std::shared_ptr<mc_tasks::EndEffectorTask> rhTask;
  double t_ = 0.0;
};
