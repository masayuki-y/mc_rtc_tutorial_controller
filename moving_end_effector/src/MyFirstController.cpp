#include "MyFirstController.h"
#include <mc_tasks/EndEffectorTask.h>

MyFirstController::MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{ //the constructor
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  // solver().addConstraintSet(kinematicsConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addTask(postureTask);
  // solver().setContacts({{}});
  solver().setContacts({
    {robots(), 0, 1, "LeftFoot", "AllGround"},
    {robots(), 0, 1, "RightFoot", "AllGround"}
  });

  //  create the task and add it to the problem
  efTask = std::make_shared<mc_tasks::EndEffectorTask>("r_wrist", robots(), 0, 5.0, 500.0);
  solver().addTask(efTask);

  mc_rtc::log::success("MyFirstController init done");
}

bool MyFirstController::run()
{
  auto pt = efTask->get_ef_pose();
  efTask->set_ef_pose(sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.5, -0.5, 1.2}});
  return mc_control::MCController::run();
}

void MyFirstController::reset(const mc_control::ControllerResetData & reset_data)
{
  efTask->reset();
  mc_control::MCController::reset(reset_data);
  // efTask->reset();
}

CONTROLLER_CONSTRUCTOR("MyFirstController", MyFirstController)
