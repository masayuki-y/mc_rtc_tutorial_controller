#include "MyFirstController.h"
#include <mc_tasks/EndEffectorTask.h>
#include <mc_rbdyn/RobotLoader.h>

MyFirstController::MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController({rm,
                           mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH) + "/../mc_int_obj_description", std::string("box")),
                           mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("ground"))}, dt)
{ //the constructor
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  // solver().addConstraintSet(kinematicsConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addTask(postureTask);
  // solver().setContacts({{}});
  solver().setContacts({
    {robots(), 0, 2, "LeftFoot", "AllGround"},
    {robots(), 0, 2, "RightFoot", "AllGround"}
  });

  //  create the task and add it to the problem
  efTask1 = std::make_shared<mc_tasks::EndEffectorTask>("r_wrist", robots(), 0, 5.0, 500.0);
  efTask2 = std::make_shared<mc_tasks::EndEffectorTask>("l_wrist", robots(), 0, 5.0, 500.0);
  solver().addTask(efTask1);
  solver().addTask(efTask2);

  mc_rtc::log::success("MyFirstController init done");
}

bool MyFirstController::run()
{
  auto pt1 = efTask1->get_ef_pose();
  efTask1->set_ef_pose(sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.5, -0.25, 1.2}});
  auto pt2 = efTask2->get_ef_pose();
  efTask2->set_ef_pose(sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.5, 0.05, 1.2}});
  return mc_control::MCController::run();
}

void MyFirstController::reset(const mc_control::ControllerResetData & reset_data)
{
  efTask1->reset();
  efTask2->reset();
  mc_control::MCController::reset(reset_data);
  robots().robot(1).posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.65, -0.1, 1.2)));
  // efTask->reset();
}

CONTROLLER_CONSTRUCTOR("MyFirstController", MyFirstController)
