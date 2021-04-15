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

  //  create the task and add it to the problem
  //priority of CoMTask is higer,default is 5 but this case is 10
  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 10.0, 1000.0);
  solver().addTask(comTask);
  //The posture task stiffness is decreased to make sure it doens't interfere with the CoM task
  postureTask->stiffness(1);

  mc_rtc::log::success("MyFirstController init done");
}

bool MyFirstController::run()
{
  efTask1->set_ef_pose(sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.5, -0.25, 1.2}});
  auto pt2 = efTask2->get_ef_pose();
  efTask2->set_ef_pose(sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.5, 0.05, 1.2}});

  //control CoM with switch_com_target()

    if(comTask->eval().norm() < 0.01)
    {
      switch_com_target();
    }


  return mc_control::MCController::run();
}

void MyFirstController::reset(const mc_control::ControllerResetData & reset_data)
{
  efTask1->reset();
  efTask2->reset();
  comTask->reset();
  mc_control::MCController::reset(reset_data);

  //comZero is obtained by doing as forrow
   comZero = comTask->com();
  robots().robot(1).posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.6, -0.1, 1.2)));
}

void MyFirstController::switch_com_target()
{
  // comZero is obtained by doing:
  // comZero = comTask->com();
  // in the reset function
  if(comDown)
  {
    comTask->com(comZero - Eigen::Vector3d{0, 0, 0.2});
  }
  else
  {
    comTask->com(comZero);
  }
  comDown = !comDown;
}


CONTROLLER_CONSTRUCTOR("MyFirstController", MyFirstController)
