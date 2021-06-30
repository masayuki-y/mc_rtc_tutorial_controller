#include "MyFirstController.h"
#include <mc_rbdyn/RobotLoader.h>
#include <mc_tasks/EndEffectorTask.h>


MyFirstController::MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController({rm,
  mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("ground"))}, dt)
  {
    config_.load(config);
    solver().addConstraintSet(contactConstraint);
    solver().addConstraintSet(dynamicsConstraint);
    solver().addTask(postureTask);
    solver().setContacts({
      {robots(), 0, 1, "LeftFoot", "AllGround"},
      {robots(), 0, 1, "RightFoot", "AllGround"}
    });


    //set EndEfector's task
    efTask1 = std::make_shared<mc_tasks::EndEffectorTask>("r_wrist", robots(), 0, 5.0, 500.0);
    efTask2 = std::make_shared<mc_tasks::EndEffectorTask>("l_wrist", robots(), 0, 5.0, 500.0);
    solver().addTask(efTask1);
    solver().addTask(efTask2);

    //set CoM's task
    comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 10.0, 1000.0);
    solver().addTask(comTask);
    // Reduce the posture task stiffness
    postureTask->stiffness(1);
    // In the reset function, reset the task to the current CoM
    comTask->reset();

    mc_rtc::log::success("MyFirstController init done");
  }

  bool MyFirstController::run()
  {
    switch_phase();
    return mc_control::MCController::run();
  }

  void MyFirstController::reset(const mc_control::ControllerResetData & reset_data)
  {
    //In reset func, move EE to box position
    efTask1->reset();
    efTask2->reset();
    mc_control::MCController::reset(reset_data);
    //obtain CoM value when initial
    comTask->reset();
    comZero = comTask->com();

    //move CoM comDown
    comTask->com(comZero - Eigen::Vector3d{0, 0, 0.2});

    //this demo doesn't appear in visualliser
    //robots().robot(1).posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.65, -0.1, 1.2)));

    //reset->run->switch_phase(GRIP -> )
  }

  void MyFirstController::switch_phase()
  {
    if(phase == SIT)
    {
      //move hands to box's back
      auto pt1 = efTask1->get_ef_pose();
      efTask1->set_ef_pose(sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.6, -0.25, 0.95}});
      auto pt2 = efTask2->get_ef_pose();
      efTask2->set_ef_pose(sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.6, 0.25, 0.95}});

      phase =   APPROACH;
    }
    //else if(phase == APPROACH)
    //{
      //change hand shape "open" -> "close"


      //}
      else if(phase =   APPROACH) //phase == GRIP )
      {
        //pull box to torso
        efTask1->reset();
        efTask2->reset();
        auto pt1 = efTask1->get_ef_pose();
        efTask1->set_ef_pose(sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.6, -0.25, 0.95}});
        auto pt2 = efTask2->get_ef_pose();
        efTask2->set_ef_pose(sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.6, 0.25, 0.95}});

        phase = PULL;
      }
    }

    CONTROLLER_CONSTRUCTOR("MyFirstController", MyFirstController)
