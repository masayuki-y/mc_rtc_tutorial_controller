#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/SurfaceTransformTask.h>


#include "api.h"

enum DoorPhase
{
      SIT = 0,
      APPROACH,
      GRIP,
      PULL
};

struct MyFirstController_DLLAPI MyFirstController : public mc_control::MCController
{
    MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

    void switch_phase();
private:
    mc_rtc::Configuration config_;

    //control EE
    std::shared_ptr<mc_tasks::EndEffectorTask> efTask1;
    std::shared_ptr<mc_tasks::EndEffectorTask> efTask2;

    //control CoM
    std::shared_ptr<mc_tasks::CoMTask> comTask;
    Eigen::Vector3d comZero;


    DoorPhase phase = APPROACH;
};
