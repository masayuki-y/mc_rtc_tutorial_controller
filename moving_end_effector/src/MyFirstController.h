#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/EndEffectorTask.h>


#include "api.h"

struct MyFirstController_DLLAPI MyFirstController : public mc_control::MCController
{
    MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

    void switch_target();

    void switch_com_target();
private:
    mc_rtc::Configuration config_;
    //std::string jointName = "NECK_Y";
    //std::string jointName = "L_SHOULDER_P";
    int jointIndex = 0;
    bool goingLeft = true;
    std::shared_ptr<mc_tasks::CoMTask> comTask;
    std::shared_ptr<mc_tasks::EndEffectorTask> efTask;
    Eigen::Vector3d comZero;
    bool comDown = true;
};
