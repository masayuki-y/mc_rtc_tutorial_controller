#include "MyFirstController.h"
#include <iostream>
#include <fstream>

using std::endl;
using std::ofstream;

std::ofstream ofs_torque("/root/my_first_controller/src/jointTorque.csv");
std::ofstream ofs_torque_three("/root/Record/joint/jointTorque_three.csv");
std::ofstream ofs_torque_right_wrist("/root/Record/joint/");
std::ofstream ofs_torque_right_elbow("/root/Record/joint/jointTorque__right_elbow.csv");
std::ofstream ofs_torque_right_sholder("/root/Record/joint/jointTorque__right_sholder.csv");
std::ofstream ofs_jointNumber("/root/my_first_controller/src/jointNumber.csv");
std::ofstream ofs_torque_part("/root/Record/Alljoints/jointTorque_All.csv");
std::ofstream ofs_torque_botharm("/root/Record/Alljoints/jointTorque_botharm.csv");

int t;


MyFirstController::MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);
  solver().setContacts({{}});

  mc_rtc::log::success("MyFirstController init done ");
}

bool MyFirstController::run()
{
  
  for(unsigned int i; i<robots().robot(0).jointTorques().size(); ++i)
    ofs_torque<< robots().robot(0).jointTorques()[i]<< " ";
  ofs_torque<< std::endl;
  ofs_jointNumber<< robots().robot(0).jointTorques().size() << std::endl;
  ofs_torque_three<< robots().robot(0).jointTorques()[1]<< " "<< robots().robot(0).jointTorques()[2] << " "<< robots().robot(0).jointTorques()[3]<< std::endl;
  //id18=R_SHOULDER_P -> id24=R_WRIST_Y
  for(unsigned int i; i<43; ++i) 
      //ofs_torque_part<< t <<", ";
      ofs_torque_part<<robots().robot(0).jointTorques()[0+i]<<", ";
      ofs_torque_part<< std::endl;

for(unsigned int i; i<26; ++i) 
      //ofs_torque_part<< t <<", ";
      //id18=right sholder id43=left lefthand 
      ofs_torque_botharm<<robots().robot(0).jointTorques()[18+i]<< ", ";
      ofs_torque_botharm<< std::endl;

  ofs_torque_right_sholder<< t <<", "<< robots().robot(0).jointTorques()[21]<< ", "<< robots().robot(0).jointTorques()[20] << ", "<< robots().robot(0).jointTorques()[22]<< std::endl;
  ofs_torque_right_elbow<< t <<", "<< robots().robot(0).jointTorques()[23]<< ", "<< robots().robot(0).jointTorques()[24]<< std::endl;
  ofs_torque_right_wrist<< t <<", "<< robots().robot(0).jointTorques()[25]<< ", "<< robots().robot(0).jointTorques()[26] << std::endl;
  t=t+1;

  return mc_control::MCController::run();
}

void MyFirstController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("MyFirstController", MyFirstController)
