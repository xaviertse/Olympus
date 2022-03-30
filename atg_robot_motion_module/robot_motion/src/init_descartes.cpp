#include <robot_motion/robot_motion.h>
//#include <descartes_moveit/ikfast_moveit_state_adapter.h>
//#include <descartes_moveit/moveit_state_adapter.h>

#include <atg_ur5e_descartes/ur5e_robot_model.h>
#include <atg_ur10_descartes/ur10_robot_model.h>
#include <atg_ur10e_descartes/ur10e_robot_model.h>
#include <abb_irb2400_descartes/abb_irb2400_robot_model.h>//worked with new addition of abb_irb2400_descartes lib
#include <abb_irb2600_12_165_descartes/abb_irb2600_12_165_robot_model.h>
#include <abb_irb1200_7_70_descartes/abb_irb1200_7_70_robot_model.h>

namespace robotMotion
{
  void robotMotionApplication::initDescartes()
  {
    //Initialize robot model
    if(config_.robot_setup_config_name == "abb_irb2400") robot_model_ptr_.reset(new abb_irb2400_descartes::AbbIrb2400RobotModel());
    else if(config_.robot_setup_config_name == "abb_irb2600_12_165") robot_model_ptr_.reset(new abb_irb2600_12_165_descartes::AbbIrb2600RobotModel());
    else if(config_.robot_setup_config_name == "abb_irb1200_7_70") robot_model_ptr_.reset(new abb_irb1200_7_70_descartes::AbbIrb1200RobotModel());
    else if(config_.robot_setup_config_name == "ur5e") robot_model_ptr_.reset(new atg_ur5e_descartes::UR5ERobotModel()); //for ur5e
    else if(config_.robot_setup_config_name == "ur10e") robot_model_ptr_.reset(new atg_ur10e_descartes::UR10ERobotModel()); //for ur10e
    else robot_model_ptr_.reset(new atg_ur10_descartes::UR10RobotModel()); //for ur10 and ur10e


    //robot_model_ptr_.reset(new descartes_moveit::IkFastMoveitStateAdapter());
    robot_model_ptr_->setCheckCollisions(true); // Joe added
    if(robot_model_ptr_->initialize(ROBOT_DESCRIPTION_PARAM, config_.group_name, config_.world_frame, config_.tip_link))
    {
      
      ROS_INFO_STREAM("Descartes Robot Model initialized");
    }

    else
    {
      ROS_ERROR_STREAM("Failed to initialize Robot Model");
      exit(-1);
    }

    bool succeeded = planner_.initialize(robot_model_ptr_);
    if(succeeded)
    {
      ROS_INFO_STREAM("Descartes Dense Planner initialized");
    }

    else
    {
      ROS_ERROR_STREAM("Failed to initialize Dense Planner");
      exit(-1);
    }

    ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");
  }
}
