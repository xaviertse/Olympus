#include <robot_motion/robot_motion.h>
namespace robotMotion
{
  void robotMotionApplication::loadParameters()
  {
    //Create private node handle
    ros::NodeHandle ph("~");

    //Create public node handle
    ros::NodeHandle nh;

    //Get the parameter data from the config files
    //Robot parameters
    if(ph.getParam("group_name",config_.group_name) &&
       ph.getParam("tip_link",config_.tip_link) &&
       ph.getParam("base_link",config_.base_link) &&
       ph.getParam("world_frame",config_.world_frame) &&
       ph.getParam("robot_setup_config_name",config_.robot_setup_config_name) && //josh added for init_descartes to right robot_model

    //Time delay and home point
       ph.getParam("trajectory/time_delay",config_.time_delay) &&
       ph.getParam("trajectory/seed_pose",config_.seed_pose) &&

    //Minimum distance between points
       ph.getParam("visualization/min_point_distance",config_.min_point_distance) &&
       
       nh.getParam("controller_joint_names",config_.joint_names) /*&&

       nh.getParam("masking_tool_velocity",config_.masking_tool_velocity)*/)
    {
      ROS_INFO_STREAM("Loaded application parameters");
    }

    else
    {
      ROS_ERROR_STREAM("Failed to load application parameters");
      exit(-1);
    }

    ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");
  }
}
