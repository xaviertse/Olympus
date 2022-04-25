#include <robot_motion/robot_motion.h>

namespace robotMotion
{
  void robotMotionApplication::initRos()
  {
    //Create publisher for trajectory visualization
    marker_publisher_  = nh_.advertise<visualization_msgs::MarkerArray>(VISUALIZE_TRAJECTORY_TOPIC, 1, true);

    moveit_run_path_client_ = nh_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(EXECUTE_TRAJECTORY_SERVICE,true);

    //Establish connection to service server
    if(moveit_run_path_client_.waitForExistence(ros::Duration(SERVICE_TIMEOUT)))
    {
      ROS_INFO_STREAM("Connected to '"<<moveit_run_path_client_.getService()<<"' service");
    }

    else
    {
      ROS_ERROR_STREAM("Failed to connect to '"<< moveit_run_path_client_.getService()<<"' service");
      exit(-1);
    }

    ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");
  }
}

