#include <robot_motion/robot_motion.h>

namespace robotMotion
{
  void robotMotionApplication::runPath(const DescartesTrajectory& path)
  {
    //Create move group to move the arm in free space
    moveit::planning_interface::MoveGroup move_group(config_.group_name);
    move_group.setPlannerId(PLANNER_ID);

    //Create goal joint pose to start of the path
    std::vector<double> seed_pose(robot_model_ptr_->getDOF());
    std::vector<double> start_pose;

    descartes_core::TrajectoryPtPtr first_point_ptr = path[0];
    first_point_ptr->getNominalJointPose(seed_pose,*robot_model_ptr_,start_pose);

    //Move arm to joint goal
    move_group.setJointValueTarget(start_pose);
    move_group.setPlanningTime(60.0f);
    moveit_msgs::MoveItErrorCodes result = move_group.move();

    if(result.val != result.SUCCESS)
    {
      ROS_ERROR_STREAM("Move to start joint pose failed");
      exit(-1);
    }

    //Create MoveIT! trajectory from Descartes Trajectory
    moveit_msgs::RobotTrajectory moveit_traj;
    fromDescartesToMoveitTrajectory(path,moveit_traj.joint_trajectory);

    //Send robot path to server for execution
    moveit_msgs::ExecuteKnownTrajectory srv;
    srv.request.trajectory = moveit_traj;
    srv.request.wait_for_execution = true;

    ROS_INFO_STREAM("Robot path sent for execution");

    if(moveit_run_path_client_.call(srv))
    {
      ROS_INFO_STREAM("Robot path execution completed");
    }
    
    else
    {
      ROS_ERROR_STREAM("Failed to run robot path with error "<<srv.response.error_code.val);
      exit(-1);
    }

    ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");
  }
}
