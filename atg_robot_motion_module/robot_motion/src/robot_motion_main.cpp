#ifdef __i386__
  #pragma message("i386 Architecture detected, disabling EIGEN VECTORIZATION")
  #define EIGEN_DONT_VECTORIZE
  #define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#else
  #pragma message("64bit Architecture detected, enabling EIGEN VECTORIZATION")
#endif

#include <robot_motion/robot_motion.h>

int main(int argc,char** argv)
{
  ros::init(argc,argv,"robot_motion");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  //Create the application
  robotMotion::robotMotionApplication application;
  application.initRos();

  //Initialize the ROS Components
  application.initRos();    //buffer load to wait for initalization of ros EXECUTE_TRAJECTORY_SERVICE
  ros::Duration(5).sleep(); //buffer load to wait for initalization from ros and RViz parameters

  //Load the parameters
  application.loadParameters();

  //Initialize the ROS Components
  //application.initRos();

  //Initialize Descartes
  application.initDescartes();

  // // Move to home position
  // // application.moveHome();

  // //Generate the trajectory
  robotMotion::DescartesTrajectory traj;
  application.generateTrajectory(traj);

  // Plan the robot path
  robotMotion::DescartesTrajectory output_path;
  application.planPath(traj,output_path);

  //Run the robot path
  if(application.robot_config_name() == "abb_irb2400"
  || application.robot_config_name() == "abb_irb2600_12_165"
  || application.robot_config_name() == "abb_irb1200_7_70") application.generateRapidScript(output_path,true);
  else application.generateURScript(output_path,true);
  ros::spinOnce();
  std::cout << std::flush;

  //Run the robot path
  application.runPath(output_path);

  //Exit the ROS node
  spinner.stop();

  return 0;
}


