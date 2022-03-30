#ifndef ROBOT_MOTION_H_
#define ROBOT_MOTION_H_

#include <ros/ros.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/move_group_interface/move_group.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>

namespace robotMotion
{
  const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
  const std::string EXECUTE_TRAJECTORY_SERVICE = "execute_kinematic_path";
  const std::string VISUALIZE_TRAJECTORY_TOPIC = "visualize_trajectory_curve";
  const double SERVICE_TIMEOUT = 90.0f;//5.0f; //90s enables rviz initialization to complete
  const double ORIENTATION_INCREMENT = 0.1f;// 0.02f; //0.5f;
  const double EPSILON = 0.0001f;
  const double AXIS_LINE_LENGHT = 0.01;
  const double AXIS_LINE_WIDTH = 0.001/2;
  //const std::string PLANNER_ID = "RRTConnectkConfigDefault";
  const std::string PLANNER_ID = "RRTConnect";
  const std::string HOME_POSITION_NAME = "home";

  typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTrajectory;

  /*  =============================== Application Data Structure ===============================
  *
  * Holds the data used at various points in the application.  This structure is populated
  * from data found in the ros parameter server at runtime.
  *
  */
  struct robotMotionConfiguration
  {
    std::string group_name;                 //Name of the manipulation group containing the relevant links in the robot
    std::string tip_link;                   //Usually the last link in the kinematic chain of the robot
    std::string base_link;                  //The name of the base link of the robot
    std::string world_frame;                //The name of the world link in the URDF file
    std::vector<std::string> joint_names;   //A list with the names of the mobile joints in the robot
    std::string robot_setup_config_name;    //josh added for init_descartes to right robot_model
    std::string filename_short;             //josh added for generating scripts files to queue

    //Trajectory Generation Members
    double time_delay;              //Time step between consecutive points in the robot path
    std::vector<double> seed_pose;  //Joint values close to the desired start of the robot path

    //Visualization Members
    double min_point_distance;      //Minimum distance between consecutive trajectory points

//    float masking_tool_velocity; // the linear speed the tool is constrained by (mm/s)
    double FEEDRATE = 1.0; //mm/s           ,generic
    double BLEND_ZONE = 0; //mm
    double FLOWRATE = 0.1/(0.6*1.0); //ml/s ,MASKING
    double JUMPSPEED = 3.0; //mm/s          ,generic
    double fixed_z = 0; //mm                ,generic
    double fixed_z_lift = 0; //mm           ,generic
    double SpindleSpeed = 0; //rpm,         ,POLISHING
    double Force        = 0; //N            ,POLISHING
    double OPT1 =0;//masking: FLOWRATE; Polishing: Force (N)  ; Impedance: RPM; Sandblasting: blast_dia    ;
    double OPT2 =0;//                 ; Polishing: Spindle RPM;               ; Sandblasting: air_flow_rate;
    double OPT3 =0;//not used
    double OPT4 =0;//not used
    std::string TCP_info = "";//TCP string   ,POLISHING
    std::string PROCESS = "";
  };


  /*  =============================== Application Class ===============================
  *
  * Provides a group of functions for planning and executing a robot path using Moveit and
  * the Descartes Planning Library
  *
  */
  class robotMotionApplication
  {
  public:
    robotMotionApplication();
    virtual ~robotMotionApplication();

    //Main Application Functions
    void loadParameters();
    void initRos();
    void initDescartes();
    void moveHome();
    void generateTrajectory(DescartesTrajectory& traj);
    void planPath(DescartesTrajectory& input_traj,DescartesTrajectory& output_path);
    void runPath(const DescartesTrajectory& path);
    void generateURScript(const DescartesTrajectory& path, bool bSaveToFile);
    void generateRapidScript(const DescartesTrajectory& path, bool bSaveToFile);
    std::string robot_config_name();

  protected:
    void fromDescartesToMoveitTrajectory(const DescartesTrajectory& in_traj, trajectory_msgs::JointTrajectory& out_traj);
    void publishPosesMarkers(const EigenSTL::vector_Affine3d& poses);

    void fromJointToCart(const DescartesTrajectory &path, int index, double *pose_rotvec_quat, Eigen::Matrix3d& rot_mat);

  protected:

    //Application Data
    robotMotionConfiguration config_;

    // Application ROS Constructs
    ros::NodeHandle nh_;                        //Object used for creating and managing ros application resources
    ros::Publisher marker_publisher_;           //Publishes visualization message to Rviz 
    ros::ServiceClient moveit_run_path_client_; //Sends a robot trajectory to moveit for execution

    // Application Descartes Constructs
    descartes_core::RobotModelPtr robot_model_ptr_; //Performs tasks specific to the Robot such IK, FK and collision detection
    // descartes_planner::SparsePlanner planner_;      //Plans a smooth robot path given a trajectory of points
    descartes_planner::DensePlanner planner_;      //Plans a smooth robot path given a trajectory of points
  };
}

#endif // ROBOT_MOTION_H_
