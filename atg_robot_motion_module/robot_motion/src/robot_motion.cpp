#include <robot_motion/robot_motion.h>
#include <descartes_utilities/ros_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <math.h>
#include <stdio.h>

namespace robotMotion
{
  robotMotionApplication::robotMotionApplication() {}

  robotMotionApplication::~robotMotionApplication() {}

  void robotMotionApplication::publishPosesMarkers(const EigenSTL::vector_Affine3d& poses)
  {
    ROS_INFO_STREAM("publishPosesMarkers "); // joe added
    //Creating Rviz markers
    visualization_msgs::Marker z_axes, y_axes, x_axes, line;
    visualization_msgs::MarkerArray markers_msg;

    z_axes.type = y_axes.type = x_axes.type = visualization_msgs::Marker::LINE_LIST;
    z_axes.ns = y_axes.ns = x_axes.ns = "axes";
    z_axes.action = y_axes.action = x_axes.action = visualization_msgs::Marker::ADD;
    z_axes.lifetime = y_axes.lifetime = x_axes.lifetime = ros::Duration(0);
    z_axes.header.frame_id = y_axes.header.frame_id = x_axes.header.frame_id = config_.world_frame;
    z_axes.scale.x = y_axes.scale.x = x_axes.scale.x = AXIS_LINE_WIDTH;
    //marker.pose.orientation.x = 0.0;
      //   marker.pose.orientation.y = 0.0;
        // marker.pose.orientation.z = 0.0;
         //marker.pose.orientation.w = 1.0;
    //Z properties
    z_axes.id = 0;
    z_axes.color.r = 0;
    z_axes.color.g = 0;
    z_axes.color.b = 1;
    z_axes.color.a = 1;

    //Y properties
    y_axes.id = 1;
    y_axes.color.r = 0;
    y_axes.color.g = 1;
    y_axes.color.b = 0;
    y_axes.color.a = 0.1;

    //X properties
    x_axes.id = 2;
    x_axes.color.r = 1;
    x_axes.color.g = 0;
    x_axes.color.b = 0;
    x_axes.color.a = 0.1;

    //Line properties
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.ns = "line";
    line.action = visualization_msgs::Marker::ADD;
    line.lifetime = ros::Duration(0);
    line.header.frame_id = config_.world_frame;
    line.scale.x = AXIS_LINE_WIDTH;
    line.id = 0;
    line.color.r = 1;
    line.color.g = 1;
    line.color.b = 0;
    line.color.a = 1;

    //Create axes markers
    z_axes.points.reserve(2*poses.size());
    y_axes.points.reserve(2*poses.size());
    x_axes.points.reserve(2*poses.size());
    line.points.reserve(poses.size());

    geometry_msgs::Point p_start,p_end;
    double distance = 0;
    Eigen::Affine3d prev = poses[0];
    
    for(unsigned int i = 0; i < poses.size(); i++)
    {
      const Eigen::Affine3d& pose = poses[i];
      distance = (pose.translation() - prev.translation()).norm();

      tf::pointEigenToMsg(pose.translation(),p_start);
      // std::cout << "config_.min_point_distance: " << config_.min_point_distance << std::endl; // joe added
      // std::cout << "distance: " << distance << std::endl;  // joe added
      if(distance > config_.min_point_distance)
      {  
        Eigen::Affine3d moved_along_x = pose * Eigen::Translation3d(AXIS_LINE_LENGHT,0,0);
        tf::pointEigenToMsg(moved_along_x.translation(),p_end);
        x_axes.points.push_back(p_start);
        x_axes.points.push_back(p_end);

        Eigen::Affine3d moved_along_y = pose * Eigen::Translation3d(0,AXIS_LINE_LENGHT,0);
        tf::pointEigenToMsg(moved_along_y.translation(),p_end);
        y_axes.points.push_back(p_start);
        y_axes.points.push_back(p_end);

        Eigen::Affine3d moved_along_z = pose * Eigen::Translation3d(0,0,AXIS_LINE_LENGHT);
        tf::pointEigenToMsg(moved_along_z.translation(),p_end);
        z_axes.points.push_back(p_start);
        z_axes.points.push_back(p_end);

        //Save previous
        prev = pose;
      }

      line.points.push_back(p_start);
    }

    markers_msg.markers.push_back(x_axes);
    markers_msg.markers.push_back(y_axes);
    markers_msg.markers.push_back(z_axes);
    markers_msg.markers.push_back(line);

    marker_publisher_.publish(markers_msg);  
  }

  void addVel(trajectory_msgs::JointTrajectory& traj)
  {
    if (traj.points.size() < 3) return;

    auto n_joints = traj.points.front().positions.size();

    for (auto i = 0; i < n_joints; ++i)
    {
      //For each point in a given joint
      for (auto j = 1; j < traj.points.size() - 1; j++)
      {
        double delta_theta = -traj.points[j - 1].positions[i] + traj.points[j + 1].positions[i];
        double delta_time = -traj.points[j - 1].time_from_start.toSec() + traj.points[j + 1].time_from_start.toSec();
        double v = delta_theta / delta_time;
        traj.points[j].velocities[i] = v;
      } 
    }
  }

  void robotMotionApplication::fromDescartesToMoveitTrajectory(const DescartesTrajectory& in_traj, trajectory_msgs::JointTrajectory& out_traj)
  {
    //Fill out info about the trajectory
    out_traj.header.stamp = ros::Time::now();
    out_traj.header.frame_id = config_.world_frame;
    out_traj.joint_names = config_.joint_names;

    descartes_utilities::toRosJointPoints(*robot_model_ptr_, in_traj, 0.4, out_traj.points);
    addVel(out_traj);
  }

  void robotMotionApplication::fromJointToCart(const DescartesTrajectory& path, int index, double *pose_rotvec_quat, Eigen::Matrix3d &rot_mat)
  {
    // josh added for cartesian pose START------------
    bool debug = 0;
    //josh to get rotation and translation matrix of cartesian pose from joint pose = path[i]
    Eigen::Affine3d cart_pose;
    std::vector<double> dummy;                                                  // R R R T
    descartes_core::TrajectoryPtPtr cur_point_ptr = path[index];                // R R R T
    cur_point_ptr->getNominalCartPose(dummy,*robot_model_ptr_,cart_pose);       // R R R T
    if(debug)std::cout <<"cart_pose["<<index<<"]\n" << cart_pose.matrix() << std::endl;  // 0 0 0 1

    //josh to get RPY (roll, pitch, yaw) from rotation matrix
    Eigen::MatrixXd rot = cart_pose.rotation().matrix();
    //Eigen::Matrix3d mat = rot;
    rot_mat = rot;
/*    //manual calculations, still valid
//    double rpy_roll  = atan2( mat(2,1),mat(2,2) );
//    double rpy_pitch = atan2( -mat(2,0), std::pow( mat(2,1)*mat(2,1) +mat(2,2)*mat(2,2) ,0.5  )  );
//    double rpy_yaw   = atan2( mat(1,0),mat(0,0) );
//    std::cout<<"manual cal -> roll is  " << rpy_roll <<std::endl;
//    std::cout<<"manual cal -> pitch is " << rpy_pitch <<std::endl;
//    std::cout<<"manual cal -> yaw is   " << rpy_yaw <<std::endl;

    //josh to get EulerAngles from rotation matrix //does not seem correct
    //Vector3d ea =  mat.eulerAngles(2,1,0); //UR uses Z-Y'-X" axes for Rrpy=Rz(yaw)Ry(pitch)Rx(roll)
    //double rpy_yaw  = ea[0];
    //double rpy_pitch = ea[1];
    //double rpy_roll   = ea[2];
    //std::cout<<"eigen -> roll is  " << rpy_roll <<std::endl; //does not seem correct
    //std::cout<<"eigen -> pitch is " << rpy_pitch <<std::endl;//does not seem correct
    //std::cout<<"eigen -> yaw is   " << rpy_yaw <<std::endl;  //does not seem correct

//    //josh to get Quaternion values from rotation matrix
//    Eigen::Quaterniond q_mat(mat);
//    std::cout <<"q_mat x,y,z,w: " << q_mat.x() << ", " << q_mat.y() << ", " << q_mat.z() << ", " << q_mat.w() << std::endl<<  std::endl;

//    //josh to get Rotational Vector from RPY from rotation matrix
//    Eigen::Matrix3d RollM;
//    RollM(0,0) = 1; RollM(0,1) = 0;             RollM(0,2) = 0;
//    RollM(1,0) = 0; RollM(1,1) = cos(rpy_roll); RollM(1,2) = -sin(rpy_roll);
//    RollM(2,0) = 0; RollM(2,1) = sin(rpy_roll); RollM(2,2) =  cos(rpy_roll);

//    Eigen::Matrix3d PitchM;
//    PitchM(0,0) = cos(rpy_pitch); PitchM(0,1) = 0; PitchM(0,2) = sin(rpy_pitch);
//    PitchM(1,0) = 0;              PitchM(1,1) = 1; PitchM(1,2) = 0;
//    PitchM(2,0) = -sin(rpy_pitch);PitchM(2,1) = 0; PitchM(2,2) = cos(rpy_pitch);

//    Eigen::Matrix3d YawM;
//    YawM(0,0) = cos(rpy_yaw); YawM(0,1) = -sin(rpy_yaw);YawM(0,2) = 0;
//    YawM(1,0) = sin(rpy_yaw); YawM(1,1) = cos(rpy_yaw); YawM(1,2) = 0;
//    YawM(2,0) = 0;            YawM(2,1) = 0;            YawM(2,2) = 1;

//    Eigen::Matrix3d rotM;
//    rotM = YawM * PitchM * RollM;
//    double rotSum = rotM(0,0) + rotM(1,1) + rotM(2,2) - 1;
//    double alpha = acos(rotSum/2);
//    double theta,my,rx,ry,rz;// = 2 * M_PI - alpha; //double theta = 0;
//    //display both calculation on alpha and 2pi-alpha
//    theta = alpha;
//    my = 1.0 / (2*sin(theta));
//    rx = my * (rotM(2,1) - rotM(1,2)) * theta;
//    ry = my * (rotM(0,2) - rotM(2,0)) * theta;
//    rz = my * (rotM(1,0) - rotM(0,1)) * theta;
//    std::cout <<"t = a       -> rx,ry,rz: " << rx << ", " << ry << ", " << rz <<  std::endl;

//    theta = 2 * M_PI - alpha;
//    my = 1.0 / (2*sin(theta));
//    rx = my * (rotM(2,1) - rotM(1,2)) * theta;
//    ry = my * (rotM(0,2) - rotM(2,0)) * theta;
//    rz = my * (rotM(1,0) - rotM(0,1)) * theta;
//    std::cout <<"t = 2PI - a -> rx,ry,rz: " << rx << ", " << ry << ", " << rz << std::endl<<  std::endl;

//    //display calculation on alpha and 2pi-alpha based on condition
//    if (q_mat.w()>0)
//    {
//      theta = alpha;
//      std::cout <<"t = a"<<  std::endl;
//    }
//    else
//    {
//      theta = 2 * M_PI - alpha;
//      std::cout <<"t = 2p-a"<<  std::endl;
//    }
//    my = 1.0 / (2*sin(theta));
//    rx = my * (rotM(2,1) - rotM(1,2)) * theta;
//    ry = my * (rotM(0,2) - rotM(2,0)) * theta;
//    rz = my * (rotM(1,0) - rotM(0,1)) * theta;
//    std::cout <<"manual cal -> rx,ry,rz: " << rx << ", " << ry << ", " << rz << std::endl<<  std::endl;*/

    //Eigen::AngleAxisd newAngleAxis(q_mat); //from Quaternion
    Eigen::AngleAxisd newAngleAxis = newAngleAxis.fromRotationMatrix(rot_mat);
    std::vector<double> rot_vec;
    rot_vec.push_back(newAngleAxis.axis()[0,0]*newAngleAxis.angle());
    rot_vec.push_back(newAngleAxis.axis()[0,1]*newAngleAxis.angle());
    rot_vec.push_back(newAngleAxis.axis()[0,2]*newAngleAxis.angle());

    if(debug)std::cout <<"AngleAxisd Axis Angle: " << newAngleAxis.angle()<<  std::endl;
    ////std::cout <<"AngleAxisd rx,ry,rz: " << rot_vec[0]*M_PI << ", " << rot_vec[1]*M_PI << ", " << rot_vec[2]*M_PI << std::endl<<  std::endl;
    if(debug)std::cout <<"AngleAxisd rx,ry,rz: " << rot_vec[0] << ", " << rot_vec[1] << ", " << rot_vec[2] <<  std::endl;
    Eigen::Quaterniond quat(rot_mat);
    if(debug)std::cout <<"Quaterniond x,y,z,w: " << quat.x() << ", " << quat.y() << ", " << quat.z() <<", "<< quat.w() << std::endl<<  std::endl;


        //return x,y,z,Rx,Ry,Rz
    pose_rotvec_quat[0] = cart_pose.matrix()(0,3);
    pose_rotvec_quat[1] = cart_pose.matrix()(1,3);
    pose_rotvec_quat[2] = cart_pose.matrix()(2,3);
    pose_rotvec_quat[3] = rot_vec[0];
    pose_rotvec_quat[4] = rot_vec[1];
    pose_rotvec_quat[5] = rot_vec[2];
    pose_rotvec_quat[6] = quat.x();
    pose_rotvec_quat[7] = quat.y();
    pose_rotvec_quat[8] = quat.z();
    pose_rotvec_quat[9] = quat.w();

    // josh added for cartesian pose END-------------
  }

  std::string robotMotionApplication::robot_config_name()  //to output robot_setup_config_name as robotMotionConfiguration is protected
  {                                                        //to output robot_setup_config_name as robotMotionConfiguration is protected
      return config_.robot_setup_config_name;              //to output robot_setup_config_name as robotMotionConfiguration is protected
  }                                                        //to output robot_setup_config_name as robotMotionConfiguration is protected
}
