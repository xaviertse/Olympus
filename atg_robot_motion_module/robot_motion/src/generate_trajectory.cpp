#include <ros/ros.h>
#include <ros/package.h>
#include <robot_motion/robot_motion.h>

#include <fstream>
#include <string>

#define PI M_PI

namespace robotMotion
{

Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * ry * rx;
}

Eigen::Affine3d face_down(Eigen::Affine3d rot_normal_plane)
{// replace this by your actual plane normal:
    Eigen::Vector3d plane_normal;
    plane_normal[0] = 1.0;
    plane_normal[1] = 0.0;
    plane_normal[2] = 0.0;
    plane_normal.normalized();
    Eigen::Quaterniond p;
    p.w() = 0;
    p.vec() = plane_normal;
    Eigen::Matrix3d rot = rot_normal_plane.rotation().matrix();
    Eigen::Quaterniond rot_normal_quat(rot);
    rot_normal_quat.normalized();
    Eigen::Quaterniond rotatedP = rot_normal_quat * p * rot_normal_quat.inverse();
    Eigen::Vector3d plane_normal2 = rotatedP.vec();
//    std::cout << "We can now use it to rotate a vector " << std::endl
//    << plane_normal << " to " << std::endl << plane_normal2 << std::endl;

    Eigen::Vector3d align_to_plane;
    align_to_plane[0] = 0.0;
    align_to_plane[1] = 1.0;
    align_to_plane[2] = 1.0;

    // Quaternion which rotates plane_normal to UnitZ, or the plane to the XY-plane:
    Eigen::Quaterniond rotQ = Eigen::Quaterniond::FromTwoVectors(plane_normal2, align_to_plane);//Eigen::Vector3d::UnitZ());


    std::cout << "Rot_plane_normal: " << plane_normal2.transpose() << '\n';
    std::cout << "Rotated plane_normal: " << (rotQ * plane_normal2).transpose() << '\n';




    return rot_normal_plane;
}

void robotMotionApplication::generateTrajectory(DescartesTrajectory &traj)
{ 
  //Create the vector of poses
  EigenSTL::vector_Affine3d poses;

  //Create an inout file stream
  std::ifstream indata;

  //Read the data from the example_gool_path.csv file inside the config file of the package
  //std::string filename = ros::package::getPath("masking") + "/config/robot_masking_queue.csv"; //  example_tool_path2
  std::string filename = "/home/data/cache/toolpath/robot_toolpath_queued.csv";

  //Open the file
  indata.open(filename);

  std::string line;
  int lineNum = 0;

  //Create the variables for the translation and transformation
  double tx, ty, tz, rx, ry, rz;
  Eigen::Affine3d part_transform; // Translation uses tx, ty, tz
  // Eigen::Affine3d transf; // Translation uses tx, ty, tz
  // Matrix3d m;             // Rotation uses rx, ry, rz

  //Read the file line by line
  while (std::getline(indata, line))
  {
    lineNum++;
    if (lineNum < 2)//read first 2 lines
      continue;

    std::stringstream lineStream(line);
    std::string cell;
    Eigen::VectorXd xyzrpy(6);
    int i = 0;
    if (lineNum == 2)
    {
      while (std::getline(lineStream, cell, ';'))
      {
        if (i==0)
        {
          config_.filename_short = cell;
          std::cout << "Recorded Filename = " << config_.filename_short << std::endl;//josh debug
        }
        if (i==1)
        {
          config_.PROCESS = cell;
          std::cout << "Recorded PROCESS = " << config_.PROCESS << std::endl;//josh debug
        }
        if (i==2)
        {
          config_.TCP_info = cell;
          std::cout << "Recorded TCP_info = " << config_.TCP_info << std::endl;//josh debug
        }
        i++;
      }
      continue;
    }

    if (lineNum == 3)
    {
      while (std::getline(lineStream, cell, ';'))
      {
        //Line4:feedrate;blend/zone radius;lift_spd;opt1;opt2;opt3;opt4;
        if (i==0)
        {
          config_.FEEDRATE = std::stod(cell);
//          config_.masking_tool_velocity = config_.FEEDRATE;//simulation speed
          std::cout << "Recorded Feedrate = " << config_.FEEDRATE << " mm/s" << std::endl;//josh debug
        }
        if (i==1)
        {
          config_.BLEND_ZONE = std::stod(cell);
          std::cout << "Recorded Blend/Zone = " << config_.BLEND_ZONE << " mm" << std::endl;//josh debug
        }
        if (i==2)
        {
          config_.JUMPSPEED = std::stod(cell);//config_.FLOWRATE = std::stod(cell)/6*1.0;
          std::cout << "Recorded cell = " << config_.JUMPSPEED << " mm/s" << std::endl;//josh debug
        }
        if (i==3)
        {
          config_.OPT1 = std::stod(cell);
          std::cout << "Recorded OPT1 = " << config_.OPT1 << " -" << std::endl;//josh debug
        }
        if (i==4)
        {
          config_.OPT2 = std::stod(cell);
          std::cout << "Recorded OPT2 = " << config_.OPT2 << " -" << std::endl;//josh debug
        }
        if (i==5)
        {
          config_.OPT3 = std::stod(cell);
          std::cout << "Recorded OPT3 = " << config_.OPT3 << " -" << std::endl;//josh debug
        }
        if (i==6)
        {
          config_.OPT4 = std::stod(cell);
          std::cout << "Recorded OPT4 = " << config_.OPT4 << " -" << std::endl;//josh debug
          break;
        }
        i++;
      }
    }
    else
    {
      //While there is a line there with ,
      while (std::getline(lineStream, cell, ';'))
      {
          xyzrpy(i) = std::stod(cell);
          i++;
          if (i==6) break;
      }

      Eigen::Affine3d tmp_transl(Eigen::Translation3d(xyzrpy[0]/1000,xyzrpy[1]/1000,xyzrpy[2]/1000));
      Eigen::Affine3d tmp_rot = create_rotation_matrix((xyzrpy[3])*PI/180,(xyzrpy[4])*PI/180,xyzrpy[5]*PI/180);
      //Eigen::Affine3d tmp_rot = create_rotation_matrix(180*PI/180,0*PI/180,0*PI/180); //for demo only

      //If it's the third line, create the world -> part transformation
      if (lineNum == 4)
      { // create the transformation from world to the part
        Eigen::Affine3d tmp_transl(Eigen::Translation3d(xyzrpy[0]/1000,xyzrpy[1]/1000,xyzrpy[2]/1000));
        Eigen::Affine3d tmp_rot = create_rotation_matrix(xyzrpy[3]*PI/180,xyzrpy[4]*PI/180,xyzrpy[5]*PI/180);
        part_transform = tmp_transl * tmp_rot; // std::cout << part_transform.matrix() << "\n" << std::endl;  // joe added
        continue;
      }

      // get the transform from the part to the process point
      Eigen::Affine3d point_transform = tmp_transl * tmp_rot;
      Eigen::Affine3d pose =  part_transform * point_transform;
      face_down(pose);
      //Eigen::Affine3d pose =  point_transform;

      // std::cout << "------------------" << std::endl;  // joe added
      // std::cout << part_transform.matrix() << "\n" << std::endl;  // joe added
      // std::cout << point_transform.matrix() << "\n" << std::endl;  // joe added
      // std::cout << pose.matrix() << "\n" << std::endl;  // joe added

      poses.push_back(pose);
    }
  }

  // Handle the approach and retreat poses.
  //Eigen::Translation3d approach_translation(Eigen::Vector3d(0,0,-0.1));
  //Eigen::Affine3d start_pose = poses[0];
  //start_pose.translate(Eigen::Vector3d(0,0,-0.04));
  //poses.insert(poses.begin(), start_pose);//
  //Eigen::Affine3d end_pose = poses[poses.size()-1];
  //end_pose.translate(Eigen::Vector3d(0,0,-0.06));
  //poses.push_back(end_pose);

  //Close the input data stream
  indata.close();

  // Visualize the poses 
  publishPosesMarkers(poses);
  ros::spinOnce(); 

  //Create the descartes trajectory points
  traj.clear();
  traj.reserve(poses.size());

  Eigen::Affine3d prev_pose;
  for (unsigned int i = 0; i < poses.size(); i++)
  {
    const Eigen::Affine3d &pose = poses[i]; 
    descartes_core::TrajectoryPtPtr pt;
    if (i == 0) {
        pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::AxialSymmetricPt(pose, ORIENTATION_INCREMENT, descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS));
        //pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::CartTrajectoryPt(pose, ORIENTATION_INCREMENT, 0));
    }
    else
    {
        double dis = (pose.matrix().col(3).head<3>() - prev_pose.matrix().col(3).head<3>()).norm() * 1000 ;
        // velocity = dis / time
        // time =  dis / vel = mm / (mm/s) = s 
        double speed_multiplier = 0.5;                                                //josh added speed_multiplier to make simulation X times faster
//        descartes_core::TimingConstraint time(dis/config_.masking_tool_velocity/speed_multiplier); //josh added speed_multiplier to make simulation X times faster
        descartes_core::TimingConstraint time(dis/config_.FEEDRATE/speed_multiplier); //josh added speed_multiplier to make simulation X times faster
        //descartes_core::TimingConstraint time(0);
        // std::cout << "\n" << pose.matrix().col(3).head<3>() << std::endl;
        // std::cout <<  pose.matrix() << std::endl;
        // std::cout << "dis: " << dis << ", vel: " << config_.masking_tool_velocity << ", time: " << dis/config_.masking_tool_velocity  << std::endl;
        // 
        pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::AxialSymmetricPt(pose, ORIENTATION_INCREMENT, descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS,time));
        //pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::CartTrajectoryPt(pose, time));
    }

    prev_pose = pose;
    //Save the points into the trajectory
    traj.push_back(pt);
  }

  ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}
}


// #include <ros/ros.h>
// #include <ros/package.h>
// #include <masking/masking_application.h>

// #include <fstream>
// #include <string>

 //#include <ros/ros.h>
 //#include <ros/package.h>
 //#include <masking/masking_application.h>

 //#include <fstream>
 //#include <string>
/*

 namespace masking
 {

 Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
   Eigen::Affine3d rx =
       Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
   Eigen::Affine3d ry =
       Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
   Eigen::Affine3d rz =
       Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
   return rz * ry * rx;
 }

 void MaskingApplication::generateTrajectory(DescartesTrajectory &traj)
 {
//   //Create the vector of poses
   EigenSTL::vector_Affine3d poses;

//   //Create an inout file stream
   std::ifstream indata;

//   //Read the data from the example_gool_path.csv file inside the config file of the package
   std::string filename = ros::package::getPath("masking") + "/config/robot_masking_toolpath.csv"; //  example_tool_path2

//   //Open the file
   indata.open(filename);

   std::string line;
   int lineNum = 0;

   //Create the variables for the translation and transformation
   double tx, ty, tz, rx, ry, rz;
   Eigen::Affine3d part_transform; // Translation uses tx, ty, tz
    Eigen::Affine3d transf; // Translation uses tx, ty, tz
    Matrix3d m;             // Rotation uses rx, ry, rz

   //Read the file line by line
   while (std::getline(indata, line))
   {
     lineNum++;
     if (lineNum < 3)
       continue;

     std::stringstream lineStream(line);
     std::string cell;
     Eigen::VectorXd xyzrpy(6);
     int i = 0;

     //While there is a line there with ,
     while (std::getline(lineStream, cell, ','))
     {
       // std::cout << cell << std::endl;  // joe added
       //Transformation data is in the third line of the CSV file
       //If on the third line, read the data and store in the variables
       if (lineNum == 3)
       {
         //"t" means TRANSLATE
         if (i == 0)
           tx = std::stod(cell);
         else if (i == 1)
           ty = std::stod(cell);
         else if (i == 2)
           tz = std::stod(cell);

         //"r" means ROTATE
         else if (i == 3)
           rx = std::stod(cell);
         else if (i == 4)
           ry = std::stod(cell);
         else if (i == 5)
           rz = std::stod(cell);
       }
       else  //If it isn't the third line, process data normally.
         xyzrpy(i) = std::stod(cell);

       i++;
     }

     //If it's the third line, create the translation vector with tx, ty, tz
     //If it's the third line, create the rotation matrix with rx, ry, rz
     if (lineNum == 3)
     {
       // create the transformation from world to the part
       Eigen::Affine3d tmp_rot = create_rotation_matrix(rx,ry,rz);
        std::cout << tmp_rot.matrix() << "\n" << std::endl;  // joe added
       Eigen::Affine3d tmp_transl(Eigen::Translation3d(Eigen::Vector3d(tx/1000, ty/1000, tz/1000)));
        std::cout << tmp_transl.matrix() << "\n" << std::endl;  // joe added
       part_transform = tmp_transl * tmp_rot;
      // std::cout << part_transform.matrix() << "\n" << std::endl;  // joe added

        transf = Eigen::Translation3d(Eigen::Vector3d(tx, ty, tz));
        m = AngleAxisd(rx, Vector3d::UnitX()) * AngleAxisd(ry, Vector3d::UnitY()) * AngleAxisd(rz, Vector3d::UnitZ());
       // std::cout << m << std::endl;  // joe added
       //Rotate the translation by the matrix "m" to get a combination of the translation and rotation
        transf.rotate(m);

       continue;
     }

     // get the transform from the part to the process point
     Eigen::Affine3d tmp_transl(Eigen::Translation3d(xyzrpy[0]/1000,xyzrpy[1]/1000,xyzrpy[2]/1000));
     Eigen::Affine3d tmp_rot = create_rotation_matrix(xyzrpy[3],xyzrpy[4],xyzrpy[5]);
      //std::cout << tmp_rot.matrix() << "\n" << std::endl;  // joe added

      Eigen::Vector3d pos = xyzrpy.head<3>();
      pos = pos / 1000.0;
      //std::cout << "pos: \n" << pos << std::endl;  // joe added

      Eigen::Vector3d rot = xyzrpy.tail<3>();
 //     std::cout << "rot: \n" << rot << std::endl;
      Matrix3d pt_rot;
      pt_rot = AngleAxisd(rot[0], Vector3d::UnitX()) * AngleAxisd(rot[1], Vector3d::UnitY()) * AngleAxisd(rot[2], Vector3d::UnitZ());
    //  std::cout << "pt_rot: \n" << pt_rot << std::endl;

      pt_rot *= m;




//      Eigen::Vector3d temp_x = (-1 * pos).normalized();
//      Eigen::Vector3d y_axis = (norm.cross(temp_x)).normalized();
//      Eigen::Vector3d x_axis = (y_axis.cross(norm)).normalized();

     Eigen::Affine3d pose = part_transform * (tmp_transl*tmp_rot);

//      pose.matrix().col(0).head<3>() = x_axis;
//      pose.matrix().col(1).head<3>() = y_axis;
//      pose.matrix().col(2).head<3>() = norm;
 //     pose.matrix().col(3).head<3>() = pos;
    
//     pose.matrix().col(0).head<3>() = norm ;
//     pose.matrix().col(1).head<3>() = x_axis;
//      pose.matrix().col(2).head<3>() = y_axis;
 //     pose.matrix().col(3).head<3>() = pos;

      //Transform the pose using the Affine3D pose we created above
     pose = transf * pose;
     //print pose;
     poses.push_back(pose);
   }

   //Close the input data stream
   indata.close();

   // Visualize the poses
   publishPosesMarkers(poses);
   ros::spinOnce();

    //Create the descartes trajectory points
    traj.clear();
    traj.reserve(poses.size());

    Eigen::Affine3d prev_pose;
    for (unsigned int i = 0; i < poses.size(); i++)
   {
      const Eigen::Affine3d &pose = poses[i];
      descartes_core::TrajectoryPtPtr pt;
      if (i == 0) {
          pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::AxialSymmetricPt(pose, ORIENTATION_INCREMENT, descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS));
      }
      else
      {
          double dis = (pose.matrix().col(3).head<3>() - prev_pose.matrix().col(3).head<3>()).norm() * 1000 ;
          // velocity = dis / time
          // time =  dis / vel = mm / (mm/s) = s
          descartes_core::TimingConstraint time(dis/config_.masking_tool_velocity);
          // std::cout << "\n" << pose.matrix().col(3).head<3>() << std::endl;
          // std::cout <<  pose.matrix() << std::endl;
          std::cout << "dis: " << dis << ", vel: " << config_.masking_tool_velocity << ", time: " << dis/config_.masking_tool_velocity  << std::endl;
          //
          pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::AxialSymmetricPt(pose, ORIENTATION_INCREMENT, descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS,time));
      }

      prev_pose = pose;
      //Save the points into the trajectory
      traj.push_back(pt);
    }

   ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
 }
 }

*/





// namespace masking#include <ros/ros.h>
// #include <ros/package.h>
// #include <masking/masking_application.h>

// #include <fstream>
// #include <string>

// namespace masking
// {
// void MaskingApplication::generateTrajectory(DescartesTrajectory &traj)
// {

//   using namespace descartes_core;
//   using namespace descartes_trajectory;

//   //Create the vector of poses
//   EigenSTL::vector_Affine3d poses;

//   //Create an inout file stream
//   std::ifstream indata;

//   //Read the data from the example_gool_path.csv file inside the config file of the package
//   std::string filename = ros::package::getPath("masking") + "/config/example_tool_path.csv";

//   //Open the file
//   indata.open(filename);

//   std::string line;
//   int lineNum = 0;

//   //Create the variables for the translation and transformation
//   double tx, ty, tz, rx, ry, rz;
//   Eigen::Affine3d transf; //Translation uses tx, ty, tz
//   Matrix3d m;             //Rotation uses rx, ry, rz

//   //Read the file line by line
//   while (std::getline(indata, line))
//   {
//     lineNum++;
//     if (lineNum < 3)
//       continue;

//     std::stringstream lineStream(line);
//     std::string cell;
//     Eigen::VectorXd xyzrpy(6);
//     int i = 0;

//     //While there is a line there with ,
//     while (std::getline(lineStream, cell, ','))
//     {
//       //Transformation data is in the third line of the CSV file
//       //If on the third line, read the data and store in the variables
//       if (lineNum == 3)
//       {
//         //"t" means TRANSLATE
//         if (i == 0)
//           tx = std::stod(cell);
//         else if (i == 1)
//           ty = std::stod(cell);
//         else if (i == 2)
//           tz = std::stod(cell);

//         //"r" means ROTATE
//         else if (i == 3)
//           rx = std::stod(cell);
//         else if (i == 4)
//           ry = std::stod(cell);
//         else if (i == 5)
//           rz = std::stod(cell);
//       }

//       //If it isn't the third line, process data normally.
//       else
//         xyzrpy(i) = std::stod(cell);

//       i++;
//     }

//     //If it's the third line, create the translation vector with tx, ty, tz
//     //If it's the third line, create the rotation matrix with rx, ry, rz
//     if (lineNum == 3)
//     {
//       transf = Eigen::Translation3d(Eigen::Vector3d(tx, ty, tz));
//       m = AngleAxisd(rx, Vector3d::UnitX()) * AngleAxisd(ry, Vector3d::UnitY()) * AngleAxisd(rz, Vector3d::UnitZ());

//       //Rotate the translation by the matrix "m" to get a combination of the translation and rotation
//       transf.rotate(m);

//       continue;
//     }

//     Eigen::Vector3d pos = xyzrpy.head<3>();
//     pos = pos / 1000.0;
//     Eigen::Vector3d norm = xyzrpy.tail<3>();
//     norm.normalize();

//     Eigen::Vector3d temp_x = (-1 * pos).normalized();
//     Eigen::Vector3d y_axis = (norm.cross(temp_x)).normalized();
//     Eigen::Vector3d x_axis = (y_axis.cross(norm)).normalized();

//     Eigen::Affine3d pose;

//     pose.matrix().col(0).head<3>() = x_axis;
//     pose.matrix().col(1).head<3>() = y_axis;
//     pose.matrix().col(2).head<3>() = norm;
//     pose.matrix().col(3).head<3>() = pos;

//     //Transform the pose using the Affine3D pose we created above
//     pose = transf * pose;

//     poses.push_back(pose);
//   }

//   //Close the input data stream
//   indata.close();

//   // Visualize the poses
//   publishPosesMarkers(poses);

//   //Create the descartes trajectory points
//   traj.clear();
//   traj.reserve(poses.size());

//   Eigen::Affine3d prev_pose;
//   for (unsigned int i = 0; i < poses.size(); i++)
//   {
//     const Eigen::Affine3d &pose = poses[i]; 
//     descartes_core::TrajectoryPtPtr pt;
//     if (i == 0) {
//         pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::AxialSymmetricPt(pose, ORIENTATION_INCREMENT, descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS));
//     }
//     else
//     {
//         double dis = (pose.matrix().col(3).head<3>() - prev_pose.matrix().col(3).head<3>()).norm() * 1000 ;
//         // velocity = dis / time
//         // time =  dis / vel = mm / (mm/s) = s 
//         descartes_core::TimingConstraint time(dis/config_.masking_tool_velocity);
//         // std::cout << "dis: " << dis << ", vel: " << config_.masking_tool_velocity << ", time: " << dis/config_.masking_tool_velocity  << std::endl;
//         pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::AxialSymmetricPt(pose, ORIENTATION_INCREMENT, descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS,time));
//     }

//     prev_pose = pose;
//     //Save the points into the trajectory
//     traj.push_back(pt);
//   }

//   ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
// }
// } // namespace masking
