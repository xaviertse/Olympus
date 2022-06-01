#include <robot_motion/robot_motion.h>
#include <fstream>
#include <sys/socket.h>


#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <ros/package.h>

//#include "descartes_trajectory/cart_trajectory_pt.h"
#include <Eigen/Geometry>

namespace robotMotion
{
  void robotMotionApplication::generateURScript(const DescartesTrajectory& path, bool bSaveToFile)
  {
        //for read csv, only for reading on/off valve on 7th input in every line.
    std::string filename_input = "/home/data/cache/toolpath/robot_toolpath_queued.csv";
    std::ifstream indata; //Create an inout file stream
    indata.open(filename_input);
    std::string line;
    std::getline(indata, line);//read 1st line - comment
    std::getline(indata, line);//read 2nd line - comment
    std::getline(indata, line);//read 3rd line - object frame
    std::getline(indata, line);//read 4th line - flowrate feedrate jumpspeed
    double z_pos[2]= {config_.fixed_z,config_.fixed_z_lift};//get fixed z_lift and z_contact

    EigenSTL::vector_Affine3d poses;  //josh , temp for output of pose in RViz to visualize
    Eigen::Matrix3d rot_mat;          //josh , temp for output of pose in RViz to visualize

    //for writing urscript
    std::string filename = "/home/data/cache/urscript/URScript.txt";
    std::string dataToWrite;// = ss.str();
    ROS_INFO_STREAM("generating UR Script file");
    //Create MoveIT! trajectory from Descartes Trajectory
    moveit_msgs::RobotTrajectory moveit_traj;
    fromDescartesToMoveitTrajectory(path,moveit_traj.joint_trajectory);

    std::stringstream ss;

    // ss  << "def My_first_program():"  << "\n";
    // ss  << "movej([-0.7601482324296471, -1.9284112483400442, 2.4200850009312065, -2.13148960204731, -1.562351390833685, -0.9523963238633675], a=1.3962634015954636, v=1.0471975511965976)"  << "\n";
    // ss  << "movej([-0.7601145807261123, -1.925313457229536, 1.4271208291636501, -1.1406326407517442, -1.5621569587688118, -0.9518539657810257], a=1.3962634015954636, v=1.0471975511965976)"  << "\n";
    // ss  << "end"  << "\n";
    ss.clear();

    if(config_.PROCESS!="NDT")
    {
      ss << "def rosmove():\n";
      ss << "\tset_tcp(" << config_.TCP_info << ")\n";
      double velocity      = config_.FEEDRATE/1000;
      double jump_velocity = config_.JUMPSPEED/1000;
      double flowrate = 0; if(config_.OPT1!=0)flowrate=config_.OPT1/6*1.0;

      std::cout << "Feedrate = "<< velocity << "mm/s,   Flowrate = " << flowrate << "ml/s,   JumpSpeed = " << jump_velocity << "mm/s"<<std::endl;
      // std::string variable_command = "a=1.5, v=0.05,r=0.008"; // a=2.5, v=0.35,r=0.008 a=4.5, v=0.65,r=0.003
      std::stringstream variable_command,variable_command1,variable_command2,variable_command2_r0;
      //variable_command << "a=1.5, v=" << config_.masking_tool_velocity/1000 << ",r=0.0003";//r=0.001
      //variable_command << "a=1.5, v=" << config_.FEEDRATE/1000 << ",r=0.1";//r=0.001 0.0003
      variable_command1 << "a=1.5, v=";
      variable_command2 << ",r=";//0.0";//,r=0.0//0.005//5mm
      if (config_.BLEND_ZONE==0) variable_command2 << "0.0";
      else variable_command2 << config_.BLEND_ZONE/1000;
      variable_command2_r0 << ",r=0";
      // std::string s = ss.str();
//          Eigen::Affine3d cart_pose;
//          descartes_trajectory::JointTrajectoryPt::getCartesianPoses(*robot_model_ptr_.get(),cart_pose);
//          std::cout <<"cart_pose" << cart_pose.matrix() << "\n" << std::endl;

      int last_IO = 0,last_last_IO = 0;
      for(int i = 0; i < moveit_traj.joint_trajectory.points.size(); i++)
      {
        double pose_rotvec_quat[10];  //x,y,z,rx,ry,rz josh added to calculate cartesian from joint pose
        fromJointToCart(path, i, pose_rotvec_quat, rot_mat);// josh added to calculate cartesian from joint pose
//            std::cout << "fromJointToCart = " << pose_n_rot_vec[0] << ", " << pose_n_rot_vec[1] << ", " << pose_n_rot_vec[2] << ", "
//                                              << pose_n_rot_vec[3] << ", " << pose_n_rot_vec[4] << ", " << pose_n_rot_vec[5] << ", " << std::endl;
//            float feedrate_scaled = config_.FEEDRATE/1000; if (i==0) feedrate_scaled = config_.JUMPSPEED/1000;

        int curr_IO = 0;
        if(std::getline(indata, line))
        {
          std::stringstream lineStream(line);
          std::string cell;
          int cell_no = 0;
          while (std::getline(lineStream, cell, ';'))
          {
              if(cell_no==6)
              {
                std::string::size_type sz;
                curr_IO = int(std::stod(cell));
//                std::cout << "IO = " << curr_IO << std::endl << std::endl;//josh debug
              }
              cell_no++;
          }
        }


        //joe's  '\tmovel([j0,j1,j2,j3,j4,j5], a=1.5, v=feedrate_scale, r=0.0003)\t\t #process motion\n'
        //josh's '\tmovel(get_inverse_kin(p[x,y,z,rx,ry,rz],[j0,j1,j2,j3,j4,j5]), a=1.5, v=feedrate_scale, r=0.0003)\t\t #process motion\n'
        //josh's script for movel using joint angles only --START================
        double z = pose_rotvec_quat[2];
        // Situation in aplhabets, state in []
        //             [0]__B__[0]
        //               |      |
        //              A|      |C
        //   ____________|      |____________
        //  [1] E [1] E [0]    [1] D [1] D [1]
        if(last_last_IO==1 && last_IO==0 && curr_IO==0)//A, jump's start, move up slowly
        {
            if(z_pos[1]>0)z=z_pos[1];//for fixed z and lift
            ss  << "\tmovel(get_inverse_kin(p["  <<
            pose_rotvec_quat[0] << "," << //x
            pose_rotvec_quat[1] << "," << //y
            z << "," << //z //pose_rotvec_quat[2] << "," << //z
            pose_rotvec_quat[3] << "," << //Rx
            pose_rotvec_quat[4] << "," << //Ry
            pose_rotvec_quat[5] << "],[" << //Rz
            moveit_traj.joint_trajectory.points[i].positions[0] << ","   << //j1
            moveit_traj.joint_trajectory.points[i].positions[1] << ","   << //j2
            moveit_traj.joint_trajectory.points[i].positions[2] << ","   << //j3
            moveit_traj.joint_trajectory.points[i].positions[3] << ","   << //j4
            moveit_traj.joint_trajectory.points[i].positions[4] << ","   << //j5
            moveit_traj.joint_trajectory.points[i].positions[5] << "]), "<< //<< //j6
            variable_command1.str() << velocity <<
            variable_command2.str() << ")\n";
        }
        else if(last_IO==0 && curr_IO==0)//B, jump traverse, move to next quickly
        {
            if(z_pos[1]>0)z=z_pos[1];//for fixed z and lift
            ss  << "\tmovel(get_inverse_kin(p["  <<
            pose_rotvec_quat[0] << "," << //x
            pose_rotvec_quat[1] << "," << //y
            z << "," << //z //pose_rotvec_quat[2] << "," << //z
            pose_rotvec_quat[3] << "," << //Rx
            pose_rotvec_quat[4] << "," << //Ry
            pose_rotvec_quat[5] << "],[" << //Rz
            moveit_traj.joint_trajectory.points[i].positions[0] << ","   << //j1
            moveit_traj.joint_trajectory.points[i].positions[1] << ","   << //j2
            moveit_traj.joint_trajectory.points[i].positions[2] << ","   << //j3
            moveit_traj.joint_trajectory.points[i].positions[3] << ","   << //j4
            moveit_traj.joint_trajectory.points[i].positions[4] << ","   << //j5
            moveit_traj.joint_trajectory.points[i].positions[5] << "]), "<< //<< //j6
            variable_command1.str() << jump_velocity <<
            variable_command2.str() << ")\n";
        }
        else if(last_IO==0 && curr_IO==1)//C, jump end, slow move down, start dispense after reach
        {
            if(z_pos[1]>0)z=z_pos[0];//for fixed z and lift
            ss  << "\tmovel(get_inverse_kin(p["  <<
            pose_rotvec_quat[0] << "," << //x
            pose_rotvec_quat[1] << "," << //y
            z << "," << //z //pose_rotvec_quat[2] << "," << //z
            pose_rotvec_quat[3] << "," << //Rx
            pose_rotvec_quat[4] << "," << //Ry
            pose_rotvec_quat[5] << "],[" << //Rz
            moveit_traj.joint_trajectory.points[i].positions[0] << ","   << //j1
            moveit_traj.joint_trajectory.points[i].positions[1] << ","   << //j2
            moveit_traj.joint_trajectory.points[i].positions[2] << ","   << //j3
            moveit_traj.joint_trajectory.points[i].positions[3] << ","   << //j4
            moveit_traj.joint_trajectory.points[i].positions[4] << ","   << //j5
            moveit_traj.joint_trajectory.points[i].positions[5] << "]), "<< //<< //j6
            variable_command1.str() << velocity <<
            variable_command2_r0.str() << ")\n";
            ss  << "\tset_analog_out(0,0.2)" << "\n" ;//for air flowrate
            ss  << "\tset_digital_out(0,True)" << "\n" ;//for toggle air
            //ss  << "\tset_analog_out(1," << config_.FLOWRATE << ")" << "\n" ;//for eco
            ss  << "\tset_analog_out(1," << flowrate << ")" << "\n" ;//for eco
            ss  << "\tset_digital_out(1,True)" << "\n" ;//for pico toggle flowrate
            ss  << "\tset_digital_out(4,True)" << "\n" ;//for eco
            ss  << "\tsleep(0.3)" << "\n" ; //dispensing delay for eco penp 0.3
        }
        else if(last_IO==1 && curr_IO==1)//D, masking traverse, move to point slowly
        {
            if(z_pos[1]>0)z=z_pos[0];//for fixed z and lift
            ss  << "\tmovep(get_inverse_kin(p["  <<
            pose_rotvec_quat[0] << "," << //x
            pose_rotvec_quat[1] << "," << //y
            z << "," << //z //pose_rotvec_quat[2] << "," << //z
            pose_rotvec_quat[3] << "," << //Rx
            pose_rotvec_quat[4] << "," << //Ry
            pose_rotvec_quat[5] << "],[" << //Rz
            moveit_traj.joint_trajectory.points[i].positions[0] << ","   << //j1
            moveit_traj.joint_trajectory.points[i].positions[1] << ","   << //j2
            moveit_traj.joint_trajectory.points[i].positions[2] << ","   << //j3
            moveit_traj.joint_trajectory.points[i].positions[3] << ","   << //j4
            moveit_traj.joint_trajectory.points[i].positions[4] << ","   << //j5
            moveit_traj.joint_trajectory.points[i].positions[5] << "]), "<< //<< //j6
            variable_command1.str() << velocity <<
            variable_command2.str() << ")\n";
        }
        else if(last_IO==1 && curr_IO==0)//E, masking end, move to point slowly, stop mask
        {
            if(z_pos[1]>0)z=z_pos[0];//for fixed z and lift
            ss  << "\tmovel(get_inverse_kin(p["  <<
            pose_rotvec_quat[0] << "," << //x
            pose_rotvec_quat[1] << "," << //y
            z << "," << //z //pose_rotvec_quat[2] << "," << //z
            pose_rotvec_quat[3] << "," << //Rx
            pose_rotvec_quat[4] << "," << //Ry
            pose_rotvec_quat[5] << "],[" << //Rz
            moveit_traj.joint_trajectory.points[i].positions[0] << ","   << //j1
            moveit_traj.joint_trajectory.points[i].positions[1] << ","   << //j2
            moveit_traj.joint_trajectory.points[i].positions[2] << ","   << //j3
            moveit_traj.joint_trajectory.points[i].positions[3] << ","   << //j4
            moveit_traj.joint_trajectory.points[i].positions[4] << ","   << //j5
            moveit_traj.joint_trajectory.points[i].positions[5] << "]), "<< //<< //j6
            variable_command1.str() << velocity <<
            variable_command2_r0.str() << ")\n";
            ss  << "\tset_analog_out(0,0)" << "\n" ;//for air flowrate
            ss  << "\tset_digital_out(0,False)" << "\n" ;//for toggle air
            ss  << "\tset_analog_out(1,0)" << "\n" ;//for eco toggle flowrate
            ss  << "\tset_digital_out(1,False)" << "\n" ;//for pico
            ss  << "\tset_digital_out(4,False)" << "\n" ;//for eco
            ss  << "\tsleep(0.3)" << "\n" ; //dispensing delay for eco pen 0.3
        }
        last_last_IO = last_IO;
        last_IO = curr_IO;

        Eigen::Affine3d tmp_transl(Eigen::Translation3d(pose_rotvec_quat[0],pose_rotvec_quat[1],pose_rotvec_quat[2]));
        Eigen::Affine3d tmp_rot; tmp_rot = rot_mat;       //josh , temp for output of pose in RViz to visualize
        Eigen::Affine3d tmp_pose = tmp_transl *  tmp_rot; //josh , temp for output of pose in RViz to visualize
        poses.push_back(tmp_pose);                        //josh , temp for output of pose in RViz to visualize

        /*joe's script for movel using joint angles only --START================
          ss  << "\tmovel([" <<
          moveit_traj.joint_trajectory.points[i].positions[0] << ","   <<
          moveit_traj.joint_trajectory.points[i].positions[1] << ","   <<
          moveit_traj.joint_trajectory.points[i].positions[2] << ","   <<
          moveit_traj.joint_trajectory.points[i].positions[3] << ","   <<
          moveit_traj.joint_trajectory.points[i].positions[4] << ","   <<
          moveit_traj.joint_trajectory.points[i].positions[5] << "], " <<
          variable_command1.str() << feedrate_scaled <<
          variable_command2.str() << ")\t\t # process motion\n";
        //joe's script for movel using joint angles only --END===================

          if(i==2){ss  << "\tsleep(10)" << "\n" ;}//just to check zeropos
          if(i==5){//if(i==1){ //command inserted to check zero pos--line1-4
              ss  << "\tset_digital_out(1,True)" << "\n" ;//for pico
              ss  << "\tset_digital_out(2,True)" << "\n" ;//for eco
              ss  << "\tsleep(0.3)" << "\n" ; //dispensing delay for eco pen
        //originally input for turning on/off IO, no ref
          if(i==1){//if(i==1){ //command inserted to check zero pos--line1-4
              ss  << "\tset_analog_out(1," << config_.FLOWRATE << ")" << "\n" ;//for eco
              ss  << "\tset_digital_out(1,True)" << "\n" ;//for pico toggle flowrate
              ss  << "\tset_digital_out(2,True)" << "\n" ;//for eco
              ss  << "\tsleep(0.3)" << "\n" ; //dispensing delay for eco pen
          }
          if(i==moveit_traj.joint_trajectory.points.size()-2){
              ss  << "\tsleep(0.3)" << "\n" ; //dispensing delay for eco pen
              ss  << "\tset_analog_out(1,0)" << "\n" ;//for eco toggle flowrate
              ss  << "\tset_digital_out(1,False)" << "\n" ;//for pico
              ss  << "\tset_digital_out(2,False)" << "\n" ;//for eco
          }*/


      }
    }

    if(config_.PROCESS=="NDT")
    {
      ss << "def rosmove():\n";
      ss << "\toffset_num=20\n";
      ss << "\tg_waypoint_before=p[0,0,0,0,0,0]\n";
      ss << "\tg_waypoint_after=p[0,0,0,0,0,0]\n";
      ss << "\tg_waypoint_target=p[0,0,0,0,0,0]\n";
      ss << "\tg_thread_flag_4 = 0\n";
      ss << "\tg_vel_in_thread=0\n";
      ss << "\tg_Fx=0\n";
      ss << "\tg_Fy=0\n";
      ss << "\tg_Fz=0\n";
      ss << "\tg_Mx=0\n";
      ss << "\tg_My=0\n";
      ss << "\tg_Mz=0\n";
      ss << "\tthread read_force():\n";
      ss << "\t  if (not socket_open(\"127.0.0.1\", 63351, \"rq_ft_sensor_stream\")):\n";
      ss << "\t    popup(\"Can't connect to the FT Sensor driver\", \"Robotiq's FT Sensor\", error=True)\n";
      ss << "\t    halt\n";
      ss << "\t  end\n";
      ss << "\t  socket_close(\"rq_ft_sensor_stream\")\n";
      ss << "\t  if (not socket_open(\"127.0.0.1\", 63350, \"rq_ft_sensor_stream\")):\n";
      ss << "\t    popup(\"Can't connect to the FT Sensor driver\", \"Robotiq's FT Sensor\", error=True)\n";
      ss << "\t    halt\n";
      ss << "\t  end\n";
      ss << "\t  if (not socket_open(\"127.0.0.1\", 29999, \"RQDashboardClient\")):\n";
      ss << "\t    popup(\"Can't connect to the Dashboard server\", \"Robotiq's FT Sensor\", error=True)\n";
      ss << "\t    halt\n";
      ss << "\t  end\n";
      ss << "\t  while True:\n";
      ss << "\t    socket_send_string(\"READ DATA\", \"rq_ft_sensor_stream\")\n";
      ss << "\t    rq_sensor_data = socket_read_ascii_float(6, \"rq_ft_sensor_stream\")\n";
      ss << "\t    if (rq_sensor_data[0] >= 6):\n";
      ss << "\t      g_Fx = rq_sensor_data[1]\n";
      ss << "\t      g_Fy = rq_sensor_data[2]\n";
      ss << "\t      g_Fz = rq_sensor_data[3]\n";
      ss << "\t      g_Mx = rq_sensor_data[4]\n";
      ss << "\t      g_My = rq_sensor_data[5]\n";
      ss << "\t      g_Mz = rq_sensor_data[6]\n";
      ss << "\t    else:\n";
      ss << "\t      g_Fx = 0.0\n";
      ss << "\t      g_Fy = 0.0\n";
      ss << "\t      g_Fz = 0.0\n";
      ss << "\t      g_Mx = 0.0\n";
      ss << "\t      g_My = 0.0\n";
      ss << "\t      g_Mz = 0.0\n";
      ss << "\t    end\n";
      ss << "\t  end #end while\n";
      ss << "\tend #end thread\n";
      ss << "\tdef get_sensor_fz():\n";
      ss << "\t  return g_Fz\n";
      ss << "\tend\n";
      ss << "\tdef rq_zero_sensor():\n";
      ss << "\t  if (socket_open(\"127.0.0.1\", 63350, \"rq_ft_sensor_acc\")):\n";
      ss << "\t    socket_send_string(\"SET ZRO\", \"rq_ft_sensor_acc\")\n";
      ss << "\t    sleep(0.1)\n";
      ss << "\t    socket_close(\"rq_ft_sensor_acc\")\n";
      ss << "\t  end\n";
      ss << "\tend\n";
      ss << "\tthread Thread_if_4():\n";
      ss << "\t    movel(g_waypoint_before,a=1,v=g_vel_in_thread ,t=0,r=0)\n";
      ss << "\t    movel(g_waypoint_target,a=1,v=0.01,t=0,r=0)\n";
      ss << "\t    movel(g_waypoint_after ,a=1,v=0.01,t=0,r=0)\n";
      ss << "\t    g_thread_flag_4 = 1\n";
      ss << "\tend\n";
      ss << "\tdef Force_Sensing(pose_1_,offset_,f_value_,velocity_):\n";
      ss << "\t  g_vel_in_thread = velocity_\n";
      ss << "\t  g_waypoint_target = pose_1_\n";
      ss << "\t  g_waypoint_before = pose_trans(pose_1_, p[0,0,-offset_/1000,0,0,0])\n";
      ss << "\t  g_waypoint_after  = pose_trans(pose_1_, p[0,0, offset_/1000,0,0,0])\n";
      ss << "\t  rq_zero_sensor()\n";
      ss << "\t  sleep(1.0)\n";
      ss << "\t  g_thread_flag_4=0\n";
      ss << "\t  if (get_sensor_fz() >= -f_value_):\n";
      ss << "\t    global thread_handler_4 = run Thread_if_4()\n";
      ss << "\t    while (g_thread_flag_4 == 0):\n";
      ss << "\t      if not (get_sensor_fz() >= -f_value_):\n";
      ss << "\t        kill thread_handler_4\n";
      ss << "\t        g_thread_flag_4 = 2\n";
      ss << "\t      else:\n";
      ss << "\t        sync()\n";
      ss << "\t      end\n";
      ss << "\t    end\n";
      ss << "\t  else:\n";
      ss << "\t    g_thread_flag_4 = 2\n";
      ss << "\t  end\n";
      ss << "\t  global F_z = get_sensor_fz()\n";
      ss << "\t  popup(str_cat(\"Force = \",-F_z), str_cat(\"\",get_actual_tcp_pose()), False, False, blocking=False)\n";
      ss << "\tend\n";
      ss << "\trun read_force()\n\n";

      ss << "\tset_tcp(" << config_.TCP_info << ")\n";
      double velocity      = config_.FEEDRATE/1000;
      double jump_velocity = config_.JUMPSPEED/1000;
      double flowrate = 0; if(config_.OPT1!=0)flowrate=config_.OPT1/2000*10;//TBC
      double force         = config_.OPT2;
      double wait_time     = config_.OPT3;

      std::cout << "Feedrate = "<< velocity << "mm/s,   JumpSpeed = " << jump_velocity << "mm/s"<<std::endl;
      std::cout << "Flowrate = "<< flowrate << "v or " << config_.OPT1 << "RPM,   Force = " << force << "N,   Wait_Time = " << wait_time << "s"<<std::endl;

      // std::string variable_command = "a=1.5, v=0.05,r=0.008"; // a=2.5, v=0.35,r=0.008 a=4.5, v=0.65,r=0.003
      std::stringstream variable_command,variable_command1,variable_command2,variable_command2_r0;
      variable_command1 << "a=1.5, v=";
      variable_command2 << ",r=";   //0.0";//,r=0.0//0.005//5mm
      if (config_.BLEND_ZONE==0) variable_command2 << "0.0";
      else variable_command2 << config_.BLEND_ZONE/1000;
      variable_command2_r0 << ",r=0";

      int last_IO = 0,last_last_IO = 2;
      for(int i = 0; i < moveit_traj.joint_trajectory.points.size(); i++)
      {
        double pose_rotvec_quat[10];  //x,y,z,rx,ry,rz josh added to calculate cartesian from joint pose
        fromJointToCart(path, i, pose_rotvec_quat, rot_mat);// josh added to calculate cartesian from joint pose
//            std::cout << "fromJointToCart = " << pose_n_rot_vec[0] << ", " << pose_n_rot_vec[1] << ", " << pose_n_rot_vec[2] << ", "
//                                              << pose_n_rot_vec[3] << ", " << pose_n_rot_vec[4] << ", " << pose_n_rot_vec[5] << ", " << std::endl;
//            float feedrate_scaled = config_.FEEDRATE/1000; if (i==0) feedrate_scaled = config_.JUMPSPEED/1000;

        int curr_IO = 0;
        if(std::getline(indata, line))
        {
          std::stringstream lineStream(line);
          std::string cell;
          int cell_no = 0;
          while (std::getline(lineStream, cell, ';'))
          {
              if(cell_no==6)
              {
                std::string::size_type sz;
                curr_IO = int(std::stod(cell));
//                std::cout << "IO = " << curr_IO << std::endl << std::endl;//josh debug
              }
              cell_no++;
          }
        }

        // Situation in aplhabets, state in 1 or 0
        //  A  0---->0 0---->0
        //       D   | ^   D
        //          B| |C
        //           v_|
        //            1
        // A: first point, last_last_IO=2 : movep quickly to first point
        // B: 0 to 1 : move down to point with FC
        // C: 1 to 0 : move up from after FC slowly
        // D: 0 to 0 : move from jump point to jump point fast
        //
        if(last_last_IO==2)// A: first point, last_last_IO=2 : movep quickly to first point
        {
          ss  << "\tmovep(get_inverse_kin(p["  <<
          pose_rotvec_quat[0] << "," << //x
          pose_rotvec_quat[1] << "," << //y
          pose_rotvec_quat[2] << "," << //z
          pose_rotvec_quat[3] << "," << //Rx
          pose_rotvec_quat[4] << "," << //Ry
          pose_rotvec_quat[5] << "],[" << //Rz
          moveit_traj.joint_trajectory.points[i].positions[0] << ","   << //j1
          moveit_traj.joint_trajectory.points[i].positions[1] << ","   << //j2
          moveit_traj.joint_trajectory.points[i].positions[2] << ","   << //j3
          moveit_traj.joint_trajectory.points[i].positions[3] << ","   << //j4
          moveit_traj.joint_trajectory.points[i].positions[4] << ","   << //j5
          moveit_traj.joint_trajectory.points[i].positions[5] << "]), "<< //<< //j6
          variable_command1.str() << jump_velocity <<
          variable_command2.str() << ")#2\n";
        }
        else if(last_IO==0 && curr_IO==1)// B: 0 to 1 : move down to point with FC
        {
          //ss << "\tset_digital_out(0,True)" << "\n" ;//for toggle
          ss << "\tset_analog_out(0," << flowrate << ")" << "\n" ;//for watson marlow
          //ss << "\tForce_Sensing (pose_1,offset,f_value,vel)\n";
          ss << "\tForce_Sensing(p["  <<
                   pose_rotvec_quat[0] << "," << //x
                   pose_rotvec_quat[1] << "," << //y
                   pose_rotvec_quat[2] << "," << //z
                   pose_rotvec_quat[3] << "," << //Rx
                   pose_rotvec_quat[4] << "," << //Ry
                   pose_rotvec_quat[5] << "]," << //Rz
                   "offset_num," << force << ","<<velocity<<")\n";
          ss << "\tset_analog_out(0,0)" << "#01\n" ;//for watson marlow off
          ss << "\tsleep(" << wait_time << ")\n" ; // delay for reading
        }
        else if(last_IO==1 && curr_IO==0)//C: 1 to 0 : move up from after FC slowly
        {
          ss  << "\tmovel(get_inverse_kin(p["  <<
          pose_rotvec_quat[0] << "," << //x
          pose_rotvec_quat[1] << "," << //y
          pose_rotvec_quat[2] << "," << //z
          pose_rotvec_quat[3] << "," << //Rx
          pose_rotvec_quat[4] << "," << //Ry
          pose_rotvec_quat[5] << "],[" << //Rz
          moveit_traj.joint_trajectory.points[i].positions[0] << ","   << //j1
          moveit_traj.joint_trajectory.points[i].positions[1] << ","   << //j2
          moveit_traj.joint_trajectory.points[i].positions[2] << ","   << //j3
          moveit_traj.joint_trajectory.points[i].positions[3] << ","   << //j4
          moveit_traj.joint_trajectory.points[i].positions[4] << ","   << //j5
          moveit_traj.joint_trajectory.points[i].positions[5] << "]), "<< //<< //j6
          variable_command1.str() << velocity <<
          variable_command2.str() << ")\n";
          ss  << "\tsleep(0.3)" << "#10\n" ; //dispensing delay for eco pen 0.3
        }
        else if(last_IO==0 && curr_IO==0)// D: 0 to 0 : move from jump point to jump point fast
        {
            ss  << "\tmovel(get_inverse_kin(p["  <<
            pose_rotvec_quat[0] << "," << //x
            pose_rotvec_quat[1] << "," << //y
            pose_rotvec_quat[2] << "," << //z
            pose_rotvec_quat[3] << "," << //Rx
            pose_rotvec_quat[4] << "," << //Ry
            pose_rotvec_quat[5] << "],[" << //Rz
            moveit_traj.joint_trajectory.points[i].positions[0] << ","   << //j1
            moveit_traj.joint_trajectory.points[i].positions[1] << ","   << //j2
            moveit_traj.joint_trajectory.points[i].positions[2] << ","   << //j3
            moveit_traj.joint_trajectory.points[i].positions[3] << ","   << //j4
            moveit_traj.joint_trajectory.points[i].positions[4] << ","   << //j5
            moveit_traj.joint_trajectory.points[i].positions[5] << "]), "<< //<< //j6
            variable_command1.str() << jump_velocity <<
            variable_command2.str() << ")#00\n";
        }


        last_last_IO = last_IO;
        last_IO = curr_IO;

        Eigen::Affine3d tmp_transl(Eigen::Translation3d(pose_rotvec_quat[0],pose_rotvec_quat[1],pose_rotvec_quat[2]));
        Eigen::Affine3d tmp_rot; tmp_rot = rot_mat;       //josh , temp for output of pose in RViz to visualize
        Eigen::Affine3d tmp_pose = tmp_transl *  tmp_rot; //josh , temp for output of pose in RViz to visualize
        poses.push_back(tmp_pose);                        //josh , temp for output of pose in RViz to visualize

    }

    }
    // Turn off masking dispense
    // ss  << "\tset_digital_out(1,False)" << "\n" ;
    ss  << "end\n" ;

    if(bSaveToFile) {
        ROS_INFO_STREAM("\tsaving...");
        std::ofstream filePtr;
        filePtr.open(filename.c_str(), std::ofstream::out);
        // std::string dataToWrite = ss.str();
        dataToWrite.clear();
        dataToWrite = ss.str();
        filePtr << dataToWrite.c_str();
        filePtr.close();
    }
    publishPosesMarkers(poses); //josh , temp for output of pose in RViz to visualize
    ros::spinOnce();            //josh , temp for output of pose in RViz to visualize

    ROS_INFO_STREAM("Done\n\n");
  }
}
