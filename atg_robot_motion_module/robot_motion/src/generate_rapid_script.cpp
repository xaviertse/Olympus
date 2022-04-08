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
  int return_cell_6(std::string line)//std::__cxx11::basic_string line)
  {
      std::stringstream lineStream(line);
      std::string cell;
      int cell_no = 0;
      while (std::getline(lineStream, cell, ';'))
      {
          if(cell_no==6)
          {
            std::string::size_type sz;
            return int(std::stod(cell));
            //std::cout << "IO = " << curr_IO << std::endl << std::endl;//josh debug
          }
          cell_no++;
      }
      //if nothing found
      return 0;
  }

  void robotMotionApplication::generateRapidScript(const DescartesTrajectory& path, bool bSaveToFile)
  {
        //for read csv, only for reading on/off valve on 7th input in every line.
        std::string filename_input = "/home/data/cache/toolpath/robot_toolpath_queued.csv";
        std::ifstream indata; //Create an inout file stream
        indata.open(filename_input);
        std::string line;
        std::getline(indata, line);//read 1st line - comment
        std::getline(indata, line);//read 2nd line - comment
        std::getline(indata, line);//read 3rd line - object frame
        std::getline(indata, line);//read 4th line - flowrate feedrate
        //double z_pos[2]= {config_.fixed_z,config_.fixed_z_lift};//get fixed z_lift and z_contact

        EigenSTL::vector_Affine3d poses;  //josh , temp for output of pose in RViz to visualize
        Eigen::Matrix3d rot_mat;          //josh , temp for output of pose in RViz to visualize

        //for writing RapidScript
        std::string filename = "/home/data/cache/RAPID/abb_script.mod";
        std::string dataToWrite;// = ss.str();
        ROS_INFO_STREAM("generating RapidScript mod file");
        //Create MoveIT! trajectory from Descartes Trajectory
        moveit_msgs::RobotTrajectory moveit_traj;
        fromDescartesToMoveitTrajectory(path,moveit_traj.joint_trajectory);

        //std::cout << config_.masking_tool_velocity << std::endl;//superseeded
        std::cout << "Feedrate = "<< config_.FEEDRATE << "mm/s,   JumpSpeed = " << config_.JUMPSPEED << "mm/s" << std::endl;
        std::cout << "Force = "<< config_.Force << "N,   SpindleSpeed = " << config_.SpindleSpeed << "RPM" << std::endl;
        std::stringstream ss;
        ss.clear();


        //pre msg for RapidScript
        int velocity=config_.FEEDRATE;//mm/s
        int jump_velocity=config_.JUMPSPEED;//mm/s
        int Zone=config_.BLEND_ZONE;//int Zone=1;
        int Force=0;//config_.Force;//N
        int Spindle_Speed=0;//config_.SpindleSpeed; //actual RPM, will be recalculated in FUNC num activateSpindle
        double Blast_Dia=0;
        double Air_Flow_Rate=0;
        std::string TCP="tcp_atg";

        if(config_.PROCESS=="Polishing")
        {
        Force=config_.OPT1;
        Spindle_Speed=config_.OPT2;
        /*Script Format as below
          MODULE SpindleTester
          TASK PERS tooldata tcp_atg:=[TRUE,[[170.939,0.478131,98.3167],[0.707717,-0.00180394,0.706491,0.00180086]],[2.2264,[-0.2995,-2.2396,45.5612],[1,0,0,0],0,0,0]];
          | VAR bool finishspindle := true; ! ^-----TCP X Y Z--------^   ^------------TCP qw qx qy qz-----------^    ^---FC Calibration Variables---^
          | VAR num nVR_AN01 := 0;
          | VAR speeddata v_atg_spd := ["velocity", 500, 5000, 1000];
          | VAR speeddata v_atg_jmp := ["jump_velocity", 500, 5000, 1000];
          | VAR num nVR_Force := "Force";
          | VAR num nVR_Force_zero_contact := 0;
          | VAR num Spin := "Spindle_Speed";
          | FUNC num activateSpindle(num desired_RPM)
          | | finishspindle := False;
          | | IF desired_RPM < 0 THEN
          | |   SetDO DIR_IO, 1;
          | | ELSEIF desired_RPM > 0 THEN
          | |   SetDO START_IO, 1;
          | | ENDIF
          | | desired_RPM := Abs(desired_RPM);
          | | nVR_AN01:= Round((desired_RPM*83)/19000);
          | | IF nVR_AN01 < 50 THEN
          | |   SETGO VR_AN01,nVR_AN01;
          | | ELSE
          | | SETGO VR_AN01, 50;
          | | ENDIF
          | | RETURN nVR_AN01;
          | ENDFUNC
          | FUNC bool StopSpindle()
          | | SetGo VR_AN01, 0;
          | | SetDO START_IO, 0;
          | | SetDO DIR_IO, 0;
          | | SetDO COM_2, 0;
          | | RETURN TRUE;
          | ENDFUNC
            FUNC num cal_force_zero_contact()
              nVR_Force_zero_contact := (nVR_Force/0.8);
              RETURN nVR_Force_zero_contact;
            ENDFUNC
          | PROC main()
          | | VAR num bitforcard := 0;
          | | VAR bool terminateSpindle;
          | | VAR loaddata TestLoad:=[2.2264,[-0.2995,-2.2396,45.5612],[1,0,0,0],0,0,0];
          | | FCCalib TestLoad;
              nVR_Force_zero_contact:=cal_force_zero_contact();
          | | ConfL \\off;
         */
        ss << "MODULE SpindleTester" << std::endl;
        ss << "TASK PERS tooldata tcp_atg:=[TRUE,[" << config_.TCP_info << "],[2.28196,[1.96965,-3.18532,52.2535],[1,0,0,0],0,0,0]];"<<std::endl;
        ss << "\tVAR bool finishspindle := true;"<<std::endl;
        ss << "\tVAR num nVR_AN01 := 0;" <<std::endl;
        ss << "\tVAR speeddata v_atg_spd := [" << velocity << ", 500, 5000, 1000];"<<std::endl;
        ss << "\tVAR speeddata v_atg_jmp := [" << jump_velocity << ", 500, 5000, 1000];"<<std::endl;
        ss << "\tVAR num nVR_Force := "<<Force<<";"<<std::endl;
        ss << "\tVAR num nVR_Force_zero_contact := 0;"<<std::endl;
        ss << "\tVAR num SpinRPM := "<<Spindle_Speed<<";"<<std::endl;
        if(Spindle_Speed!=0){
        ss << "\tFUNC num activateSpindle(num desired_RPM)" <<std::endl;
        ss << "\t\tfinishspindle := False;" <<std::endl;
        //ss << "\t\tSetDO COM_2, 1;" <<std::endl;
        ss << "\t\tIF desired_RPM < 0 THEN" <<std::endl;
        ss << "\t\t\tSetDO DIR_IO, 1;" <<std::endl;
        ss << "\t\tELSEIF desired_RPM > 0 THEN" <<std::endl;
        ss << "\t\t\tSetDO START_IO, 1;" <<std::endl;
        ss << "\t\tENDIF" <<std::endl;
        ss << "\t\tdesired_RPM := Abs(desired_RPM);" <<std::endl;
        ss << "\t\tnVR_AN01:= Round((desired_RPM*83)/19000);" <<std::endl;
        ss << "\t\tIF nVR_AN01 < 50 THEN" <<std::endl;
        ss << "\t\t\tSETGO VR_AN01,nVR_AN01;" <<std::endl;
        ss << "\t\tELSE" <<std::endl;
        ss << "\t\tSETGO VR_AN01, 50;" <<std::endl;
        ss << "\t\tENDIF" <<std::endl;
        ss << "\t\tRETURN nVR_AN01;" <<std::endl;
        ss << "\tENDFUNC" <<std::endl;
        ss << "\tFUNC bool StopSpindle()" <<std::endl;
        ss << "\t\tSetGo VR_AN01, 0;" <<std::endl;
        ss << "\t\tSetDO START_IO, 0;" <<std::endl;
        ss << "\t\tSetDO DIR_IO, 0;" <<std::endl;
        ss << "\t\tSetDO COM_2, 0;" <<std::endl;
        ss << "\t\tRETURN TRUE;" <<std::endl;
        ss << "\tENDFUNC" <<std::endl;
        }
        ss << "\tFUNC num cal_force_zero_contact()" <<std::endl;
        ss << "\t\tnVR_Force_zero_contact := (nVR_Force/0.8);" <<std::endl;
        ss << "\t\tRETURN nVR_Force_zero_contact;" <<std::endl;
        ss << "\tENDFUNC" <<std::endl;
        ss << "\tPROC main()" << std::endl;
        ss << "\t\tVAR num bitforcard := 0;" <<std::endl;
        ss << "\t\tVAR bool terminateSpindle;" <<std::endl;
        ss << "\t\tVAR loaddata TestLoad:=[2.2264,[-0.2995,-2.2396,45.5612],[1,0,0,0],0,0,0];" << std::endl;
        ss << "\t\tFCCalib TestLoad;" << std::endl;
        ss << "\t\tnVR_Force_zero_contact:=cal_force_zero_contact();" << std::endl;
        ss << "\t\tConfL \\off;" << std::endl;
        }

        if(config_.PROCESS=="Sandblasting")//Sandblasting
        {
          Blast_Dia     =config_.OPT1; //not in use yet
          Air_Flow_Rate =config_.OPT2; //not in use yet
          /*Script format as below
          MODULE SandblastTest
          TASK PERS tooldata tcp_atg:=[TRUE,[[170.939,0.478131,98.3167],[0.707717,-0.00180394,0.706491,0.00180086]],[2.2264,[-0.2995,-2.2396,45.5612],[1,0,0,0],0,0,0]];
            VAR speeddata v_atg_spd := [" velocity ", 500, 5000, 1000];
            VAR speeddata v_atg_jmp := [" jump_velocity ", 500, 5000, 1000];
            PROC main()
              ConfL \\off;
          */
          ss << "MODULE SandblastTest" << std::endl;
          //ss << "TASK PERS tooldata tcp_atg:=[TRUE,[[170.939,0.478131,98.3167],[0.707717,-0.00180394,0.706491,0.00180086]],[2.2264,[-0.2995,-2.2396,45.5612],[1,0,0,0],0,0,0]];"<<std::endl;
          ss << "TASK PERS tooldata tcp_atg:=[TRUE,[" << config_.TCP_info << "],[2.5,[27.4,0,150.7],[1,0,0,0],0.056,0.007,0]];"<<std::endl;
          ss << "\tVAR speeddata v_atg_spd := [" << velocity << ", 500, 5000, 1000];"<<std::endl;
          ss << "\tVAR speeddata v_atg_jmp := [" << jump_velocity << ", 500, 5000, 1000];"<<std::endl;
          ss << "\tPROC main()" << std::endl;
          ss << "\t\tConfL \\off;" << std::endl;
        }

        int last_IO = 0;
        for(int i = 0; i < moveit_traj.joint_trajectory.points.size(); i++) {
            double pose_rotvec_quat[10];  //x,y,z,rx,ry,rz josh added to calculate cartesian from joint pose
//            double pose_n_quat[7];
            fromJointToCart(path, i, pose_rotvec_quat, rot_mat);// josh added to calculate cartesian from joint pose
//            return_quat(pose_n_rot_vec,pose_n_quat);        // josh added for quaternions from pose_n_rot_vec
//            std::cout << "fromJointToCart = " << pose_n_rot_vec[0] << ", " << pose_n_rot_vec[1] << ", " << pose_n_rot_vec[2] << ", "
//                                              << pose_n_rot_vec[3] << ", " << pose_n_rot_vec[4] << ", " << pose_n_rot_vec[5] << ", " << std::endl;
            //float velocity_scaled = velocity; if (i==0) velocity_scaled = jump_velocity;
            int curr_IO = 0;
            if(std::getline(indata, line)) curr_IO=return_cell_6(line); //read IO variable from csv
            double X = pose_rotvec_quat[0]*1000;
            double Y = pose_rotvec_quat[1]*1000;
            double Z = pose_rotvec_quat[2]*1000;
            double qw = pose_rotvec_quat[9];
            double qx = pose_rotvec_quat[6];
            double qy = pose_rotvec_quat[7];
            double qz = pose_rotvec_quat[8];
            //last_IO && curr_IO
            // 00 = Pose to move to next step using MoveL, WaitTime 5
            // 01 = Pose to move to offset, start spindle @ offset, delay than start force ctrl;
            // 11 = Pose to continue force ctrl.
            // 10 = Pose to stop force ctrl, offset than stop spindle.
            //RAPID "\tMoveL [[x,y,z],[q1,q2,q3,q4],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v50,fine,tcp_atg;"
            //RAPID "\tFCPress1LStart [[x,y,z],[q1,q2,q3,q4],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v100,\\Fz:="ForceZ",50,z"zone",tcp_atg;" //force in N
            //RAPID "\tFCPressL [[x,y,z],[q1,q2,q3,q4],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v"velocity","ForceZ",z"zone",tcp_atg;"
            //RAPID "\tFCPressEnd [[x,y,z],[q1,q2,q3,q4],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v"velocity",tcp_atg;" //for last press point
            //RAPID "\tbitforcard := activateSpindle("speed");"
            //RAPID "\tfinishspindle := StopSpindle();"
            if ((last_IO==0 && curr_IO==0))
            {
              //move to next step using MoveL
              ss <<"\t\tMoveL [["
                 <<X<<","<<Y<<","<<Z<<"],["<<qw<<","<<qx<<","<<qy<<","<<qz  //x,y,z,qw,qy,qz
                 <<"],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v_atg_jmp,fine," << TCP << ";\n";
            }
            if ((last_IO==0 && curr_IO==1))
            {
              //move to offset z=2mm using MoveL Offs
              ss << "\t\tMoveL Offs([["
                 <<X<<","<<Y<<","<<Z<<"],["<<qw<<","<<qx<<","<<qy<<","<<qz  //x,y,z,qw,qy,qz
                 << "],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],0,0,5),v_atg_jmp,fine," << TCP << ";\n";

              //start spindle @ offset and wait / start IO and wait
              if(config_.PROCESS=="Polishing" && Spindle_Speed!=0)
              {ss<<"\t\tbitforcard:=activateSpindle(SpinRPM);\n\t\tWaitTime 5;\n";}
              else if(config_.PROCESS=="Sandblasting" || (config_.PROCESS=="Polishing" && Spindle_Speed==0))
              {ss<<"\t\tSetDO START_IO,1;\n\t\tWaitTime 2;\n";}

              //start force ctrl using FCPress1LStart(no offset)
              if(Force>0.1)//for testing pos only
              {
                ss << "\t\tFCPress1LStart [["
                   <<X<<","<<Y<<","<<Z<<"],["<<qw<<","<<qx<<","<<qy<<","<<qz  //x,y,z,qw,qy,qz
                   << "],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v_atg_spd,\\Fz:=nVR_Force,50,z"<<Zone<<","<<TCP<<";\n";
              }
              else
              {
                ss <<"\t\tMoveL [["
                   <<X<<","<<Y<<","<<Z<<"],["<<qw<<","<<qx<<","<<qy<<","<<qz  //x,y,z,qw,qy,qz
                   <<"],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v_atg_spd,z"<<Zone<<","<<TCP<<";\n";
              }
            }
            if ((last_IO==1 && curr_IO==1))
            {
              //move to next step using FCPress1L
              if(Force>0.1)//for testing pos only
              {
                ss << "\t\tFCPressL [["
                   <<X<<","<<Y<<","<<Z<<"],["<<qw<<","<<qx<<","<<qy<<","<<qz  //x,y,z,qw,qy,qz
                   << "],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v_atg_spd,nVR_Force,z"<<Zone<<"," << TCP << ";\n";
              }
              else
              {
                ss <<"\t\tMoveL [["
                   <<X<<","<<Y<<","<<Z<<"],["<<qw<<","<<qx<<","<<qy<<","<<qz  //x,y,z,qw,qy,qz
                   <<"],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v_atg_spd,z"<<Zone<<"," << TCP << ";\n";
              }
            }
            if ((last_IO==1 && curr_IO==0))
            {
              //end force ctrl using FCPressLEnd
              if(Force>0.1)//for testing pos only
              {
                ss << "\t\tFCPressEnd [["
                   <<X<<","<<Y<<","<<Z+5<<"],["<<qw<<","<<qx<<","<<qy<<","<<qz  //x,y,z,qw,qy,qz
                   << "],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v_atg_spd\\ZeroContactValue:=nVR_Force_zero_contact,"<< TCP <<";\n";
              }
              else
              {
                ss <<"\t\tMoveL [["
                  <<X<<","<<Y<<","<<Z+5<<"],["<<qw<<","<<qx<<","<<qy<<","<<qz  //x,y,z,qw,qy,qz
                   <<"],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v_atg_spd,z"<<Zone<<"," << TCP << ";\n";
              }

              //offset z=2mm using MoveL Offs
              ss << "\t\tWaitTime 2;\n";
              ss << "\t\tMoveL Offs([["
                 <<X<<","<<Y<<","<<Z<<"],["<<qw<<","<<qx<<","<<qy<<","<<qz  //x,y,z,qw,qy,qz
                 << "],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],0,0,25),v_atg_spd,z1," << TCP << ";\n";

              //stop spindle / stop IO
              if(config_.PROCESS=="Polishing" && Spindle_Speed!=0)
              {ss<<"\t\tterminateSpindle := StopSpindle();\n";}
              if(config_.PROCESS=="Sandblasting" || (config_.PROCESS=="Polishing" && Spindle_Speed==0))
              {ss<<"\t\tSetDO START_IO,0;\n";}
            }
            last_IO = curr_IO;

            Eigen::Affine3d tmp_transl(Eigen::Translation3d(pose_rotvec_quat[0],pose_rotvec_quat[1],pose_rotvec_quat[2]));
            Eigen::Affine3d tmp_rot; tmp_rot = rot_mat;       //josh , temp for output of pose in RViz to visualize
            Eigen::Affine3d tmp_pose = tmp_transl *  tmp_rot; //josh , temp for output of pose in RViz to visualize
            poses.push_back(tmp_pose);                        //josh , temp for output of pose in RViz to visualize
        }
        // End Program
        ss << "\tENDPROC" << std::endl;
        ss << "ENDMODULE" << std::endl;

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
