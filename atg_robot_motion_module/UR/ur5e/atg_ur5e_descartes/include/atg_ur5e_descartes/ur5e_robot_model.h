/*
 * ur10_robot_model.h
 *
 *  Created on: Apr 15, 2015
 *      Author: ros-devel
 */

#ifndef ATG_UR5E_DESCARTES_UR5E_ROBOT_MODEL_H_
#define ATG_UR5E_DESCARTES_UR5E_ROBOT_MODEL_H_

#include <descartes_moveit/moveit_state_adapter.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <eigen_conversions/eigen_kdl.h>
#include <tf_conversions/tf_kdl.h>
#include <atg_ur5e_descartes/ur_moveit_plugin.h>
#include <atg_ur5e_descartes/ur_kin.h>


namespace atg_ur5e_descartes
{

const std::string UR5E_BASE_LINK = "base_link";
const std::string UR5E_TIP_LINK = "ee_link";

class UR5ERobotModel: public descartes_moveit::MoveitStateAdapter, public ur5e_ur_kinematics::URKinematicsPlugin
{
public:
  UR5ERobotModel();
  virtual ~UR5ERobotModel();

  virtual bool initialize(const std::string &robot_description, const std::string& group_name,
                            const std::string& world_frame,const std::string& tcp_frame);

  virtual bool getAllIK(const Eigen::Affine3d &pose, std::vector<std::vector<double> > &joint_poses) const;

  virtual bool getIK(const Eigen::Affine3d &pose, const std::vector<double> &seed_state,
                     std::vector<double> &joint_pose) const;

  descartes_core::Frame world_to_base_;// world to arm base
  descartes_core::Frame tool_to_tip_; // from arm tool to robot tool

};

} /* namespace atg_ur5e_descartes */

#endif /* ATG_UR5E_DESCARTES_UR5E_ROBOT_MODEL_H_ */
