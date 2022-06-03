#!/usr/bin/env python
import os, sys
import argparse
import sys
sys.path.append('..')

PI = 3.141592653589793

parser = argparse.ArgumentParser()
parser.add_argument('-r',   '--robot', default="UR5e" ,help='robot type')
parser.add_argument('-i_file' ,'--init_fileout', default="data/coupons/initial_params.yaml" ,help='output file for initial params')
parser.add_argument('-jl_file','--joint_limit_fileout', default="data/coupons/joint_limits.yaml" ,help='output file for joint limits')
parser.add_argument('-jc',  '--joints_cur', nargs='+',default=['0', '0', '0', '0', '0', '0'])
parser.add_argument('-jmin','--joints_min', nargs='+',default=['0', '0', '0', '0', '0', '0'])
parser.add_argument('-jmax','--joints_max', nargs='+',default=['0', '0', '0', '0', '0', '0'])

args = parser.parse_args()

joint_cur = [float(args.joints_cur[0]),float(args.joints_cur[1]),float(args.joints_cur[2]),float(args.joints_cur[3]),float(args.joints_cur[4]),float(args.joints_cur[5])]
joint_min = [float(args.joints_min[0]),float(args.joints_min[1]),float(args.joints_min[2]),float(args.joints_min[3]),float(args.joints_min[4]),float(args.joints_min[5])]
joint_max = [float(args.joints_max[0]),float(args.joints_max[1]),float(args.joints_max[2]),float(args.joints_max[3]),float(args.joints_max[4]),float(args.joints_max[5])]

if(abs(joint_cur[0])>0.000001): joint_cur[0] = "{:.5f}".format(joint_cur[0]*PI/180)
if(abs(joint_cur[1])>0.000001): joint_cur[1] = "{:.5f}".format(joint_cur[1]*PI/180)
if(abs(joint_cur[2])>0.000001): joint_cur[2] = "{:.5f}".format(joint_cur[2]*PI/180)
if(abs(joint_cur[3])>0.000001): joint_cur[3] = "{:.5f}".format(joint_cur[3]*PI/180)
if(abs(joint_cur[4])>0.000001): joint_cur[4] = "{:.5f}".format(joint_cur[4]*PI/180)
if(abs(joint_cur[5])>0.000001): joint_cur[5] = "{:.5f}".format(joint_cur[5]*PI/180)

if(abs(joint_min[0])>0.000001): joint_min[0] = "{:.5f}".format(joint_min[0]*PI/180)
if(abs(joint_min[1])>0.000001): joint_min[1] = "{:.5f}".format(joint_min[1]*PI/180)
if(abs(joint_min[2])>0.000001): joint_min[2] = "{:.5f}".format(joint_min[2]*PI/180)
if(abs(joint_min[3])>0.000001): joint_min[3] = "{:.5f}".format(joint_min[3]*PI/180)
if(abs(joint_min[4])>0.000001): joint_min[4] = "{:.5f}".format(joint_min[4]*PI/180)
if(abs(joint_min[5])>0.000001): joint_min[5] = "{:.5f}".format(joint_min[5]*PI/180)

if(abs(joint_max[0])>0.000001): joint_max[0] = "{:.5f}".format(joint_max[0]*PI/180)
if(abs(joint_max[1])>0.000001): joint_max[1] = "{:.5f}".format(joint_max[1]*PI/180)
if(abs(joint_max[2])>0.000001): joint_max[2] = "{:.5f}".format(joint_max[2]*PI/180)
if(abs(joint_max[3])>0.000001): joint_max[3] = "{:.5f}".format(joint_max[3]*PI/180)
if(abs(joint_max[4])>0.000001): joint_max[4] = "{:.5f}".format(joint_max[4]*PI/180)
if(abs(joint_max[5])>0.000001): joint_max[5] = "{:.5f}".format(joint_max[5]*PI/180)

print(args.robot)
print("initial_joints",joint_cur)
print("joints_min",joint_min)
print("joints_max",joint_max)

file_name = args.init_fileout
fout = open(file_name,'w')
fout.write('#added by ATG for setup/sim launch '+args.robot+'\n')
fout.write('initial_joint_state: [' +str(joint_cur[0])+','+str(joint_cur[1])+','+str(joint_cur[2])+','+str(joint_cur[3])+','+str(joint_cur[4])+','+str(joint_cur[5])+']\n')
fout.write('trajectory/seed_pose: ['+str(joint_cur[0])+','+str(joint_cur[1])+','+str(joint_cur[2])+','+str(joint_cur[3])+','+str(joint_cur[4])+','+str(joint_cur[5])+']\n')
fout.close()

angular_speed = [3.14159,3.14159,3.14159,3.14159,3.14159,3.14159]
joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']

file_name = args.joint_limit_fileout
fout = open(file_name,'w')
if(args.robot=='UR5e'):
    angular_speed = [3.14159,3.14159,3.14159,6.14159,6.14159,6.14159]
    joint_names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
elif(args.robot=='UR10e' or args.robot=='UR10'):
    angular_speed = [2.0944,2.0944,2.0944,3.14159,3.14159,3.14159]
    joint_names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
elif(args.robot=='ABB IRB 1200'):
    angular_speed = [5.027,4.189,5.236,6.981,7.069,10.472]
    joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
elif(args.robot=='ABB IRB 2400'):
    angular_speed = [2.61799,2.61799,2.61799,6.28319,6.28319,7.85398]#from datasheet 150,150,150,360,360,450
    joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
elif(args.robot=='ABB IRB 2600'):
    angular_speed = [3.05433,3.05433,3.05433,6.28319,6.28319,8.72665]#from datasheet 175,175,175,360,360,500
    joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']


fout.write("# joint_limits.yaml allows the dynamics properties specified in the URDF to be overwritten or augmented as needed\n")
fout.write("# Specific joint properties can be changed with the keys [max_position, min_position, max_velocity, max_acceleration]\n")
fout.write("# Joint limits can be turned off with [has_velocity_limits, has_acceleration_limits]\n")
fout.write("# "+args.robot+"\n")
fout.write("joint_limits:\n")
for i in range(0,6):
    fout.write("  "+joint_names[i]+":\n")
    fout.write("    has_velocity_limits: true\n")
    fout.write("    max_velocity: "+str(angular_speed[i])+"\n")
    fout.write("    has_acceleration_limits: false\n")
    fout.write("    max_acceleration: 0\n")
    if(float(joint_min[i])>=-1000.0): fout.write("    min_position: "+str(joint_min[i])+"\n")
    if(float(joint_max[i])<= 1000.0): fout.write("    max_position: "+str(joint_max[i])+"\n")

fout.close()

print("Completed output to tool_model/ initial_param.yaml & joint_limits.yaml")


