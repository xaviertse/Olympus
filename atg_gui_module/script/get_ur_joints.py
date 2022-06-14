#!/usr/bin/env python
import os, sys


#HOST = "192.168.0.100" # The remote host
#PORT = 30004 # The same port as used by the server

import argparse
import sys
sys.path.append('..')
import rtde.rtde as rtde #robot starts moving unexpected
import rtde.rtde_config as rtde_config

#HOST = sys.argv[1] superceeded by parser
#PORT = sys.argv[2]
#file_name = sys.argv[3]
PI = 3.141592653589793

parser = argparse.ArgumentParser()
parser.add_argument('--host', default="192.168.0.100" ,help='name of host to connect to (localhost)')
parser.add_argument('--port', type=int, default=30004, help='port number (30004)')
parser.add_argument('--samples', type=int, default=0,help='number of samples to record')
parser.add_argument('--frequency', type=int, default=125, help='the sampling frequency in Herz')
parser.add_argument('--config', default=os.path.dirname(__file__)+'/record_configuration.xml', help='data configuration file to use (configuration.xml)')
parser.add_argument("--verbose", help="increase output verbosity", action="store_true")
parser.add_argument("--buffered", help="Use buffered receive which doesn't skip data", action="store_true")
parser.add_argument("--binary", help="save the data in binary format", action="store_true")
parser.add_argument("--fileout", default="data/coupons/joint_values.txt" ,help='name of host to connect to (localhost)')

args = parser.parse_args()

conf = rtde_config.ConfigFile(args.config)
output_names, output_types = conf.get_recipe('out')

print ("Reading from RTDE via:-\nIP: "+ args.host)
print ("Port: "+ str(args.port))
args.port = 30004

print ("Overwrite to use following port instead\nPort: "+ str(args.port))
con = rtde.RTDE(args.host, args.port)

try:
    con.connect()
except Exception as e:
    print(e)
    file_name = args.fileout
    fout = open(file_name,'w')
    fout.write('Invalid Connection\n')
    fout.close()
    print("Invalid connection, output to "+file_name)
    exit()

print ("Connected")
con.send_output_setup(output_names, output_types, frequency=args.frequency)
con.send_start()
print ("Started")
state = con.receive(args.binary)
#analog_IN = [state.standard_analog_output0, state.standard_analog_output1, state.standard_analog_input0, state.standard_analog_input1, state.tool_analog_input0, state.tool_analog_input1]
#print("Analog: " + str(analog_IN))
print ("Received")
j1,j2,j3,j4,j5,j6 = state.actual_q
if(j1!=0): j1="{:.2f}".format(j1*180/PI)
if(j2!=0): j2="{:.2f}".format(j2*180/PI)
if(j3!=0): j3="{:.2f}".format(j3*180/PI)
if(j4!=0): j4="{:.2f}".format(j4*180/PI)
if(j5!=0): j5="{:.2f}".format(j5*180/PI)
if(j6!=0): j6="{:.2f}".format(j6*180/PI)

print("j1: "+str(j1)+"\nj2: "+str(j2)+"\nj3: "+str(j3)+"\nj4: "+str(j4)+"\nj5: "+str(j5)+"\nj6: "+str(j6))

con.send_pause()
con.disconnect()


file_name = args.fileout
fout = open(file_name,'w')
fout.write('Valid Joints\n')
fout.write(str(j1)+'\n')
fout.write(str(j2)+'\n')
fout.write(str(j3)+'\n')
fout.write(str(j4)+'\n')
fout.write(str(j5)+'\n')
fout.write(str(j6)+'\n')
fout.close()
print("Completed output to "+file_name)


