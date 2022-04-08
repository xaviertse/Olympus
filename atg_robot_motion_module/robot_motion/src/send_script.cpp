//#include <robot_motion/robot_motion.h>
#include <fstream>
#include <sys/socket.h>


#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"

#include <sstream>
 
int main(int argc, char **argv)
{

    // load text file to string stream
    std::string scriptname;
    std::string robotIP;
    int robotPort=0;
    ros::init(argc, argv,"urscript_upload");
    if (ros::master::check())
    {
    ros::NodeHandle nh("~");
    nh.getParam("scriptname", scriptname);
    if(scriptname=="")
    {
        scriptname = "URScript.txt";
        ROS_INFO("Loading Current Script: %s", scriptname.c_str());
    }
    else
    {
        ROS_INFO("Loading Script: %s", scriptname.c_str());
    }
    nh.getParam("robotIP", robotIP);
    if(robotIP=="")
    {
        robotIP = "192.168.0.100";
        ROS_INFO("Loading Current robotIP: %s", robotIP.c_str());
    }
    else
    {
        ROS_INFO("Loading robotIP: %s", robotIP.c_str());
    }
    nh.getParam("robotPort", robotPort);
    if(robotPort==0)
    {
        robotPort = 30002;
        ROS_INFO("Loading Current robotPort: %s", std::to_string(robotPort).c_str());
    }
    else
    {
        ROS_INFO("Loading robotPort: %s", std::to_string(robotPort).c_str());
    }

    //std::string filename = ros::package::getPath("robot_motion") + "/config/"+param;//URScript.txt";
    std::string filename = "/home/data/cache/urscript/"+scriptname;//URScript.txt;
    std::cout<<"Reading script from: "<<filename<<std::endl;
    std::ifstream file(filename.c_str()); 
    std::stringstream ss;
    ss << file.rdbuf();
    file.close();
    std::string dataToWrite;
    dataToWrite = ss.str();
    // operations on the buffer...
 

 
    int PORT = robotPort;//30003;
    struct sockaddr_in address;
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    std::cout<<"Sending  Through port: "<<PORT<<std::endl;
    // char *hello = "Hello from client";
    char buffer[1024] = {0};
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }
  
    memset(&serv_addr, '0', sizeof(serv_addr));
  
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
      
    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, robotIP.c_str(), &serv_addr.sin_addr)<=0)  // 127.0.0.1 192.168.1.10 "192.168.0.100"
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }
  
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        return -1;
    }
    send(sock , dataToWrite.c_str() , strlen(dataToWrite.c_str()) , 0 );
    printf("File Sent to UR Robot\n");
    valread = read( sock , buffer, 1024);
    printf("%s\n",buffer );
    }
    else
    {
      std::cout<<"ROS IS OFFLINE"<<std::endl;
    }
    return 0;

 
}
