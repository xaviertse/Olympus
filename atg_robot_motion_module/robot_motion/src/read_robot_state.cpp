// #include <masking/masking_application.h>
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
 
    int PORT = 30002; 
    struct sockaddr_in address;
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
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
    if(inet_pton(AF_INET, "192.168.1.10", &serv_addr.sin_addr)<=0)  // 127.0.0.1 
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }
  
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        return -1;
    }
 

    // valread = read( sock , buffer, 1024);
    // printf("Robot State Message:\n");
    // printf("%s\n",buffer );
    
    printf("\nFinished\n\n");
    return 0;

 
}