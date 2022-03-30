#!/usr/bin/env python
import os, sys

#folder = sys.argv[1]

def check_folder(folder_input):
    #check if folder name ends with /, os.path.exist only checks for folder with ending /
    if (folder_input[-1]=="/"):
        1#print "last char is //"
    else:
        folder_input += "/"

    #split folder into folder names
    folder_split = folder_input.split("/")
    #print folder_input
    #print folder_split
    #print len(folder_split)

    #check and create folder from base folder
    folder_re = "/" #new string
    for i in range(1,len(folder_split)-1):
        folder_re += folder_split[i] + "/"
        if os.path.exists(folder_re):
            1#print folder_re, "   <- exist"
        else:
            print folder_re, "   <- non-exist, creating.."
            try:
                os.mkdir(folder_re)
            except:
                1
            if os.path.exists(folder_re):
                print folder_re, "   <- created successfully."
            else:
                print folder_re, "   <- error creating, may already exist, please check."

def check_file(file_input):
    if os.path.exists(file_input):
        1
    else:
        foutt = file(file_input,'w')


#check for external folder and file, used for data transfer and cache
check_folder("/home/data/backup")
check_folder("/home/data/cache/URScript")
check_folder("/home/data/cache/RAPID")
check_folder("/home/data/cache/toolpath")
check_file('/home/data/cache/atg_configuration.ini')

#check for internal folder, cache for temp running files (resets when run with docker)
#filename = os.getcwd()
#print filename, "  <-getcwd"
check_folder(os.getcwd()+"/data/coupons/clusters")

