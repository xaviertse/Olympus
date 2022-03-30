#!/usr/bin/env python
import os, sys

from_folder = sys.argv[1]
to_folder = sys.argv[2]
#folder = "clusters"

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



#check/create to_folder  ->  clear to_folder  ->  append files from from_folder to to_folder
check_folder(to_folder)

if os.path.exists(from_folder) and os.path.exists(to_folder):
        #clear to_folder
        files = os.listdir(to_folder+'/')
	for f in files:
		#print f
                try:
                    os.remove(to_folder+'/'+f)
                except:
                    os.rmdir(to_folder+'/'+f)
        #duplicate from_folder to to_folder
        files = os.listdir(from_folder+'/')
        for f in files:
                #print f
                try:
                    #os.system('cp file1.txt file7.txt')
                    os.system('cp '+from_folder+'/'+f+' '+to_folder+'/'+f)
                except:
                    print 'Duplicate fail with file: '+from_folder+'/'+f
        sys.exit(0)
else:
    if os.path.exists(from_folder)==0:
        print "Cannot find "+from_folder
    if os.path.exists(to_folder)==0:
        print "Cannot find "+to_folder
    sys.exit(1)
    #os.mkdir(folder+'/')


