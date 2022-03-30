#!/bin/bash

#HOST='192.168.125.1'
#USER="Default User"
#PSWD=robotics
#FILE=spindle_forcectrl2.mod

HOST=$1
USER="Default User"
PSWD=robotics
FILE=$2

cd /home/data/cache/RAPID/

#yourfilenames=`ls ./*.mod`
#for eachfile in $yourfilenames
#do
#   echo $eachfile
#done
echo $HOST
echo $USER
echo $FILE

wput -v -u $FILE ftp://"$USER":$PSWD@$HOST/atg/
#wput -v -u move_ctrl.mod ftp://"Default User":robotics@192.168.125.1/atg/
