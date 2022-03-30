#!/usr/bin/env python
import string
import os, sys
import codecs

file_name = sys.argv[1]

fout = open(file_name.replace('.pcd','.txt'),'w')

f = open(file_name)
lines = f.readlines()
lines = lines[11:len(lines)]
fout.writelines([l.replace(' ',';') for l in lines])
f.close()
fout.close()
exit()

fout = open(file_name.replace('.pcd','.txt'),'w')
while True:
          line = f.readline()
          if len(line)==0:
                 break
          line = line.strip()
          if line[0].isdigit() == True or line[0] == '-':
                 L = line.split(' ')
                 fout.write(L[0])
                 for i in range(1,len(L)):
                         fout.write(';'+L[i])
                 fout.write('\n')
fout.close()
