#!/usr/bin/env python
import os, sys

folder = sys.argv[1]
#folder = "clusters"


if os.path.exists(folder):
	files = os.listdir(folder+'/')
	for f in files:
		#print f
                try:
                    os.remove(folder+'/'+f)
                except:
                    os.rmdir(folder+'/'+f)
else:
	os.mkdir(folder+'/')

