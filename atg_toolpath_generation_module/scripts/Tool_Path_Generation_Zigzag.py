#!/usr/bin/env python
# -*- coding: utf-8 -*-
#author Xavier Xie
import cv2
import numpy as np
import matplotlib.pyplot as plt
from skimage import morphology
from skimage import img_as_ubyte
import math
import copy

import pandas as pd

import time
import sys
import os
import matplotlib.image as mpimg

def rotate_around_point_highperf(xy, radians, origin=(0, 0)):             #test point rotation   use as below
    """Rotate a point around a given point.                               #test point rotation   theta = math.radians(90)
                                                                          #test point rotation   point = (5, -11)
    I call this the "high performance" version since we're caching some   #test point rotation   print(rotate_around_point_highperf(point, theta))
    values that are needed >1 time. It's less readable than the previous  #test point rotation
    function but it's faster.                                             #test point rotation
    """                                                                   #test point rotation
    x, y = xy                                                             #test point rotation
    offset_x, offset_y = origin                                           #test point rotation
    adjusted_x = (x - offset_x)                                           #test point rotation
    adjusted_y = (y - offset_y)                                           #test point rotation
    cos_rad = math.cos(radians)                                           #test point rotation
    sin_rad = math.sin(radians)                                           #test point rotation
    qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y           #test point rotation
    qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y          #test point rotation
    return qx, qy                                                         #test point rotation
                                                                          #test point rotation

#[a, b, c, d, e] = [,,, , ]

temp_pts_sorted = []
external_on = int(sys.argv[1])
internal_on = int(sys.argv[2])
sort_pts_on = int(sys.argv[3])
resolution  = float(sys.argv[4].replace(",","."))/10
forced_resolution = int(sys.argv[5])
hole_patch_size = float(sys.argv[6].replace(",","."))
step_size   = float(sys.argv[7].replace(",","."))
offset      = float(sys.argv[8].replace(",","."))
downsample  = int(sys.argv[9])
rotation    = float(sys.argv[10].replace(",",".")) #test point rotation
reverse_toolpath = int(sys.argv[11])

step_offset = 8 #temp josh, 3 for polish, 8 for AE  superceded by resolution and step_size calculation
#pt_filter = 1 #temp josh, 1 for polish, 5 for AE superceded by downsample input

print "Toolpath_gen_SimpleCoupon -- external_on = ", external_on,"  internal_on = ",internal_on,"   sort_pts_on = ", sort_pts_on,"   resolution = ", resolution

time1 = time.time()

file_0 = file('data/coupons/transformed_boundarypoints.txt') #read the projected boundary points
boundary_pts = []
while True:
          line = file_0.readline()
          if len(line) == 0:
                 break
          line = line.strip()
          L = line.split(';')
          boundary_pts.append([float(L[0]), float(L[1]), float(L[2])])
file_0.close()

time1a = time.time()

file_2 = file('data/coupons/cloudPointsProjected.txt')
target_surface_pts = []
while True:
          line = file_2.readline()
          if len(line) == 0:
                 break
          line = line.strip()
          L = line.split(';')
          target_surface_pts.append([float(L[0]), float(L[1]), float(L[2])])
file_2.close()

time1b = time.time()

sumy=0
sumz=0

for i in range(0,len(target_surface_pts)):

          sumy=sumy+target_surface_pts[i][1]
          sumz=sumz+target_surface_pts[i][2]

centroidy=sumy/len(target_surface_pts)
centroidz=sumz/len(target_surface_pts)
print "centroidy=", centroidy
print "centroidz=", centroidz

time1c = time.time()

#josh trying to estimate point density with 20 points
#r = 45.0
boundary_pt_space_est = 0
if(1):
    tmp_avger=[]
    increament = int(len(boundary_pts)/20)
    if len(boundary_pts)<20:
        increament = 1
    for i in range(0,len(boundary_pts),increament):#for i in range(0,len(boundary_pts),5):
        tmp_spacing=[]
        for j in range(0,len(boundary_pts)):
            if(i<>j):
                ij_dist=((boundary_pts[i][1]-boundary_pts[j][1])**2+(boundary_pts[i][2]-boundary_pts[j][2])**2)**(1.0/2)
                tmp_spacing.append(ij_dist)
        #append into avger
        tmp_avger.append(min(tmp_spacing))
    boundary_pt_space_est = sum(tmp_avger)/float(len(tmp_avger))
    #r = 1.5/boundary_pt_space_est*resolution #calculate r with reference that r is size 3 in image
    print "Boundary spacing est =",boundary_pt_space_est,'r =',resolution," boundary pts=",len(boundary_pts)

#resolution = float(0.5)   #mm #shift to input value                   # resolution = size / pixel
if (resolution<boundary_pt_space_est and forced_resolution==0):
    resolution=boundary_pt_space_est

#resolution factor calculation based on input resolution and step size
#step_size =   float(25.0) #mm #shift to input value                   # size = resolition * pixel
step_pixel =  int(step_size/resolution)  #force step_pixel to be int   # pixel = size / res
print "bef1 res2 = ",resolution, " step_size = ",step_size, " step_pixel = ",step_pixel

if step_pixel<1:         #force step pixel to be min 1
    step_pixel = 1       #force step pixel to be min 1
resolution = float(step_size/step_pixel) #overwrite resolution so that resolution corrospond with step_pixel in int
print "bef2 res2 = ",resolution, " step_size = ",step_size, " step_pixel = ",step_pixel
#resolution = float(1.0/resolution)
step_offset = step_pixel-1 #temp value before reimplimenting step_offset as step_pixel in segment finding
print "aft res2 = ",resolution, " step_size = ",step_size, " step_pixel = ",step_offset

#offset = -2
offset_factor = 0                     #new offset factor
if offset!=0:                         #new offset factor
    offset_factor = offset/resolution #new offset factor
print "offset_factor=",offset_factor #new offset factor

#hole patching via dilate(+)and erode(-)
if hole_patch_size==0:
    hole_patch_size=resolution*4;
patch_factor = int(((float(hole_patch_size)/float(resolution))/2))
print "patch_factor=",patch_factor,"    hole_patch_size/resolution= ",hole_patch_size,"/",resolution

pixel_shift = 0                 #additional shift for offset/patch operations
if offset_factor>patch_factor:  #additional shift for offset/patch operations
    pixel_shift = offset_factor #additional shift for offset/patch operations
else:                           #additional shift for offset/patch operations
    pixel_shift = patch_factor  #additional shift for offset/patch operations

time2 = time.time()

#if resolution2>0:
#    r=resolution2

file_1 = file('data/coupons/cloudPointsProjected.txt') #read the projected point of the cluster
projected_pts = []
x = []
y = []
z = []                #resolution 7 times,results are depends on the density of the point cloud data:  low density = small value, high density = large value
d_x = 20.0+pixel_shift              #shift of the image along x axis
d_y = 20.0+pixel_shift              #shift of the image algong y axis
d_z = 20.0+pixel_shift
z_count = 0

#project yz into image plane      #seems like this is just repeating above readfile? projected_pts not used... - josh comment
while True:
          line = file_1.readline()
          if len(line) == 0:
                 break
          line = line.strip()
          L = line.split(';')
          x.append(float(L[0])) #x.append(int(float(L[0])*r))  #increase the resolution to pixes
          y.append(float(L[1])) #y.append(int(float(L[1])*r))
          z.append(float(L[2])) #z.append(int(float(L[2])*r))
          projected_pts.append([float(L[0]), float(L[1]), float(L[2])])
file_1.close()
time3 = time.time()

x = np.array(x)
y = np.array(y)
z = np.array(z)

c_x=(np.min(x)+np.max(x))/2                                           #test point rotation
c_y=(np.min(y)+np.max(y))/2                                           #test point rotation
c_z=(np.min(z)+np.max(z))/2                                           #test point rotation
                                                                      #test point rotation
for i in range(0,len(y)):                                             #test point rotation
    theta = math.radians(rotation)                                    #test point rotation
    point = (y[i], z[i])                                              #test point rotation
    y[i],z[i] = rotate_around_point_highperf(point, theta,(c_y,c_z))  #test point rotation

for i in range(0,len(y)):                                             #test point rotation subs the following
    x[i] = int(float(x[i])/resolution)#x[i] = int(float(x[i])*r)      #test point rotation x.append(int(float(L[0])*r))
    y[i] = int(float(y[i])/resolution)#y[i] = int(float(y[i])*r)      #test point rotation y.append(int(float(L[1])*r))
    z[i] = int(float(z[i])/resolution)#z[i] = int(float(z[i])*r)      #test point rotation z.append(int(float(L[2])*r))

x_shift = 1.0*np.min(x)
y_shift = 1.0*np.min(y)
z_shift = 1.0*np.min(z)

x = x - x_shift
y = y - y_shift
z = z - z_shift

w = np.max(x)
h = np.max(y)
d = np.max(z)

#---------------here covert the projected point cloud data into a image and processed with opencv----------------#
img = np.zeros((int(h+d_x*2),int(d+d_y*2)),np.uint8) #initialize an image with black color, slightly large than projected point cloud data
for i in range(0,len(y)):
        img[int(y[i]+d_y)][int(z[i]+d_z)] = 255  #position of the projected image shift toward the center of the image

#offset = 2
offset_factor = 0                     #new offset factor
if offset!=0:                         #new offset factor
    offset_factor = offset/resolution #new offset factor
#print "offset_factor=",offset_factor #new offset factor

#kernel = np.ones((7,7),np.uint8)                #this kernel offset too much
kernel = np.array ([[0, 1, 0],                   #this kernel offset 1 pixel more accurately
                    [1, 1, 1],                   #this kernel offset 1 pixel more accurately
                    [0, 1, 0]], dtype = np.uint8)#this kernel offset 1 pixel more accurately
img = cv2.dilate(img,kernel,iterations = patch_factor) #img = cv2.dilate(img,kernel,iterations = 2)
img = cv2.erode(img,kernel,iterations = patch_factor)  #img = cv2.erode(img,kernel,iterations = 2)
if (offset_factor>0):                                            #new offset factor outwards
    img = cv2.dilate(img,kernel,iterations = int(offset_factor)) #new offset factor outwards
if (offset_factor<0):                                               #new offset factor inwards
    img = cv2.erode(img,kernel,iterations = int(abs(offset_factor)))#new offset factor inwards
#img = cv2.erode(img,kernel,iterations = 1) #additional erode to offset inwards

cv2.imwrite('data/coupons/tmp.png',img)
#cv2.imshow('Donut', img)en number is
#cv2.waitKey(0)
img = cv2.imread('data/coupons/tmp.png')


def detech_display_0():
    os.setsid();
    plt.figure()
    plt.title('Img Plot')
    #plt.axis("off")
    plt.imshow(img, cmap = 'gray', interpolation = 'bicubic')
    plt.draw()
    plt.show()
    #_ = raw_input("Press [enter] to continue.")

if os.fork(): #deteching display so code can run in background,LINUX only
    pass
else:
    print "displaying appending lines in fork()"
    detech_display_0()
    exit()

if(1):#get external and internal edges
    im = cv2.imread('data/coupons/tmp.png')
    gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    ret, binary = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
    im2, contours, hierarchy = cv2.findContours(binary,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)#CHAIN_APPROX_SIMPLE)
    mask = np.ones(im.shape[:2], dtype="uint8") * 0
    if external_on == 1:
        cv2.drawContours(mask, contours,0, (255,0,0), 3)#3) all= -1, external= 0, internal= 1-->all
        im = cv2.bitwise_and(im, im, mask=mask)
        print "Enternal Contour appended"
    if internal_on == 1:
        contour_index = 1
        while(1):
            try:
                cv2.drawContours(mask, contours,contour_index, (255,0,0), 3)#3)#all is -1
                im = cv2.bitwise_and(im, im, mask=mask)
                print "Inner Contour",contour_index,"appended"
            except:
                print "No more inner contour"
                break
            contour_index+=1

    cv2.imwrite('data/coupons/edges.png', mask)
    edges = cv2.imread('data/coupons/edges.png')

#if external_on == 1:
#    im = cv2.imread('data/coupons/tmp.png')
#    gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
#    ret, binary = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
#    # Run findContours - Note the RETR_EXTERNAL flag
#    # Also, we want to find the best contour possible with CHAIN_APPROX_NONE
#    if internal_on == 1:
#        im2, contours, hierarchy = cv2.findContours(binary,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)#CHAIN_APPROX_SIMPLE)
#        mask = np.ones(im.shape[:2], dtype="uint8") * 0
#        cv2.drawContours(mask, contours,-1, (255,0,0), 3)#3)#all is -1
#        im = cv2.bitwise_and(im, im, mask=mask)
#        cv2.imwrite('data/coupons/edges.png', mask)
#        edges = cv2.imread('data/coupons/edges.png')
#    else:
#        im2, contours, hierarchy = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)#CHAIN_APPROX_SIMPLE)
#        mask = np.ones(im.shape[:2], dtype="uint8") * 0
#        cv2.drawContours(mask, contours,0, (255,0,0), 3)#3)
#        im = cv2.bitwise_and(im, im, mask=mask)
#        cv2.imwrite('data/coupons/edges.png', mask)
#        edges = cv2.imread('data/coupons/edges.png')
#else:
#    edges = cv2.Canny(img,150,200)
#    cv2.imwrite("data/coupons/edges.png", edges)

im = cv2.imread("data/coupons/edges.png")
im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
im = cv2.threshold(im, 0, 255, cv2.THRESH_OTSU)[1]
im = morphology.skeletonize(im > 0)
im = img_as_ubyte(im)
cv2.imwrite("data/coupons/dst.png", im)
dst = im


skel = dst/255
skel = cv2.filter2D(skel,-1,np.ones((3,3),np.float32))
idx = np.where(skel==4)
for i in range(0,len(idx[0])):
        im[idx[0][i]][idx[1][i]] = 0

skel = im
cv2.imwrite("data/coupons/skel.png", im)
ret,skel = cv2.threshold(skel,1,255,0)

#-------------plot the projection results --------------- now hidden
#plt.subplot(221),plt.imshow(img,cmap = 'gray')
#plt.title('Original'), plt.xticks([]), plt.yticks([])
#plt.subplot(222),plt.imshow(edges,cmap = 'gray')
#plt.title('Edges'), plt.xticks([]), plt.yticks([])
#plt.subplot(223),plt.imshow(dst,cmap = 'gray')
#plt.title('Dst'), plt.xticks([]), plt.yticks([])
#plt.subplot(224),plt.imshow(skel,cmap = 'gray')
#plt.title('Skel'), plt.xticks([]), plt.yticks([])
#plt.show()

#print "exit at point 1 for debugging"
#exit()

#---------------------------projected toolpath and sorting the point in anti-clockwise -------------------------------
idx = np.where(skel==255)#idx = np.where(skel==255)
temp_pts = []
tmp_y =[]
tmp_z =[]
r_correction = 0.9986
for i in range(0,len(idx[0])):
    y1 = (idx[0][i]- d_y + y_shift)*resolution+0.5*resolution#y1 = (idx[0][i]- d_y + y_shift)/r+0.5/r #minus offset in image's resolution error
    z1 = (idx[1][i]- d_z + z_shift)*resolution+0.5*resolution#z1 = (idx[1][i]- d_z + z_shift)/r+0.5/r #minus offset in image's resolution error
    tmp_y.append(y1)
    tmp_z.append(z1)
    temp_pts.append([0,y1,z1])



#fout0 = file('data/coupons/projected_tool_path.txt','w')
#for i in range(0, len(temp_pts)):
#        fout0.write(str(temp_pts[i][0])+';'+str(temp_pts[i][1])+';'+str(temp_pts[i][2])+'\n') #projected toolpath waypoints
#fout0.close()

#n = len(tmp_y)

#______________________________________________________________________________________________________________________

#replaced with pt search #fout2 = file('data/coupons/zigzag_masking_tool_path_idx_normals.txt','w')
fout3 = file('data/coupons/sampling_tool_path.txt','w')#('data/coupons/zigzag_masking_tool_path.txt','w')
final_zigzag_toolpath_pts=[]


time4 = time.time()
# -*- coding: utf-8 -*-

#-===sreekan========================================================================================================================

original = cv2.imread('data/coupons/tmp.png',0)
#ret,th1 = cv2.threshold(original,127,255,cv2.THRESH_BINARY) #for the gear images from online
img1 = cv2.bitwise_not(original)
#img1 = original
ret,th1 = cv2.threshold(img1,180,255,cv2.THRESH_BINARY)
img_plot = cv2.imread('data/coupons/tmp.png',0)
#cv2.imshow('img',img1)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
#th1 = copy.deepcopy(img1)
img2 = copy.deepcopy(img_plot)
img3 = copy.deepcopy(img_plot) #for plotting purpose
img_plot = cv2.cvtColor(img_plot, cv2.COLOR_GRAY2RGB)

#------setting the parameters----------------------------------------
margin_offset = 1; tool_dia = 0 # in pixels
col_limit = 100       # if the previous last pixel and the new start pixel columns differ more than this amount they wont be connected, instead they will be in separate sections

#------------------find the boundingbox boundary values---------------

rows,cols = th1.shape
print(rows,cols)
i_max = 0
j_max = 0
index = []
for i in range(0,rows):
                for j in range(0,cols):
                                if th1[i][j]!=255:
                                                index.append((i,j))

index_array = np.array(index)
row_min,col_min = np.amin(index_array, axis=0)
row_max,col_max = np.amax(index_array, axis=0)
print('left top corner',row_min,col_min )
print('right bottom corner',row_max,col_max)

#----------------------finding the number of segemnts, starting and ending pixel for each of the segment

Segment_list = [ [] for k in range(0,row_max+1) ] # start and end pixel for each of the segments
start_found = False
i = row_min
offset_act = step_offset  #the actual number of pixels between two parallel paths, originally 3
offset = offset_act+1
Segment_list_temp = []
for i in range(row_min,row_max,offset):
                start_found = False
                end_found = False
                for j in range(col_min,col_max):

                                if (th1[i][j]!=255 and not start_found):
                                                a = Segment_list[i]
                                                a.append(j)
                                                start_found = True
                                                end_found = False

                                elif start_found:
                                                if th1[i][j]==255:
                                                                end_found = True
                                                                a = Segment_list[i]
                                                                a.append(j-1)
                                                                start_found = False

                if end_found == False:
                                a.append(j)

                seg = Segment_list[i]
                for k in range(0,len(seg),2):
                                Segment_list_temp.append([seg[k],seg[k+1]])
                Segment_list[i] = Segment_list_temp
                Segment_list_temp = []

#----------------------------plotting----------------

#----------------first row---------------
j = row_min
S = Segment_list[j]
C = S.pop(0)
col_min = C[0]
col_max = C[1]
p1 = [j,col_min]
p2 = [j,col_max]#for storing

q1 = p1[::-1]   #for plotting
q2 = p2[::-1]

prev = p2
C_prev = [col_min,col_max]
c_avg = int((C_prev[0]+C_prev[1])/2)
#cv2.line(img1,tuple(q1),tuple(q2),(0,0,255),1)

count = 0
startFlag = 2
boundFlag = 0
a = np.array(Segment_list)
Coords_list = [ [] for k in range(0,row_max+1) ]
section_ind = -1
coords = []


while (len(a.shape)==1):
    #print('shape',len(a.shape)) #checking only
    for j in range(row_min,row_max+1,offset):
        #print('j',j) #checking only
        startFlag = startFlag+1
        S = Segment_list[j]

        if len(S)!=0:           #if the row is empty
            if startFlag==1:
                section_ind = section_ind+1
                count = count-1

                s_new = S.pop(0)
                col_min = s_new[0]
                col_max = s_new[1]
                c_avg = int((col_min+col_max)/2)
                if not (np.linalg.norm(col_max-col_min)<2*margin_offset+1+tool_dia):
                    if count%2==0:
                        p1 = [j,col_min+margin_offset]
                        p2 = [j,col_max-margin_offset]#for storing

                        q1 = p1[::-1]   #for plotting
                        q2 = p2[::-1]
                        cv2.line(img_plot,tuple(q1),tuple(q2),(0,0,255),1)#blue

                    else:
                        p1 = [j,col_max-margin_offset]
                        p2 = [j,col_min+margin_offset]

                        q1 = p1[::-1]
                        q2 = p2[::-1]
                        cv2.line(img_plot,tuple(q1),tuple(q2),(0,0,255),1)#blue

                    boundFlag = 0
                    if ((np.linalg.norm(prev[0]-p1[0])>offset) or (np.linalg.norm(prev[1]-p1[1])>col_limit)):
                        boundFlag = 1

                    whiteCount = 0
                    '''
                    for t in range(0,100):
                                    p = p1+np.subtract(prev,p1)*t/100
                                    r = int(p[0])
                                    c = int(p[1])
                                    if th1[r,c] == 255:
                                                    whiteCount = whiteCount+1
                    if whiteCount>80:
                                    boundFlag=1
                    '''

                    if(boundFlag==1):
                       section_ind = section_ind+1
                    else:
                        try:
                            cv2.line(img_plot,tuple(q_prev),tuple(q1),(255,255,0),2)#0,0,0 yellow need to check what this is, not appearing
                        except:
                            pass

                    Coords_list[section_ind].append([p1,p2])
                    boundFlag = 0


                #else:
                    #section_ind = section_ind+1

                prev = p2
                C_prev = [col_min,col_max]
                count = count+1
                c_avg = int((col_min+col_max)/2)
                #Coords_list[section_ind].append([p1,p2])

            elif (th1[j][c_avg]!=255):
                #print('in else flag')
                for s in S:
                    #print(s) #checking only [??,??]
                    if c_avg in range(s[0],s[1]):

                        #s_new = s
                        index = S.index(s)

                        s_new = S.pop(index)

                        col_min = s_new[0]
                        col_max = s_new[1]
                        #print('index',index)
                        if not (np.linalg.norm(col_max-col_min)<2*margin_offset+1+tool_dia):
                             if count%2==0:
                                 p1 = [j,col_min+margin_offset]
                                 p2 = [j,col_max-margin_offset]#for storing

                                 q1 = p1[::-1]   #for plotting
                                 q2 = p2[::-1]
                                 q_prev = prev[::-1]

                                 cv2.line(img_plot,tuple(q1),tuple(q2),(0,255,0),1)

                             else:
                                 p1 = [j,col_max-margin_offset]
                                 p2 = [j,col_min+margin_offset]

                                 q1 = p1[::-1]
                                 q2 = p2[::-1]
                                 q_prev = prev[::-1]
                                 cv2.line(img_plot,tuple(q1),tuple(q2),(0,255,0),1)

                             boundFlag = 0
                             if ((np.linalg.norm(prev[0]-p1[0])>offset) or (np.linalg.norm(prev[1]-p1[1])>col_limit)):
                                 boundFlag = 1

                             whiteCount = 0
                             '''
                             for t in range(0,100):
                                             p = p1+np.subtract(prev,p1)*t/100
                                             r = int(p[0])
                                             c = int(p[1])
                                             if th1[r,c] == 255:
                                                             whiteCount = whiteCount+1
                             if whiteCount>80:
                                             boundFlag=1
                             '''

                             if(boundFlag==1):
                                 section_ind = section_ind+1
                             else:
                                 cv2.line(img_plot,tuple(q_prev),tuple(q1),(255,111,255),2)#0,0,0 jump in hotpink

                             Coords_list[section_ind].append([p1,p2])
                             boundFlag = 0

                             break

                        #else:
                            #section_ind = section_ind+1

                prev = p2
                C_prev = [col_min,col_max]
                c_avg = int((col_min+col_max)/2)
                count = count+1
            else:
                #print('second else')
                breakFlag = 0
                #------the row just above susepected zero row--------
                col_min_prev = col_min
                col_max_prev = col_max
                p1 = [j-offset,col_min_prev]
                p2 = [j-offset,col_max_prev]#for storing
                q1 = p1[::-1]   #for plotting
                q2 = p2[::-1]

                #cv2.line(img1,tuple(q1),tuple(q2),(0,0,0),1)
                #--------------------------------------------------

                #----------check if the current row has a segment connected to the previous row
                for c in range(col_min,col_max+1):
                                if th1[j][c]!=255:
                                                breakFlag = 1

                #-----------find the nearest segment--------------
                #print('running')
                if breakFlag==1:
                    min = row_max
                    s_temp = [[0,0]]
                    for s in S:
                        for c in s:
                            temp = np.linalg.norm(np.subtract([j,c],[j,c_avg]))

                            if temp<min:
                                min = temp
                                s_temp = s
                                #print('min found')
                    index = S.index(s_temp)

                    col_min = s_temp[0]
                    col_max = s_temp[1]
                    S.pop(index)

                    #c_avg_temp = int((col_min_temp+col_max_temp)/2)
                    #------------------------------

                    if not (np.linalg.norm(col_max-col_min)<2*margin_offset+1+tool_dia):

                        if count%2==0:
                            p1 = [j,col_min+margin_offset]
                            p2 = [j,col_max-margin_offset]#for storing

                            q1 = p1[::-1]   #for plotting
                            q2 = p2[::-1]
                            q_prev = prev[::-1]

                            cv2.line(img_plot,tuple(q1),tuple(q2),(255,0,0),1)

                        else:
                            p1 = [j,col_max-margin_offset]
                            p2 = [j,col_min+margin_offset]

                            q1 = p1[::-1]
                            q2 = p2[::-1]
                            q_prev = prev[::-1]
                            cv2.line(img_plot,tuple(q1),tuple(q2),(255,0,0),1)

                        boundFlag = 0
                        if ((np.linalg.norm(prev[0]-p1[0])>offset) or (np.linalg.norm(prev[1]-p1[1])>col_limit)):
                            boundFlag = 1

                        whiteCount = 0
                        '''
                        for t in range(0,40):
                                        p = p1+np.subtract(prev,p1)*t/40
                                        r = int(p[0])
                                        c = int(p[1])
                                        if th1[r,c] == 255:
                                                        whiteCount = whiteCount+1
                        if whiteCount>100:
                                        boundFlag=1
                        '''

                        if(boundFlag==1):
                            section_ind = section_ind+1
                        else:
                            cv2.line(img_plot,tuple(q_prev),tuple(q1),(255,165,0),2)#0,0,0 jump in orange

                        #cv2.line(img_plot,tuple(q_prev),tuple(q1),(0,0,0),1)
                        Coords_list[section_ind].append([p1,p2])

                        prev = p2
                        C_prev = [col_min,col_max]



                        c_avg = int((col_min+col_max)/2)
                        count = count+1
                    #else:
                        #section_ind = section_ind+1
                else:
                    pass



        else:
            startFlag = 0
            a = np.array(Segment_list)

#cv2.imshow('withLines',img_plot)
cv2.imwrite('data/coupons/color_plot_Rim.png',img_plot)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

# ---------------final plotting---------------------------------------------
for m in Coords_list:
                prev = 0
                if (len(m)==0):
                                pass
                else:
                                for k in range(0,len(m)):
                                                p = m[k]
                                                p1 = (p[0][1],p[0][0])
                                                p2 = (p[1][1],p[1][0])

                                                cv2.line(img2,tuple(p1),tuple(p2),(255,255,0),1)

                                                if prev!=0:
                                                                cv2.line(img2,tuple(prev),tuple(p1),(255,255,0),1)
                                                prev = p2

#cv2.namedWindow('plotted',cv2.WINDOW_NORMAL)
#cv2.resizeWindow('plotted',600,600)

#cv2.imshow('plotted',img2)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
cv2.imwrite('data/coupons/Rim_plotted.png',img2)

#----Sorting the segments-----

Coords_list_final = [x for x in Coords_list if x!=[]]
Coords_list_final_copy = copy.deepcopy(Coords_list_final)
Coords_list_sorted = [[] for k in range(0,len(Coords_list_final))]

index1 = 0
index2 = 0
Coords_list_sorted[index2] = Coords_list_final[index1]
Coords_list_final.pop(index1)


index_min = 0
corner1 = [row_min,col_min]
corner2 = [row_max,col_max]
max_dist = np.linalg.norm(np.subtract(corner1,corner2))

while(len(Coords_list_final)!=0):
        min = max_dist
        p = Coords_list_sorted[index2]
        seg = p[len(p)-1]
        last_pix = seg[1]
        for m in Coords_list_final:
                first_pixel = m[0][0]
                dist = np.linalg.norm(np.subtract(first_pixel,last_pix))
                if dist<min:
                        min = dist
                        index_min = Coords_list_final.index(m)

        index2 = index2+1
        Coords_list_sorted[index2] = Coords_list_final[index_min]
        Coords_list_final.pop(index_min)

#final real-time plotting--------------------------------------------
'''
for m in Coords_list_sorted:
                prev = 0
                if (len(m)==0):
                                pass
                else:
                                for k in range(0,len(m)):
                                                p = m[k]
                                                p1 = (p[0][1],p[0][0])
                                                p2 = (p[1][1],p[1][0])
                                                #print('m[0]',m[0])
                                                cv2.line(img3,tuple(p1),tuple(p2),(255,0,0),1)

                                                if prev!=0:
                                                                cv2.line(img3,tuple(prev),tuple(p1),(255,0,0),1)
                                                prev = p2
                                                plt.imshow(img3, cmap = 'gray', interpolation = 'bicubic')
                                                plt.pause(0.01)
                                                plt.show()


plt.imshow(img3, cmap = 'gray', interpolation = 'bicubic')
plt.show()
'''
final_path=[]
#plt.ion()
#shift into detech_display #plt.imshow(img3, cmap = 'gray', interpolation = 'bicubic')
#shift into detech_display #rows,cols = img_plot.shape[0:2]
#img3 = cv2.cvtColor(img3, cv2.COLOR_GRAY2RGB)#---> superceded
for m in Coords_list_sorted:
    prev = 0
    if (len(m)==0):
        pass
    else:
        for k in range(0,len(m)):
            p = m[k]
            y1 = p[0][0];y2 = p[1][0]
            x1 = p[0][1];x2 = p[1][1]
            p1 = [x1,y1]
            p2 = [x2,y2]
            d = abs(x1-x2)
            for q in range (0, d):
                x11=x1+(x2-x1)/d*q
                final_path.append([0,x11,y1])
                #print x1, y1

            #print('m[0]',m[0])
            #cv2.line(img3,tuple(p1),tuple(p2),(0,0,255),1) #saving final image for deteched display -->superceded
            #shift into detech_display #plt.plot([x1,x2],[y1,y2],color = 'red')
            #if prev!=0:
                            #plt.plot(prev,p1,color = 'red')
            prev = p2

            #shift into detech_display #plt.pause(0.0001)
            #plt.show()


#deteching final figure of display---
#plt.imsave('data/coupons/zigzag_toolpath.png',img3) #can use Rim images instead, nicer connections lines

def detech_display_1():
    os.setsid();
    #final_path_duplicated=[]
    #plt.ion()
    plt.imshow(img3, cmap = 'gray', interpolation = 'bicubic')
    #rows,cols = img_plot.shape[0:2]
    for m in Coords_list_sorted:
        prev = 0
        if (len(m)==0):
            pass
        else:
            for k in range(0,len(m)):
                p = m[k]
                y1 = p[0][0];y2 = p[1][0]
                x1 = p[0][1];x2 = p[1][1]
                #p1 = [x1,y1]
                #p2 = [x2,y2]
                #d = abs(x1-x2)
                #for q in range (0, d):
                #    x11=x1+(x2-x1)/d*q
                #    final_path_duplicated.append([0,x11,y1])
                #    #print x1, y1

                #print('m[0]',m[0])
                #cv2.line(img3,tuple(p1),tuple(p2),(0,0,255),1) #saving final image for deteched display
                plt.plot([x1,x2],[y1,y2],color = 'red')
                #if prev!=0:
                                #plt.plot(prev,p1,color = 'red')
                #prev = p2

                plt.pause(0.0001)
                #plt.show()

    plt.close()
    im = cv2.imread('data/coupons/color_plot_Rim.png')#'data/coupons/zigzag_toolpath.png')
    plt.figure()
    plt.title('Zigzag Toolpath')
    #plt.axis("off")
    plt.imshow(im, cmap = 'gray', interpolation = 'bicubic')
    plt.draw()
    plt.show()
    #_ = raw_input("Press [enter] to continue.")

time4a = time.time()
#plt.show(block=False)
#plt.close()
if os.fork(): #deteching display so code can run in background,LINUX only
    pass
else:
    print "displaying appending lines in fork()"
    detech_display_1()
    exit()



#print final_path
p1=-1
#final_path=[]
print "****************len is equal to", len(final_path)
for p in range(0,len(final_path)):
       y3 = (final_path[p][2]- d_y + y_shift)*resolution+0.5*resolution #y3 = (final_path[p][2]- d_y + y_shift)/r+0.5/r
       z3 = (final_path[p][1]- d_z + z_shift)*resolution+0.5*resolution #z3 = (final_path[p][1]- d_z + z_shift)/r+0.5/r
       if p==0:
              final_zigzag_toolpath_pts.append([0, y3, z3])
              p1=p1+1


       else:
              if (y3-final_zigzag_toolpath_pts[p1][1])**2+(z3-final_zigzag_toolpath_pts[p1][2])**2>1.6:
                     final_zigzag_toolpath_pts.append([0, y3, z3])
                     p1=p1+1
print "p1 is equal to", p1


#test point rotation, undo rotate
for i in range(0,len(final_zigzag_toolpath_pts)):                                   #test point rotation
    theta = math.radians(rotation*-1)                                               #test point rotation
    point = (final_zigzag_toolpath_pts[i][1], final_zigzag_toolpath_pts[i][2])      #test point rotation
    final_zigzag_toolpath_pts[i][1], final_zigzag_toolpath_pts[i][2] = rotate_around_point_highperf(point, theta,(c_y,c_z))  #test point rotation

if reverse_toolpath:
    final_zigzag_toolpath_pts.reverse()#reverse zigzag points

time5 = time.time()
#-===========================================================================================================================


'''
for p in range(0,len(final_path)):
       y1 = (final_path[p][1]- d_y + y_shift)/r+0.5/r
       z1 = (final_path[p][2]- d_z + z_shift)/r+0.5/r
       if p==0:
              final_zigzag_toolpath_pts.append([0, y1, z1])
              p1=p1+1


       else:
              if (y1-final_zigzag_toolpath_pts[p1][1])**2+(z1-final_zigzag_toolpath_pts[p1][2])**2>1.6:
                     final_zigzag_toolpath_pts.append([0, y1, z1])
                     p1=p1+1
print "p1 is equal to", p1


'''

#for i in range (0, len(final_zigzag_toolpath_pts)):
#        print final_zigzag_toolpath_pts[i][1], final_zigzag_toolpath_pts[i][2],'+++++++++++'
#cv2.imshow('bw',pat)

#cv2.waitKey(10000)

#cv2.destroyAllWindows()

''' replacing matching loop to find idx using cv2 k Nearest Neighbour, josh reduced from 18s to 0.1s#

interim_index=[]
#print pat
#print "m is equal to", m
for i in range(0, len(final_zigzag_toolpath_pts),pt_filter):#filter down points by 5x
        min_dist1 = np.inf
        min_dist2 = np.inf
        #min_dist = 1000000000000000.0
        idy = 0
        idm = 0
        #print i

        for k in range(0,len(target_surface_pts)):

                  #dist = abs(target_surface_pts[j][0]-matching_pts[i][0])+abs(target_surface_pts[j][1]-matching_pts[i][1])+abs(target_surface_pts[j][2]-matching_pts[i][2])
                  dist = (target_surface_pts[k][1]-final_zigzag_toolpath_pts[i][1])**2+(target_surface_pts[k][2]-final_zigzag_toolpath_pts[i][2])**2
                  dist = math.sqrt(dist)
                  if dist<min_dist1:
                             min_dist1 = dist
                             idy = k                    #find the corresponding point cloud data point no. j
                  #idxdup.append(idy)
       # if i in interim_index:
#                  for j in range(0,len(boundary_pts)):
#                             dist = (boundary_pts[j][1]-final_zigzag_toolpath_pts[i][1])**2+(boundary_pts[j][2]-final_zigzag_toolpath_pts[i][2])**2   #Double stars (**) are exponentiation, 2**2 means 2 squared (2^2)
                             #dist = abs(boundary_pts[j][1]-matching_pts[i][1])+abs(boundary_pts[j][2]-matching_pts[i][2])
#                             if dist<min_dist2:
#                                           min_dist2 = dist
#                                           idm = j
        #          final_zigzag_toolpath_pts[i][0] = boundary_pts[idm][0]

        #else:
                  final_zigzag_toolpath_pts[i][0] = target_surface_pts[idy][0]

        print idy, " @ ", i,"/",len(final_zigzag_toolpath_pts)
        #plt.subplot(111), plt.plot(matching_pts[i][1],   matching_pts[i][2],  color+'o')
        #plt.subplot(111), plt.plot(target_surface_pts[idy][1], target_surface_pts[idy][2],color+'.')
        fout3.write(str(final_zigzag_toolpath_pts[i][0])+';'+str(final_zigzag_toolpath_pts[i][1])+';'+str(final_zigzag_toolpath_pts[i][2])+'\n') #toolpath waypoints
        fout2.write(str(idy)+'\n')
'''

#START ----- using k nearest neighbour, all data have to be put into array instead of list (this is more than 10 times faster than loop)
trainData=np.zeros((len(target_surface_pts),2))
response=np.zeros((len(target_surface_pts),1))
for i in range(0, len(target_surface_pts)):
    trainData[i][0] = target_surface_pts[i][1] #yz only
    trainData[i][1] = target_surface_pts[i][2] #yz only
    response[i]= i                             #index
trainData = np.array(trainData).astype('float32') #format to float
response = np.array(response).astype('float32')   #format to float
#print trainData #check data values
#print response #check data index
knn = cv2.ml.KNearest_create()
knn.train(trainData,cv2.ml.ROW_SAMPLE,response)

if len(final_zigzag_toolpath_pts)<10*downsample:
    pt_filter = 1
else:
    pt_filter = downsample

for i in range(0, len(final_zigzag_toolpath_pts),pt_filter):
    findthis=np.zeros((1,2))                         #array of point to find
    findthis[0][0] = final_zigzag_toolpath_pts[i][1] #array of point to find
    findthis[0][1] = final_zigzag_toolpath_pts[i][2] #array of point to find
    findthis = np.array(findthis).astype('float32')  #array of point to find
    ret,results,neighbours,dist = knn.findNearest(findthis,1)
    fout3.write(str(target_surface_pts[int(results[0])][0])+';'+str(final_zigzag_toolpath_pts[i][1])+';'+str(final_zigzag_toolpath_pts[i][2])+'\n') #toolpath waypoints
    #replaced with pt search #fout2.write(str(int(results[0]))+'\n')
#END ------ using k nearest neighbour, all data have to be put into array instead of list (this is more than 10 times faster than loop)


#fout.close()
#fout1.close()
#replaced with pt search #fout2.close() #fout2 = file('data/coupons/zigzag_masking_tool_path_idx_normals.txt','w')
fout3.close() #fout3 = file('data/coupons/zigzag_masking_tool_path.txt','w')

time6 = time.time()

print "Timing p, readfile tf_bound:  ", format(time1a-time1 , '.4f'),"s"
print "Timing p, readfile pt_cloud:  ", format(time1b-time1a, '.4f'),"s"
print "Timing p, calculate_centroid: ", format(time1c-time1b, '.4f'),"s"
print "Timing p, pt dist estimation: ", format(time2-time1c , '.4f'),"s"
print "Timing p, 2nd readfile:       ", format(time3-time2  , '.4f'),"s"
print "Timing p, Convert pts to img: ", format(time4-time3  , '.4f'),"s"
print "Timing p, cal lines and plot: ", format(time4a-time4 , '.4f'),"s"
print "Timing p, waiting for key:    ", format(time5-time4a , '.4f'),"s"
print "Timing p, pt matching:        ", format(time6-time5  , '.4f'),"s"


