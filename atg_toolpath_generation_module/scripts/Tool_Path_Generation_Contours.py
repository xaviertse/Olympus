#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import matplotlib.pyplot as plt
from skimage import morphology
from skimage import img_as_ubyte
import math
import copy
import scipy.interpolate as interpolate
from scipy.interpolate import splev, splrep
from scipy.misc import comb
#from pykalman import KalmanFilter

import time
import sys
import os

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

#[a, b, c, d, e, f, g] = [int(sys.argv[1]),int(sys.argv[2]),int(sys.argv[3]), float(sys.argv[4].replace(",",".")), float(sys.argv[5].replace(",",".")), float(sys.argv[6].replace(",",".")), float(sys.argv[7].replace(",","."))]

temp_pts_sorted = []
external_on = int(sys.argv[1])
internal_on = int(sys.argv[2])
sort_pts_on = int(sys.argv[3])
#resolution = float(sys.argv[4].replace(",","."))
resolution = float(sys.argv[4].replace(",","."))/10
forced_resolution = int(sys.argv[5])
hole_patch_size = float(sys.argv[6].replace(",","."))
offset     = float(sys.argv[7].replace(",","."))
downsample = int(sys.argv[8])
rotation =   float(sys.argv[9].replace(",","."))  #test point rotation
reverse_toolpath = int(sys.argv[10])
include_percent_start = float(sys.argv[11].replace(",","."))
include_percent_end   = float(sys.argv[12].replace(",","."))
print "Toolpath generation for boundary -- external_on = ", external_on,"  internal_on = ",internal_on,"   sort_pts_on = ", sort_pts_on,"   resolution = ", resolution
print "Toolpath generation for boundary -- downsample = ", downsample,"  rotation = ",rotation,"  reverse_toolpath = ",reverse_toolpath,"  /%start = ",include_percent_start,"  /%end = ",include_percent_end

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

#file_2 = file('data/coupons/target_surface.txt')
file_2 = file('data/coupons/cloudPointsProjected.txt')
target_surface_pts = []
rect=[]
while True:
          line = file_2.readline()
          if len(line) == 0:
                 break
          line = line.strip()
          L = line.split(';')
          target_surface_pts.append([float(L[0]), float(L[1]), float(L[2])])
          rect.append([float(L[1]), float(L[2])])

file_2.close()
#rect1=np.array(rect)
#box=cv2.minAreaRect(rect1)
#(x, y), (width, height), angle = box

time1b = time.time()

#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #file_5 = file('data/coupons/normals.txt')
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #normals = []
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #while True:
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #          line = file_5.readline()
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #          if len(line) == 0:
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #                 break
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #          line = line.strip()
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #          L = line.split(';')
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #          normals.append([float(L[0]), float(L[1]), float(L[2])])
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #file_5.close()

#josh trying to estimate point density with 10 points
#r=45
boundary_pt_space_est = 0
if(1):
    tmp_avger=[]
    increament = int(len(boundary_pts)/20)
    if len(boundary_pts)<20:
        increament = 1
    for i in range(0,len(boundary_pts),increament):
        tmp_spacing=[]
        for j in range(0,len(boundary_pts)):
            if(i<>j):
                ij_dist=((boundary_pts[i][1]-boundary_pts[j][1])**2+(boundary_pts[i][2]-boundary_pts[j][2])**2)**(1.0/2)
                tmp_spacing.append(ij_dist)
        #append into avger
        tmp_avger.append(min(tmp_spacing))
    boundary_pt_space_est = sum(tmp_avger)/float(len(tmp_avger))
    #r = 2/boundary_pt_space_est*resolution #calculate r with reference that r is size 3 in image
    #r=1.0
    #r=math.sqrt(len(target_surface_pts)/(width*height))
    print "Boundary spacing est =",boundary_pt_space_est,'r =',resolution," boundary pts=",len(boundary_pts)

#resolution = float(0.5)   #mm #shift to input value                   # = size/pixel
if (resolution<boundary_pt_space_est and forced_resolution==0):
    resolution=boundary_pt_space_est
print "aft res = ",resolution

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

file_1 = file('data/coupons/cloudPointsProjected.txt') #read the projected point of the cluster
projected_pts = []
x = []
y = []
z = []
#r = 45.0                 #resolution 7 times,results are depends on the density of the point cloud data:  low density = small value, high density = large value
d_x = 20.0+pixel_shift              #shift of the image along x axis
d_y = 20.0+pixel_shift              #shift of the image algong y axis
d_z = 20.0+pixel_shift
z_count = 0

#project yz into image plane
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
#x_shift = 0.0
#y_shift = 0.0
#z_shift = 0.0

x = x - x_shift
y = y - y_shift
z = z - z_shift


w = np.max(x)
h = np.max(y)
d = np.max(z)

#---------------here covert the projected point cloud data into a image and processed with opencv----------------#
img = np.zeros((int(math.floor(h+d_y*2)),int(math.floor(d+d_z*2))),np.uint8) #initialize an image with black color, slightly large than projected point cloud data
for i in range(0,len(y)):
        img[int(math.floor(y[i]+d_y))][int(math.floor(z[i]+d_z))] = 255  #position of the projected image shift toward the center of the image


#kernel = np.ones((3,3),np.uint8)                #this kernel offset too much
kernel = np.array ([[0, 1, 0],                   #this kernel offset 1 pixel more accurately
                    [1, 1, 1],                   #this kernel offset 1 pixel more accurately
                    [0, 1, 0]], dtype = np.uint8)#this kernel offset 1 pixel more accurately
img = cv2.dilate(img,kernel,iterations = patch_factor) #img = cv2.dilate(img,kernel,iterations = 2)
img = cv2.erode(img,kernel,iterations = patch_factor)  #img = cv2.erode(img,kernel,iterations = 2)
if (offset_factor>0):                                            #new offset factor outwards
    img = cv2.dilate(img,kernel,iterations = int(offset_factor)) #new offset factor outwards
if (offset_factor<0):                                               #new offset factor inwards
    img = cv2.erode(img,kernel,iterations = int(abs(offset_factor)))#new offset factor inwards



cv2.imwrite('data/coupons/tmp.png',img)
#cv2.imshow('Donut', img)
#cv2.waitKey(0)
img = cv2.imread('data/coupons/tmp.png')


#Xavier added cornner detection for segmentation of boundary
img = cv2.imread('data/coupons/tmp.png')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
# find Harris corners
gray = np.float32(gray)
dst = cv2.cornerHarris(gray,9,5,0.1)#2,3,0.04
dst = cv2.dilate(dst,None)
ret, dst = cv2.threshold(dst,0.01*dst.max(),255,0)
dst = np.uint8(dst)
# find centroids
ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
# define the criteria to stop and refine the corners
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
corners = cv2.cornerSubPix(gray,np.float32(centroids),(5,5),(-1,-1),criteria)
# Now draw them
res = np.hstack((centroids,corners))
res = np.int0(res)
#print "centroids=",centroids
#print "corners", corners
img[res[:,1],res[:,0]]=[255,0,0]
img[res[:,3],res[:,2]] = [0,255,0]
#img[int(math.floor(500))][int(math.floor(600))] = 255 #test point rotation
img[int(math.floor(c_y+d_y))][int(math.floor(c_z+d_z))] = 255 #test point rotation
print "C_y=",c_y,' C_z =',c_z
cv2.imwrite('data/coupons/corner_detection.png',img)
'''show first figure ------ hidden
plt.subplot(111),plt.imshow(img,cmap = 'gray')
plt.title('corner'), plt.xticks([]), plt.yticks([])
'''
time4 = time.time()
#plt.show()
time4a = time.time()

#Xavier added simplified version of corner detection

#img = cv2.imread('data/coupons/tmp.png')
#gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#gray = np.float32(gray)
#dst = cv2.cornerHarris(gray,9,5,0.1)
#result is dilated for marking the corners, not important
#dst = cv2.dilate(dst,None)
# Threshold for an optimal value, it may vary depending on the image.
#img[dst>0.01*dst.max()]=[0,0,255]
#cv2.imshow('dst',img)
#if cv2.waitKey(0) & 0xff == 27:
#    cv2.destroyAllWindows()

#plt.subplot(111),plt.imshow(img,cmap = 'gray')
#plt.title('corner'), plt.xticks([]), plt.yticks([])
#plt.show()
fout3 = file('data/coupons/contour_number.txt','w')

if(1):#get external and internal edges
    im = cv2.imread('data/coupons/tmp.png')
    gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    ret, binary = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
    im2, contours, hierarchy = cv2.findContours(binary,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)#CHAIN_APPROX_SIMPLE)
    mask = np.ones(im.shape[:2], dtype="uint8") * 0
    fout3.write(str(len(contours))+'\n')
    if external_on == 1 and internal_on ==0:
        cv2.drawContours(mask, contours,0, (255,0,0), 3)#3) all= -1, external= 0, internal= 1-->all
        im = cv2.bitwise_and(im, im, mask=mask)
        print "Enternal Contour appended"

    if internal_on == 1 and external_on == 0:
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
    if external_on == 1 and internal_on ==1:

            for i in range(0, len(contours)):

                cv2.drawContours(mask, contours,i, (255,255,0), 3)#3)#all is -1
                im = cv2.bitwise_and(im, im, mask=mask)


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
kernel1 = np.ones((5,5),np.float32)/25
im = cv2.filter2D(im,-1,kernel1)


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


#-------------plot the projection results ---------------
def detech_display_1():
    os.setsid();
    plt.subplot(221),plt.imshow(img,cmap = 'gray')
    plt.title('Original'), plt.xticks([]), plt.yticks([])
    plt.subplot(222),plt.imshow(edges,cmap = 'gray')
    plt.title('Edges'), plt.xticks([]), plt.yticks([])
    plt.subplot(223),plt.imshow(dst,cmap = 'gray')
    plt.title('Dst'), plt.xticks([]), plt.yticks([])
    plt.subplot(224),plt.imshow(skel,cmap = 'gray')
    plt.title('Skel'), plt.xticks([]), plt.yticks([])
    plt.draw()
    plt.show()
    #_ = raw_input("Press [enter] to continue.")

time4b = time.time()
if os.fork(): #deteching display so code can run in background,LINUX only
    pass
else:
#if 1:
    detech_display_1()
    exit()
#plt.show()
#p=mp.Process(target=detech_display_1)
#p.start()
time4c = time.time()
#print "exit at point 1 for debugging"
#exit()

#---------------------------projected toolpath and sorting the point in anti-clockwise -------------------------------
idx = np.where(skel==255)#idx = np.where(skel==255)
temp_pts = []
tmp_y =[]
tmp_z =[]
r_correction = 0.9986
print '++++++++++++++++len idx', len(idx[0])
for i in range(0,len(idx[0])):
    #y1 = (idx[0][i]- d_y + y_shift)/(r*r_correction)-1.5/r #minus offset in image's resolution error
    #z1 = (idx[1][i]- d_z + z_shift)/(r*r_correction)-1.5/r #minus offset in image's resolution error
    y1 = (idx[0][i]- d_y + y_shift)*resolution+0.5*resolution#y1 = (idx[0][i]- d_y + y_shift)/r+0.5/r #minus offset in image's resolution error
    z1 = (idx[1][i]- d_z + z_shift)*resolution+0.5*resolution#z1 = (idx[1][i]- d_z + z_shift)/r+0.5/r #minus offset in image's resolution error
    tmp_y.append(y1)
    tmp_z.append(z1)
    temp_pts.append([0,y1,z1])

#just looking at resoluted points vs boundary points
#for i in range(0, len(boundary_pts)):
#    plt.subplot(111), plt.plot(boundary_pts[i][1], boundary_pts[i][2],'r.')
#for i in range(0, len(temp_pts)):
#    plt.subplot(111), plt.plot(temp_pts[i][1], temp_pts[i][2],'b.')
#plt.axes().set_aspect('equal', 'datalim'), plt.title("Original Boundary [r] vs Image Pts [b]")
#plt.show()


#replacing all tmp x and y with direct boundary pts.
#tmp_y = []
#tmp_z = []

#for i in range(0,len(boundary_pts)):
#    tmp_y.append(boundary_pts[i][1])
#    tmp_z.append(boundary_pts[i][2])
#    temp_pts.append([0,boundary_pts[i][1],boundary_pts[i][2]])


fout0 = file('data/coupons/projected_tool_path.txt','w')
for i in range(0, len(temp_pts)):
        fout0.write(str(temp_pts[i][0])+';'+str(temp_pts[i][1])+';'+str(temp_pts[i][2])+'\n') #projected toolpath waypoints
fout0.close()

time5 = time.time()

n = len(tmp_y)
'''
def clock(y, z):    #-------------------sort 2d point according to clockwise neighbour
    start = time.time()
    angles = []
    (y0, z0) = (np.sum(y)/n, np.sum(z)/n)  # centroid

    for j in range(n):
        (dy, dz) = (y[j] - y0, z[j] - z0)
        angles.append(math.degrees(math.atan2(float(dz), float(dy))))
    for k in range(n):
        angles.append(angles[k] + 800)
    #print(angles)

    a = np.append(y,z)
    results = [copy.copy(x) for (y,x) in sorted(zip(angles,a), key=lambda pair: pair[0])]
    #print("result is: ", results)
    for i in range(0,n):
        temp_pts_sorted.append([0, results[i], results[i+n]])
    end = time.time()
    print 'clock - ', (end - start)  #just to view time spent for routine
clock(tmp_y,tmp_z)'''
time6 = time.time()

temp_pts_sorted2=[]
#-------------------sort 2d point according to closest neighbour Official Name <Iterative closest point>
#                                                   https://en.wikipedia.org/wiki/Point_set_registration
def soortie(y, z):
    start = time.time() #to clock time
    yy=[]               #temp y array
    zz=[]               #temp z array
    sort_y = []         #temp y sorted array
    sort_z = []         #temp z sorted array
    print "There are",len(y),"no. of points to sort for soortie"
    #copy y and z array into yy and zz, y and z are pointers, manipulating them changes the original array
    for j in range(n):
        yy.append(y[j])
        zz.append(z[j])

    #initiate 1st y and z point into sort
    #remove 1st y and z point from array to compare
    #compare each point in yy&zz array with each othe#append first point to close loop up
    #append yy&zz with min to next sort y&z array
    #pop (remove) min yy&zz from array and repeat until all is checked
    sort_y.append(yy[0])
    sort_z.append(zz[0])
    yy.pop(0)
    zz.pop(0)
    for j in range(n-1): #outer loop
        p_dis=[]
        for i in range(len(yy)): #inner loop
            #p_dis.append(math.sqrt((yy[i]-sort_y[j])**2 + (zz[i]-sort_z[j])**2))#vector dis, 2D
            p_dis.append(abs(yy[i]-sort_y[j]) + abs(zz[i]-sort_z[j])) #also vector dis but NOT the actual hypotenuse, runs faster
        minindex = np.argmin(p_dis) #find min in p_dis array's index
        mindist = p_dis[minindex]   #store value, but not important, only for viewing
        sort_y.append(yy[minindex]) #get closest yy and append into sort_y
        sort_z.append(zz[minindex]) #get closest zz and append into sort_z
        yy.pop(minindex)            #remove/pop closest yy
        zz.pop(minindex)            #remove/pop closest zz
        #print j,'-',sort_y[j],'---',sort_z[j],'mindist=',mindist,'  remaining',len(yy)

#    #direct adding into array, no filtering out abnomalies
#    for i in range(0,len(sort_y)):
#        temp_pts_sorted2.append([0, sort_y[i],sort_z[i]])

#    sort_y.reverse()
#    sort_z.reverse()
    #initiate appending to array while calculating for "skipped" points
    #"skipped" points are defined as points which are 2x the average rate
    yzavg=[] #array for calculating yz average
    yzbig=[] #index of array with big diff
    yzskip=[]#index to skip
    yzavgValue = 0.0
    #find dist btw all sorted values, append into yzavg[]
    for i in range(0,len(sort_y)):
        yzdiff=0
        if(i==0):
            yzdiff=abs(sort_y[i]-sort_y[len(sort_y)-1]) + abs(sort_z[i]-sort_z[len(sort_y)-1])#[0]-[last]
        else:
            yzdiff=abs(sort_y[i]-sort_y[i-1]) + abs(sort_z[i]-sort_z[i-1])
        yzavg.append(yzdiff)
    yzavgValue = sum(yzavg)/float(len(yzavg)) #real average
    #find all abnormally large diff, append into yzbig[]
    for i in range(0,len(yzavg)):
        if(yzavg[i]>3*yzavgValue):
            yzbig.append(i)
            print "Index with large value to skip -", i
    #check if any yzbig exist, check if large diff is in clusters of 20 or less, exclude them
    if(len(yzbig)>1):
        for i in range(0, len(yzbig)):
            p1 = yzbig[i]
            p2 = yzbig[(i+1)%len(yzbig)]
            if (p2<p1): p2+=len(yzavg)
            print "yzloop",i,"p2-p1 =",p2,"-",p1," = ",p2-p1
            if(p2-p1==1):
                print "\tNeed to remove -",p1
                yzskip.append(p1)
            elif(p2-p1<20):
                print "\tNeed to remove btw -",p1,"&",p2
                for j in range(0,p2-p1):
                    yzskip.append(p1+j)

    #put all sorted values back into temp_pts_sorted2
#    for i in range(0,len(sort_y)):
#        temp_pts_sorted2.append([0, sort_y[i],sort_z[i]])
    #put sorted values back into temp_pts_sorted2 but ignore indexes in yzskip
    for i in range(0,len(sort_y)):
        if(len(yzskip)>0):
            toskip = 0
            for j in range(0,len(yzskip)):
                if(i==yzskip[j]%len(yzavg)):
                    print "Removing Index -",i
                    toskip=1
            if (toskip==0):
               temp_pts_sorted2.append([0, sort_y[i],sort_z[i]])
        else:
            temp_pts_sorted2.append([0, sort_y[i],sort_z[i]])

    #temp_pts_sorted2.reverse() #can toggle to be default dir or reverse
    end = time.time()
    print 'soortie - ', (end - start) #just to view time spent for routine
#end soortie-------------------------------

soortie(tmp_y,tmp_z)
time7 = time.time()

print '------------------------'
sampling_pts = []
if sort_pts_on == 0:
    temp_pts_sorted = temp_pts_sorted
    #temp_pts_sorted = temp_pts

'''
yarray1=[]  #for building line graph
zarray1=[]  #for building line graph
yarray2=[]  #for building line graph
zarray2=[]  #for building line graph
yarray3=[]  #for building line graph
zarray3=[]  #for building line graph

step = int(n/200) #show 150 points in graph
if (step<1): step=1
for i in range(0,len(tmp_y),step):
    #plt.subplot(131), plt.plot(tmp_y[i], tmp_z[i],'.')
    yarray1.append(tmp_y[i])
    zarray1.append(tmp_z[i])
    plt.subplot(131), plt.text(tmp_y[i], tmp_z[i], str(i/step))
    plt.title('temp points'), plt.xticks([]), plt.yticks([])
    #plt.subplot(132), plt.plot(temp_pts_sorted[i][1], temp_pts_sorted[i][2],'*')
    yarray2.append(temp_pts_sorted[i][1])
    zarray2.append(temp_pts_sorted[i][2])
    plt.subplot(132), plt.text(temp_pts_sorted[i][1], temp_pts_sorted[i][2],str(i/step))
    plt.title('sorted clock points'), plt.xticks([]), plt.yticks([])
    #plt.subplot(133), plt.plot(temp_pts_sorted2[i][1], temp_pts_sorted2[i][2],'*')
    if (len(temp_pts_sorted2)>i):#error handling, temp_pts_sorted2 might be smaller as points might be omitted
        yarray3.append(temp_pts_sorted2[i][1])
        zarray3.append(temp_pts_sorted2[i][2])
        plt.subplot(133), plt.text(temp_pts_sorted2[i][1], temp_pts_sorted2[i][2],str(i/step))
        plt.title('sorted sortie points'), plt.xticks([]), plt.yticks([])
#plt.axes().set_aspect('equal', 'datalim')
#plt.subplot(131), plt.plot(yarray1,zarray1,"-")#to show in lines
#plt.subplot(132), plt.plot(yarray2,zarray2,"-")#to show in l#append first point to close loop up
#plt.subplot(133), plt.plot(yarray3,zarray3,"-")#to show in lines
#plt.show()


yarray1=[]
zarray1=[]
for i in range(0,len(temp_pts_sorted2)):
    if (i%step==0):
        plt.subplot(111), plt.text(temp_pts_sorted2[i][1], temp_pts_sorted2[i][2],str(i/step))
    yarray1.append(temp_pts_sorted2[i][1])
    zarray1.append(temp_pts_sorted2[i][2])#append first point to close loop up

#plt.subplot(111), plt.text(temp_pts_sorted2[len(temp_pts_sorted2)-1][1], temp_pts_sorted2[len(temp_pts_sorted2)-1][2],str(len(temp_pts_sorted2)-1)+"L")
#plt.subplot(111), plt.plot(yarray1,zarray1,"-")#to show min lines
#plt.axes().set_aspect('equal', 'datalim'), plt.title("Sortiee Points")
#plt.show()
'''

time8 = time.time()
#print "exiting debug at point 2"
#exit()#<===================================================
if sort_pts_on == 0:
    temp_pts_sorted = temp_pts_sorted
    #temp_pts_sorted = temp_pts
else:
    temp_pts_sorted=temp_pts_sorted2

#-------josh remove specific portion of toolpath if needed
#omit_percent_start = 64.32
#omit_percent_end   = 100.0
#temp_pts_remain    = []
#if omit_percent_start<>omit_percent_end:
#    for i in range(0,len(temp_pts_sorted),1):
#        i_percent = float(i)/(len(temp_pts_sorted)-1)*100.0

#        if not (i_percent>=omit_percent_start and i_percent<=omit_percent_end):
#            temp_pts_remain.append(temp_pts_sorted[i])
#            print i_percent, " in"
#        else:
#            print i_percent, " out"
#    temp_pts_sorted = temp_pts_remain
#    plt.show()
#-------josh include specific portion of toolpath if needed
if reverse_toolpath:
    temp_pts_sorted.reverse() #can toggle to be default dir or reverse


#include_percent_start = f #shift to top
#include_percent_end   = g #shift to top
#include_percent_start = float(0.05)
#include_percent_end   = float(64.40)
#include_percent_start = float(64.5)
#include_percent_end   = float(100)
temp_pts_remain    = []
if include_percent_start<>include_percent_end:
    for i in range(0,len(temp_pts_sorted)):
        i_percent = float(i)/(len(temp_pts_sorted)-1)*100.0

        if (i_percent>=include_percent_start and i_percent<=include_percent_end):
            temp_pts_remain.append(temp_pts_sorted[i])
            #print i_percent, " in" #TROUBLESHOOT DISPLAY
        else:
            #print i_percent, " out" #TROUBLESHOOT DISPLAY
            1
    temp_pts_sorted = temp_pts_remain

#-------josh trying to sort/filter detect corners------
time9 = time.time()

step = 3 #filter down to 5 points per step
sampling_pts_j = []
angle_compare = []
angle_added = []
for i in range(0,len(temp_pts_sorted),step):
    sampling_pts_j.append([0,temp_pts_sorted[i][1],temp_pts_sorted[i][2]]) #len from previous point, x, y

#just displaying avg point distances for debugging
sum_j = 0.0
for i in range(0,len(sampling_pts_j),1): #calculating avg len from point to point
    i_compare = i - 1
    if i_compare < 0 : #underflow compare
        i_compare = len(sampling_pts_j)+i_compare
    #print i,i_compare, len(sampling_pts_j)
    len_compared = abs(sampling_pts_j[i][2]-sampling_pts_j[i_compare][2])+abs(sampling_pts_j[i][1]-sampling_pts_j[i_compare][1])
    sum_j+=len_compared
    sampling_pts_j[i][0]=len_compared
#    plt.subplot(111), plt.plot(sampling_pts_j[i][1], sampling_pts_j[i][2],'.')
#    plt.subplot(111), plt.text(sampling_pts_j[i][1],m sampling_pts_j[i][2],sampling_pts_j[i][0])
#    plt.title('sampling pts Len'), plt.xticks([]), plt.yticks([])
#plt.axes().set_aspect('equal', 'datalim')
#print "avg=", sum_j/len(sampling_pts_j)
#plt.show()
len_avg = sum_j/len(sampling_pts_j)

#----- JOSH RUNNING B-SPLINE START ----------------
#-- Only good for straight lines and non vertical lines in xy axis... polyfit=1 and lentobreak = 4x
#for i in range(0,len(sampling_pts_j),1):
#    plt.subplot(111), plt.plot(sampling_pts_j[i][1], sampling_pts_j[i][2],'y.')
    #plt.subplot(111), plt.plot(sampling_pts_j[i][2], sampling_pts_j[i][1],'g.')#reversed
#filter_pts_j = []
#line = 0
#i=0
#while (i<len(sampling_pts_j)):
#for i in range(0,len(sampling_pts_j),1):
#    x=[]
#    y=[]
#    whilecount = 0
#    line+=1
#    while 1: #look for straightsamplint_tool_path_idx lines continously till point too far from line
#        i_next = i+1+whilecount;
#        if (i_next>=len(sampling_pts_j)):
#            i_next-=len(sampling_pts_j)
#        x.append(sampling_pts_j[i+whilecount][1])   #build up x array
#        y.append(sampling_pts_j[i+whilecount][2])   #build up y array
#        z1 = np.polyfit(x,y,1)                      #build x and y array into b-spline, 1 for straight lines
#        p1 = np.poly1d(z1)
#        yvals = p1(x)
#        lensub = abs(x[whilecount]-sampling_pts_j[i_next][1])+abs(yvals[whilecount]-sampling_pts_j[i_next][2]) #last point in line to next point compare
        #lensub = abs(x[whilecount]-sampling_pts_j[i_next][2])+abs(yvals[whilecount]-sampling_pts_j[i_next][1]) #last point in line to next point compare,reversed
 #       lentobreak = 5*(len_avg)  #4.5 for straight lines, just an estimate of how far to break for line
#        print "count=",whilecount+i,lensub,"vs",lentobreak,"line->",line
#        whilecount+=1
#        if (lensub>lentobreak):
#            for ii in range(0, len(x)):
#                filter_pts_j.append([0, x[ii], yvals[ii]])
#                if (line%3==1):
#                    plt.subplot(111), plt.plot(x[ii], yvals[ii],'b.')
#                elif (line%3==2):
#                    plt.subplot(111), plt.plot(x[ii], yvals[ii],'c.')
#                else:
 #                   plt.subplot(111), plt.plot(x[ii], yvals[ii],'m.')
 #           print "break becoz of lensub"
 #           break;
#        if (i+whilecount>=len(sampling_pts_j)):
#            for ii in range(0, len(x)):
#                filter_pts_j.append([0, x[ii], yvals[ii]])
#                plt.subplot(111), plt.plot(x[ii], yvals[ii],'ro')
#            print "break becoz of i+whilecount"
#            break;
#
#    i+=whilecount
#for j in range(0,len(filter_pts_j)):
#    plt.subplot(111), plt.plot(filter_pts_j[j][1], filter_pts_j[j][2],'.')

#plt.axes().set_aspect('equal', 'datalim'), plt.title("Sampled pts vs B-Spline Fitted pts")
#plt.show()

#----- JOSH RUNNING B-PLINE END ---------------------

#------------------------------------------------------skel curve-----------------------



#------------------------------------------spline fitting the toolpath-------------------------------------------------
step = 8   #step size for sampling point from the sorted toolpath
size = 0
#for i in range(0, int(len(temp_pts_sorted)/step)):
    #size = size + 1
    #sampling_pts.append([0, temp_pts_sorted[i*step][1],temp_pts_sorted[i*step][2]])
for i in range(0, len(temp_pts_sorted),step):
    size = size + 1
    sampling_pts.append([0, temp_pts_sorted[i][1],temp_pts_sorted[i][2]])

steps = 4  #no of points for spline fitting
fitting_pts = []
count = 0
for i in range(0, int(size/steps)):
    x = []
    y = []
    for j in range(i*steps, i*steps+steps):
        x.append(sampling_pts[j][1])
        y.append(sampling_pts[j][2])

#    xmin = x.min()
#    xmax = x.max()
    tck,u=interpolate.splprep([x,y],s=None)
    xnew,ynew= splev(u,tck)#append first point to close loop up

    #print x
    #print y
              #z1 = np.polyfit(x, y, 2)
              #p1 = np.poly1d(z1)
    #   print(p1)
    #xx = []
    #increment = (np.max(x)-np.min(x))/steps
    #for j in range(0, steps):
        #xx.append(increment*j+np.min(x))
               # yvals=p1(x)
    for i in range(0, len(x)):
        fitting_pts.append([0, xnew[i], ynew[i]])
        count = count + 1
j = count
for i in range(j,size):
    fitting_pts.append([0, sampling_pts[i][1], sampling_pts[i][2]])
    count = count + 1

print '------------------------'
print size
print count
print len(fitting_pts)

def detech_display_2():
    os.setsid()
    yarray1=[]
    zarray1=[]
    for i in range(0, size):
        #plt.subplot(121), plt.plot(sampling_pts[i][1], sampling_pts[i][2], '.',label='original values')
        #plt.subplot(122), plt.plot(fitting_pts[i][1], fitting_pts[i][2], '.',label='polyfit values')
        if (i<10):
            plt.subplot(111), plt.plot(sampling_pts[i][2], sampling_pts[i][1], 'c.')
            plt.subplot(111), plt.plot( fitting_pts[i][2],  fitting_pts[i][1], 'ro')
        else:#
            plt.subplot(111), plt.plot(sampling_pts[i][2], sampling_pts[i][1], 'c.')
            plt.subplot(111), plt.plot( fitting_pts[i][2],  fitting_pts[i][1], 'b*')
        yarray1.append(fitting_pts[i][2])
        zarray1.append(fitting_pts[i][1])
    plt.subplot(111), plt.plot(yarray1, zarray1, 'b-'), plt.title("sampled pts[c] vs line fitted pts[b]")
    plt.axes().set_aspect('equal', 'datalim')
    plt.gca().invert_yaxis()
    #plt.axis('off')
    plt.gca().get_xaxis().set_visible(False)
    plt.gca().get_yaxis().set_visible(False)
    plt.draw()
    plt.show()
    #_ = raw_input("Press [enter] to continue.")

time9a = time.time()
if os.fork(): #deteching display so code can run in background,LINUX only
    pass
else:
#if 1:
    detech_display_2()
    exit()
#plt.show()
#p2=mp.Process(target=detech_display_2)
#p2.start()

time10 = time.time()

fout = file('data/coupons/sampling_tool_path.txt','w')
#replaced idx matching loop with knearest #fout1 = file('data/coupons/sampling_tool_path_idx.txt','w')
#replaced idx matching loop with knearest #fout2 = file('data/coupons/sampling_tool_path_idx_normals.txt','w')
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #fout5 = file('data/coupons/Kalman_filter_normals.txt','w')
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #before_normals=[]
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #kalman_normals=[]

#sampling_pts =temp_pts_sorted
matching_pts=[] #use for comparison with original boundary points to get normal and original points sorted
#print 'point sizes =', len(fitting_pts)
#for i in range(0, len(fitting_pts)):
#    matching_pts.append(fitting_pts[i])

#test point rotation, undo rotate
for i in range(0,len(sampling_pts)):                                   #test point rotation
    theta = math.radians(rotation*-1)                                          #test point rotation
    point = (sampling_pts[i][1], sampling_pts[i][2])                   #test point rotation
    sampling_pts[i][1], sampling_pts[i][2] = rotate_around_point_highperf(point, theta,(c_y,c_z))  #test point rotation

print 'point sizes =', len(sampling_pts)
for i in range(0, len(sampling_pts)):
    matching_pts.append(sampling_pts[i])

#if (sort_pts_on):
#    matching_pts.append(sampling_pts[0]) #for closing gap in boundary pts, no need for filling operations
#else:
#    temp_pts_sorted3=[]
#    yy=[]               #temp y array
#    zz=[]               #temp z array
#    print "There are",len(matching_pts),"no. of points to sort for lefttoright"
#    #copy y and z array into yy and zz, y and z are pointers, manipulating them changes the original array
#    for j in range(len(matching_pts)):
#        yy.append(matching_pts[j][1])
#        zz.append(matching_pts[j][2])
#    for j in range(len(matching_pts)):
#        minindex = np.argmin(yy)
#        temp_pts_sorted3.append([0, yy[minindex],zz[minindex]])
#        yy.pop(minindex)            #remove/pop closest yy
#        zz.pop(minindex)            #remove/pop closest zz
#    matching_pts = temp_pts_sorted3

time11 = time.time()

''' replacing matching loop to find idx using cv2 k Nearest Neighbour, josh reduced from 18s to 0.1s#
idxdup=[]
for j in range(0,len(boundary_pts)):
    plt.subplot(111), plt.plot(boundary_pts[j][1], boundary_pts[j][2],'y.')

#xavier getting the correct normals for boundary points
for i in range(0, len(matching_pts)):
        min_dist = np.inf
        #min_dist = 1000000000000000.0
        idx = 0
        #idx1 = 0
        #print i
        for j in range(0,len(boundary_pts)):
                dist = (boundary_pts[j][1]-matching_pts[i][1])**2+(boundary_pts[j][2]-matching_pts[i][2])**2   #Double stars (**) are exponentiation, 2**2 means 2 squared (2^2)
                #dist = abs(boundary_pts[j][1]-matching_pts[i][1])+abs(boundary_pts[j][2]-matching_pts[i][2])
                if dist<min_dist:
                   min_dist = dist
                   idx = j                    #find the corresponding point cloud data point no. j
                   idxdup.append(idx)
        color = 'r'
        if(i%5==1):
            color='b'
        elif(i%5==2):
            color='g'
        elif(i%5==3):
            color='m'
        elif(i%5==4):
            color='c'

#        plt.subplot(111), plt.plot(matching_pts[i][1],   matching_pts[i][2],  color+'o')
#        plt.subplot(111), plt.plot(boundary_pts[idx][1], boundary_pts[idx][2],color+'.')
        matching_pts[i][0] = boundary_pts[idx][0]      #find the Z value of the projected point cloud data
        #print boundary_pts[idx][0] #just to verify Z value
        #matching_pts[i][1] = boundary_pts[idx][1]
        #matching_pts[i][2] = boundary_pts[idx][2]
        fout.write(str(matching_pts[i][0])+';'+str(matching_pts[i][1])+';'+str(matching_pts[i][2])+'\n') #toolpath waypoints
        #fout.write(str(matching_pts[i][0])+';'+str(matching_pts[i][1])+';'+str(matching_pts[i][2])+'\n') #toolpath waypoints

        fout1.write(str(idx)+'\n')

#append first point to close loop up
#if external_on == 1 and internal_on ==0:
#     matching_pts.append(matching_pts[0])
i=len(matching_pts)-1
#fout.write(str(matching_pts[i][ 0])+';'+str(matching_pts[i][1])+';'+str(matching_pts[i][2])+'\n') #toolpath waypoints
fout1.write(str(idxdup[0])+'\n')

for i in range(0, len(matching_pts)):
        min_dist = np.inf
        #min_dist = 1000000000000000.0
        idy = 0
        #print i
        for k in range(0,len(target_surface_pts)):
                #dist = abs(target_surface_pts[j][0]-matching_pts[i][0])+abs(target_surface_pts[j][1]-matching_pts[i][1])+abs(target_surface_pts[j][2]-matching_pts[i][2])
                dist = (target_surface_pts[k][1]-matching_pts[i][1])**2+(target_surface_pts[k][2]-matching_pts[i][2])**2
                dist = math.sqrt(dist)
                if dist<min_dist:
                   min_dist = dist
                   idy = k                    #find the corresponding point cloud data point no. j
                   #idxdup.append(idy)
        print idy, " @ ", i,"/",len(matching_pts)

        #plt.subplot(111), plt.plot(matching_pts[i][1],   matching_pts[i][2],  color+'o')
        #plt.subplot(111), plt.plot(target_surface_pts[idy][1], target_surface_pts[idy][2],color+'.')

        fout2.write(str(idy)+'\n')
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #        before_normals.append([normals[idy][0],normals[idy][1],normals[idy][2]])
        #append first point to close loop up
        #matching_pts.append(matching_pts[0])
        #i=len(matching_pts)-1
        #fout.write(str(matching_pts[i][0])+';'+str(matching_pts[i][1])+';'+str(matching_pts[i][2])+'\n') #toolpath waypoints
        #fout1.write(str(idxdup[0])+'\n')
        #color = 'r'
        #if(i%5==1):
        #    color='b'
        #elif(i%5==2):
        #    color='g'
        #elif(i%5==3):
        #    color='m'
        #elif(i%5==4):
        #    color='c'
        #plt.subplot(111), plt.plot(matching_pts[i][1],   matching_pts[i][2],  color+'o')
        #plt.subplot(111), plt.plot(boundary_pts[idx][1], boundary_pts[idx][2],color+'.')
        #matching_pts[i][0] = boundary_pts[idx][0]      #find the Z value of the projected point cloud data
        #print boundary_pts[idx][0] #just to verify Z value#append first point to close loop up
        #matching_pts.append(matching_pts[0])
        #i=len(matching_pts)-1
        #fout.write(str(matching_pts[i][0])+';'+str(matching_pts[i][1])+';'+str(matching_pts[i][2])+'\n') #toolpath waypoints
        #fout1.write(str(idxdup[0])+'\n')
        #matching_pts[i][1] = boun#append first point to close loop up

        #matching_pts.append(matching_pts[0])
        #i=len(matching_pts)-1
        #fout.write(str(matching_pts[i][0])+';'+str(matching_pt s[i][1])+';'+str(matching_pts[i][2])+'\n') #toolpath waypoints
        #fout1.write(str(idxdup[0])+'\n')ts[i][0])+';'+str(matching_pts[i][1])+';'+str(matching_pts[i][2])+'\n') #toolpath waypoints
        #fout1.write(str(idxdup[0])+'\n')dary_pts[idx][1]
        #matching_pts[i][2] = boundary_pts[idx][2]
        #fout.write(str(matching_pts[i][0])+';'+str(matching_pts[i][1])+';'+str(matching_pts[i][2])+'\n') #toolpath waypoints
'''
#xavier added the Kalman Filter to smoothen the normals

#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #measurements1=[]
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #measurements2=[]
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #measurements3=[]
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##states_pred=[]
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #for p in range (0,len(before_normals)):
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##  measurements1 = [before_normals[i][0], before_normals[i][1], before_normals[i][2]] # 3 observations
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #           measurements1.append(before_normals[p][0]) # 3 observations
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #           measurements2.append(before_normals[p][1])
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #           measurements3.append(before_normals[p][2])
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #kf = KalmanFilter(transition_matrices = [[1, 1], [0, 1]], transition_covariance=0.1 * np.eye(2))
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##kf = KalmanFilter(initial_state_mean=0, n_dim_obs=3)
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##kf = kf.em(measurements1, n_iter=5)
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##(filtered_state_means1, filtered_state_covariances1) = kf.filter(measurements1)
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##(smoothed_state_means1, smoothed_state_covariances1) = kf.smooth(measurements1)
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #states_pred1 = kf.em(measurements1).smooth(measurements1)[0]
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #print states_pred1
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #kf = KalmanFilter(transition_matrices = [[1, 1], [0, 1]], transition_covariance=0.1 * np.eye(2))
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##kf = KalmanFilter(initial_state_mean=0, n_dim_obs=3)
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##kf = kf.em(measurements1, n_iter=5)
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##(filtered_state_means1, filtered_state_covariances1) = kf.filter(measurements1)
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##(smoothed_state_means1, smoothed_state_covariances1) = kf.smooth(measurements1)
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #states_pred2 = kf.em(measurements2).smooth(measurements2)[0]
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #print states_pred2
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #kf = KalmanFilter(transition_matrices = [[1, 1], [0, 1]], transition_covariance=0.1 * np.eye(2))
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##kf = KalmanFilter(initial_state_mean=0, n_dim_obs=3)
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##kf = kf.em(measurements1, n_iter=5)
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##(filtered_state_means1, filtered_state_covariances1) = kf.filter(measurements1)
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##(smoothed_state_means1, smoothed_state_covariances1) = kf.smooth(measurements1)
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #states_pred3 = kf.em(measurements3).smooth(measurements3)[0]
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #print states_pred3
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##kf = KalmanFilter(transition_matrices = [[1, 1], [0, 1]], observation_matrices = [[0.1, 0.5], [-0.3, 0.0]])
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##kf = kf.em(measurements1, n_iter=5)
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##(filtered_state_means1, filtered_state_covariances1) = kf.filter(measurements1)
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt ##(smoothed_state_means1, smoothed_state_covariances1) = kf.smooth(measurements1)
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #for p in range (0,len(states_pred1)):
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #          fout5.write(str(states_pred1[p][0])+' '+str(states_pred2[p][0])+' '+str(states_pred3[p][0])+'\n') #toolpath waypoints

time11a = time.time()

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

if len(matching_pts)<10*downsample:
    pt_filter = 1
else:
    pt_filter = downsample

for i in range(0, len(matching_pts),pt_filter):#for i in range(0, len(matching_pts),pt_filter):
    findthis=np.zeros((1,2))                         #array of point to find
    findthis[0][0] = matching_pts[i][1] #array of point to find
    findthis[0][1] = matching_pts[i][2] #array of point to find
    findthis = np.array(findthis).astype('float32')  #array of point to find
    ret,results,neighbours,dist = knn.findNearest(findthis,1)
    fout.write(str(target_surface_pts[int(results[0])][0])+';'+str(matching_pts[i][1])+';'+str(matching_pts[i][2])+'\n') #toolpath waypoints
    #replaced with pt search #fout2.write(str(int(results[0]))+'\n')
#END ------ using k nearest neighbour, all data have to be put into array instead of list (this is more than 10 times faster than loop)




fout.close()  #fout = file('data/coupons/sampling_tool_path.txt','w')
#replaced idx matching loop with knearest #fout1.close() #fout1 = file('data/coupons/sampling_tool_path_idx.txt','w')
#replaced idx matching loop with knearest #fout2.close() #fout2 = file('data/coupons/sampling_tool_path_idx_normals.txt','w')
fout3.close() #fout3 = file('data/coupons/contour_number.txt','w')
#kalman to shift into c++ due to targeted pt normal search, no more normal.txt #fout5.close()
#idxdup.sort()


#plt.axis('equal'), plt.title("Final Points (Not Fitted)")
#plt.show()
time12 = time.time()

print "Timing p, readfile tf_bound:  ", format(time1a-time1 , '.4f'),"s"
print "Timing p, readfile pt_cloud:  ", format(time1b-time1a, '.4f'),"s"
print "Timing p, pt dist estimation: ", format(time2-time1b , '.4f'),"s"
print "Timing p, 2nd readfile:       ", format(time3-time2  , '.4f'),"s"
print "Timing p, Convert pts to img: ", format(time4-time3  , '.4f'),"s"
print "Timing p, waiting for key:    ", format(time4a-time4 , '.4f'),"s"
print "Timing p, mask for boundary:  ", format(time4b-time4a, '.4f'),"s"
print "Timing p, waiting for key:    ", format(time4c-time4b, '.4f'),"s"
print "Timing p, shift img to 0,0:   ", format(time5-time4c , '.4f'),"s"
print "Timing p, clock sort #off :   ", format(time6-time5  , '.4f'),"s"
print "Timing p, soortie #on :       ", format(time7-time6  , '.4f'),"s"
print "Timing p, show sortpts #off:  ", format(time8-time7  , '.4f'),"s"
print "Timing p, cal % range inc:    ", format(time9-time8  , '.4f'),"s"
print "Timing p, line fitting #off : ", format(time9a-time9 , '.4f'),"s"
print "Timing p, waiting for key:    ", format(time10-time9a, '.4f'),"s"
print "Timing p, undo rotation :     ", format(time11-time10, '.4f'),"s"
print "Timing p, pt match loop #off: ", format(time11a-time11, '.4f'),"s"
print "Timing p, pt match knearest:  ", format(time12-time11a, '.4f'),"s"
#plt.show()
time13 = time.time()
print "Timing p, waiting for key:    ", format(time13-time12, '.4f'),"s"
