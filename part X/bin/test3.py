import numpy as np
import math
import linecache

f1 = "./groundtruth1.txt"
f2 = open("./groundtruth2.txt")
f3 = "./associate.txt"
bias=0
bias_x=0
bias_y=0
bias_z=0
i=1
for line in f2:
    if line[0] == '#':
        continue
    data = line.split()
    x2 = float(data[2] ) 
    y2 = float(data[3] ) 
    z2 = float(data[4] ) 
    f3_num_line = linecache.getline(f3,int(data[1]))
    pose_time = float(f3_num_line[5:15])
    print(pose_time)
    f1_num_line = linecache.getline(f1,i)
    true_time = float(f1_num_line[5:15])
    while true_time<pose_time:
        data0 = f1_num_line.split()        
        x0 = float(data0[1] ) 
        y0 = float(data0[2] ) 
        z0 = float(data0[3] )
        true_time_0=true_time
        i=i+1
        f1_num_line = linecache.getline(f1,i)
        true_time = float(f1_num_line[5:15])
    print(true_time)
    data1 = f1_num_line.split()
    x1 = float(data1[1] ) 
    y1 = float(data1[2] ) 
    z1 = float(data1[3] )
    bis_x = (abs(x1-x2)*(true_time-pose_time)+abs(x0-x2)*(pose_time-true_time_0))/(true_time-true_time_0)
    bis_y = (abs(y1-y2)*(true_time-pose_time)+abs(y0-y2)*(pose_time-true_time_0))/(true_time-true_time_0)
    bis_z = (abs(z1-z2)*(true_time-pose_time)+abs(z0-z2)*(pose_time-true_time_0))/(true_time-true_time_0)
    bias_x = bias_x+bis_x
    bias_y = bias_y+bis_y
    bias_z = bias_z+bis_z
    bis = ((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2)) ** 0.5
    print(bis_x)
    print(bis_y)
    print(bis_z)
    print(bis)
    print("******************")
    bias=bias+bis
f4 = open("./groundtruth2.txt")
lines_num = len(f4.readlines())
print(bias_x/lines_num)
print(bias_y/lines_num)
print(bias_z/lines_num)
print(bias/lines_num)


    

