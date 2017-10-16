
# coding: utf-8

# In[1]:

import scipy.io as sio
import numpy as np
# tracks = sio.loadmat('GTSAM_tracks.mat')
# tracks = tracks['results']

'''maintain circular queue for GTSAM oordinates to publish'''
from collections import deque
# ringbuff = deque(tracks, len(tracks))
# a = ringbuff.pop()


# In[2]:

#for 3d coordinated use this code instead of above code
#read constraints which will be our factor graphs
# file1 = open("/home/iiith/abhishek/Map-merging/build/data/traj1_traj2_with_inliers.g2o",'r')
file1 = open("/home/iiith/codes/general/3d_1on3_incremental_isam.g2o",'r')
# file1 = open("/home/iiith/abhishek/MultiRobot-2.0/3d/build/isamnfg.g2o",'r')
factors = []
for i in file1:
    if len(i.split())<=9:
        values1 = i.split()[1:9]
#         import pdb;pdbD.set_trace()
        prev_frame=int(values1[0])
#         next_frame=int(values1[1])
        x=float(values1[1])
        y=float(values1[2])
        z =float(values1[3])
        qx = float(values1[4])
        qy = float(values1[5])
        qz = float(values1[6])
        qw = float(values1[7])
        factors.append([x,y,z,qx,qy,qz,qw])
file1.close()
tracks=np.array(factors)
ringbuff = deque(tracks, len(tracks))


# In[3]:

#if we have loop 3 first in g2o file and then loop 1 then use this code to invert

track_3 = tracks[0:1198]
track_1 = tracks[1198:]
track_inverted = np.zeros((1926,7))
track_inverted[0:728] = track_1[0:728]
track_inverted[728:] = track_3[0:]

track_inverted = track_inverted[1:]
#double ended queue is filled again
tracks = track_inverted
ringbuff = deque(tracks, len(tracks))


# In[4]:

#!/usr/bin/env python  
import roslib
import math
from nav_msgs.msg import Odometry
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import message_filters
from sensor_msgs.msg import sensor_msgs


pub = rospy.Publisher('/GTSAM/odometry', Odometry, queue_size=10)

def publish_tf(msg1,msg2):
    global ringbuff
    global pub
    pose= ringbuff.popleft()
    br = tf.TransformBroadcaster()
#     x,y,z,roll,pitch,yaw = (pose[0],0,pose[1],0,pose[2],0)
    Tr = np.array([pose[0],pose[1],0])
    vRg = np.array([[1,0,0],[0,0,-1],[0,1,0]])
    Tr = np.matmul(vRg,Tr)
    br.sendTransform((pose[0],pose[1],pose[2]),
                     (pose[3], pose[4],pose[5],pose[6]),
                     msg1.header.stamp,
                     '/zed_tracked_frame',
                     "odom")
    pose_3d = Odometry()
    pose_3d.header.stamp = msg1.header.stamp
    pose_3d.header.frame_id='odom'
    pose_3d.child_frame_id='zed_tracked_frame'
#     angles = tf.transformations.quaternion_from_euler(pose[3], pose[4],pose[5])
    pose_3d.pose.pose = Pose(Point(pose[0],pose[1],pose[2]),Quaternion(pose[3], pose[4],pose[5],pose[6]))
    pub.publish(pose_3d)
    

if __name__ == '__main__':
    rospy.init_node('GTSAM_track')
#     rospy.Subscriber('/stereo_odometer/odometry',
#                      Odometry,
#                      publish_tf)
    I1 = message_filters.Subscriber("/camera/left/image_rect_color", sensor_msgs.msg.Image)
    I2 =  message_filters.Subscriber("/camera/right/image_rect_color", sensor_msgs.msg.Image)
    ts = message_filters.TimeSynchronizer([I1, I2], 10)
    ts.registerCallback(publish_tf)
#     rospy.publish
    rospy.spin()
        


# In[ ]:



