#!/usr/bin/env python
from sqlite3 import Timestamp
from tabnanny import verbose
import rospy
import numpy as np
from math import sqrt
import csv
import os
import message_filters
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import CompressedImage, JointState
import rosbag
from tf_conversions import posemath
import PyKDL

####################################################### CSV & BAG creation #######################################################

# Problema con python2.7, non si puo' usare input se non con variabili dichiarate in precedenza (non puoi chiamare il file come ti pare)
path = '/home/npasini1/Desktop/dVRK_UserStudy/dVRK_pyrecorder/'
name = input('Please, specify your name: ')
csvname = name + ".csv"
bagname = name + ".bag"
csvFileName = os.path.join(path,csvname)
bagFileName = os.path.join(path,bagname)

bag = rosbag.Bag(bagFileName, 'w')
# with open(csvFileName, 'w', newline='') as f:     # con python3
f = open(csvFileName, 'w')
writer = csv.writer(f, delimiter = ',')

####################################################### CALLBACK FUNCTION #######################################################

# Callback function: ogni messaggio viene chiamato simultaneamente con quest acallback cosi' i messaggi saranno tutti sincronizzati
# In ordine abbiamo: posa PSM1, posa PSM2, velocita' PSM1, velocita' PSM2, jaw1_sub, jaw2_sub

# At the first call, let's store the frame for both RCMs of PSM1 and 2

def psm1_cart_callback(msg):

    global f1_cart_frame
    # f1_cart_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w) , PyKDL.Vector(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
    f1_cart_frame = posemath.fromMsg(msg.pose)
    print('Succesfully stored PSM1 to Cart transform!')
    f1_cart_sub.unregister()

def psm2_cart_callback(msg):

    global f2_cart_frame
    # f2_cart_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w) , PyKDL.Vector(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
    f2_cart_frame = posemath.fromMsg(msg.pose)
    print('Succesfully stored PSM2 to Cart transform!')
    f2_cart_sub.unregister()

def sync_callback(msg1, msg2, msg3, msg4, msg5, msg6, img_msg, msg8):
    # print('callback')
    ecmx = msg8.pose.position.x
    ecmy = msg8.pose.position.y
    ecmz = msg8.pose.position.z

    #Timestamp
    timestamp = img_msg.header.seq

    # PSM1 pose: we need to transform the pose so that it's referred to the cart instead of RCM --> so we can reach the same 3D point even with different insertion points on the patient)
    f1 = posemath.fromMsg(msg1.pose)
    psm1_to_cart = f1_cart_frame * f1

    psm1x = psm1_to_cart.p[0]
    psm1y = psm1_to_cart.p[1]
    psm1z = psm1_to_cart.p[2]

    quat1 = psm1_to_cart.M.GetQuaternion()

    psm1ox = quat1[0]
    psm1oy = quat1[1]
    psm1oz = quat1[2]
    psm1ow = quat1[3]

    # PSM2 pose
    f2 = posemath.fromMsg(msg2.pose)
    psm2_to_cart = f2_cart_frame * f2

    psm2x = psm2_to_cart.p[0]
    psm2y = psm2_to_cart.p[1]
    psm2z = psm2_to_cart.p[2]

    quat2 = psm2_to_cart.M.GetQuaternion()

    psm2ox = quat2[0]
    psm2oy = quat2[1]
    psm2oz = quat2[2]
    psm2ow = quat2[3]

    # PSM1 linear and angular velocity
    psm1lvx = msg3.twist.linear.x
    psm1lvy = msg3.twist.linear.y
    psm1lvz = msg3.twist.linear.z

    psm1avx = msg3.twist.angular.x
    psm1avy = msg3.twist.angular.y
    psm1avz = msg3.twist.angular.z

    # PSM2 linear and angular velocity
    psm2lvx = msg4.twist.linear.x
    psm2lvy = msg4.twist.linear.y
    psm2lvz = msg4.twist.linear.z

    psm2avx = msg4.twist.angular.x
    psm2avy = msg4.twist.angular.y
    psm2avz = msg4.twist.angular.z

    # Jaw position, velocity and effort
    jaw1position = msg5.position[0]
    jaw1velocity = msg5.velocity[0]
    jaw1effort = msg5.effort[0]

    jaw2position = msg6.position[0]
    jaw2velocity = msg6.velocity[0]
    jaw2effort = msg6.effort[0]

    # Tooltip distance: I need the position of the tools wrt the base of the robot, NOT the RCM
    distance = sqrt((psm1x-psm2x)**2 + (psm1y-psm2y)**2 + (psm1z-psm2z)**2)
    # print(distance)ECM_RCM_pose

    bag.write("image", img_msg)

    ####################################################### WRITING TO CSV FILE #####################################################

    gesture = 'GESTURE'

    writer.writerow([gesture,timestamp,psm1x,psm1y,psm1z,psm1ox,psm1oy,psm1oz,psm1ow,psm2x,psm2y,psm2z,psm2ox,psm2oy,psm2oz,psm2ow,
                     psm1lvx,psm1lvy,psm1lvz,psm1avx,psm1avy,psm1avz,psm2lvx,psm2lvy,psm2lvz,psm2avx,psm2avy,psm2avz,
                     jaw1position,jaw1velocity,jaw1effort,jaw2position,jaw2velocity,jaw2effort,distance,ecmx,ecmy,ecmz])

########################################################## SUBSCRIBERS ##########################################################

f1_cart_sub = rospy.Subscriber('/SUJ/PSM1/local/measured_cp', PoseStamped, psm1_cart_callback)
f2_cart_sub = rospy.Subscriber('/SUJ/PSM2/local/measured_cp', PoseStamped, psm2_cart_callback)

PSM1position_sub = message_filters.Subscriber('/PSM1/local/measured_cp', PoseStamped)
PSM2position_sub = message_filters.Subscriber("/PSM2/local/measured_cp", PoseStamped)
ECMposition_sub = message_filters.Subscriber("/ECM/measured_cp", PoseStamped)
PSM1velocity_sub = message_filters.Subscriber("/PSM1/local/measured_cv", TwistStamped)
PSM2velocity_sub = message_filters.Subscriber("/PSM2/local/measured_cv", TwistStamped)
jaw1_sub = message_filters.Subscriber("/PSM1/jaw/measured_js", JointState)
jaw2_sub = message_filters.Subscriber("/PSM2/jaw/measured_js", JointState)
video_frame_sub = message_filters.Subscriber("/jhu_daVinci/left/decklink/jhu_daVinci_left/image_raw/compressed", CompressedImage)

def main():

    ###################################################### NODE INIZIALIZATION ######################################################

    rospy.init_node('pyrecorder', anonymous=True)

    writer.writerow(['Predicted Gesture','Timestamps','PSM1x','PSM1y','PSM1z','PSM1_quat_x','PSM1_quat_y','PSM1_quat_z','PSM1_quat_w','PSM2x','PSM2y','PSM2z',
                    'PSM2_quat_x','PSM2_quat_y','PSM2_quat_z','PSM2_quat_w','PSM1_linearv_x','PSM1_linearv_y','PSM1_linearv_z',
                    'PSM1_angularv_x','PSM1_angularv_y','PSM1_angularv_z','PSM2_linearv_x','PSM2_linearv_y','PSM2_linearv_z',
                    'PSM2_angularv_x','PSM2_angularv_y','PSM2_angularv_z','jaw1_position','jaw1_velocity','jaw1_effort',
                    'jaw2_position','jaw2_velocity','jaw2_effort','Distance','ECMx','ECMy','ECMz'])

    ###################################################### TIME SYNCHRONIZATION #####################################################

    rospy.sleep(1)

    ts = message_filters.ApproximateTimeSynchronizer([PSM1position_sub, PSM2position_sub, PSM1velocity_sub, PSM2velocity_sub, jaw1_sub, jaw2_sub, video_frame_sub, ECMposition_sub], 1, 0.1, allow_headerless=True)
    ts.registerCallback(sync_callback)

    rospy.spin()

if __name__ == '__main__':
    main()