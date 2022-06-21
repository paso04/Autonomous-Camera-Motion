#!/usr/bin/env python
from sqlite3 import Timestamp
from tabnanny import verbose
import rospy
import numpy as np
from math import sqrt
import csv
import os
import message_filters
from geometry_msgs.msg import TransformStamped, TwistStamped, PoseStamped
from sensor_msgs.msg import Image, CompressedImage, JointState
from std_msgs.msg import Float64MultiArray
import rosbag
import tensorflow as tf
import keras as tfk
import PyKDL
import dvrk

ecm = dvrk.ecm('ECM')

####################################################### CSV & BAG creation #######################################################

path = '/home/npasini1/Desktop/Recordings/'
name = input('Please, specify your ID (e.g. id1): ')
csvname = name + ".csv"
bagname = name + ".bag"
csvFileName = os.path.join(path,csvname)
bagFileName = os.path.join(path,bagname)

bag = rosbag.Bag(bagFileName, 'w')
# with open(csvFileName, 'w', newline='') as f:     # con python3
f = open(csvFileName, 'w')
writer = csv.writer(f, delimiter = ',')

####################################### DICTIONARY & EMPTY MATRIX CREATION & MODEL LOADING ######################################
 
model = tfk.models.load_model('/home/npasini1/Desktop/model_checkpoints/magnet1_allepochs.h5')
global X_RealTime
global window_length
window_length = 30
X_RealTime = np.empty((window_length,33))
gesture_dictionary = {0: 'Not Holding needle',
                      1: 'Needle Positioning',
                      2: 'Suture Throw',
                      3: 'Tissue Bite'
                      }

####################################################### CALLBACK FUNCTION #######################################################

# Callback function: ogni messaggio viene chiamato simultaneamente con quest acallback cosi' i messaggi saranno tutti sincronizzati
# In ordine abbiamo: posa PSM1, posa PSM2, velocita' PSM1, velocita' PSM2, jaw1_sub, jaw2_sub

# At the first call, let's store the frame for both RCMs of PSM1 and 2

def psm1_cart_callback(msg7):

    global f1_cart_frame
    f1_cart_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(msg7.pose.orientation.x, msg7.pose.orientation.y, msg7.pose.orientation.z, msg7.pose.orientation.w) , PyKDL.Vector(msg7.pose.position.x, msg7.pose.position.y, msg7.pose.position.z))
    print('Succesfully stored PSM1 to Cart transform!')
    f1_cart_sub.unregister()

def psm2_cart_callback(msg8):

    global f2_cart_frame
    f2_cart_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(msg8.pose.orientation.x, msg8.pose.orientation.y, msg8.pose.orientation.z, msg8.pose.orientation.w) , PyKDL.Vector(msg8.pose.position.x, msg8.pose.position.y, msg8.pose.position.z))
    print('Succesfully stored PSM2 to Cart transform!')
    f2_cart_sub.unregister()

def ecm_cart_callback(msg9):
    global ecm_cart
    ecm_cart = [msg9.pose.position.x, msg9.pose.position.y, msg9.pose.position.z]
    ecm_cart_sub.unregister()

def sync_callback(msg1, msg2, msg3, msg4, msg5, msg6, img_msg):
    # print('callback')
    #Timestamp
    timestamp = img_msg.header.seq

    # PSM1 pose: we need to transform the pose so that it's referred to the cart instead of RCM --> so we can reach the same 3D point even with different insertion points on the patient)
    f1 = PyKDL.Frame(PyKDL.Rotation.Quaternion(msg1.pose.orientation.x, msg1.pose.orientation.y, msg1.pose.orientation.z, msg1.pose.orientation.w) , PyKDL.Vector(msg1.pose.position.x, msg1.pose.position.y, msg1.pose.position.z))
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
    f2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(msg2.pose.orientation.x, msg2.pose.orientation.y, msg2.pose.orientation.z, msg2.pose.orientation.w) , PyKDL.Vector(msg2.pose.position.x, msg2.pose.position.y, msg2.pose.position.z))
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
    # print(distance)

    bag.write("image", img_msg)

    ######################################################## FEATURES STORING #######################################################

    # Dimensione delle features: vettore di 1 x n_features --> classificazione 
    features_to_concat = np.array([[psm1x,psm1y,psm1z,psm1ox,psm1oy,psm1oz,psm1ow,psm2x,psm2y,psm2z,psm2ox,psm2oy,psm2oz,psm2ow,
                     psm1lvx,psm1lvy,psm1lvz,psm1avx,psm1avy,psm1avz,psm2lvx,psm2lvy,psm2lvz,psm2avx,psm2avy,psm2avz,
                     jaw1position,jaw1velocity,jaw1effort,jaw2position,jaw2velocity,jaw2effort,distance]])
    global X_RealTime
    global window_length
    X_RealTime = np.concatenate((X_RealTime[1:,:],features_to_concat), axis=0)

    X_position_independent = X_RealTime.astype(float)
    X_position_independent[:,[0,1,2,7,8,9]] = X_RealTime[:,[0,1,2,7,8,9]] - np.tile(X_RealTime[0,[0,1,2,7,8,9]],(window_length,1))

    # X_RealTime[:,[0,1,2,7,8,9]] = X_RealTime[:,[0,1,2,7,8,9]] - np.tile(X_RealTime[0,[0,1,2,7,8,9]],(window_length,1))

    # print(X_RealTime.shape)

    ###################################################### MODEL CLASSIFICATION #####################################################
    # with graph.as_default():

    # prediction = model.predict(np.empty((1,30,33)))
    prediction = model.predict(np.expand_dims(X_position_independent, 0), verbose=0) # X_realtime dev essere (1,5,33) / (1,30,33)

    gesture_int = np.argmax(prediction, axis=1)
    gesture = gesture_dictionary[gesture_int[0]]

    print("Gesture performed: ", gesture)
    # gesture = 'GESTURE'
    ####################################################### WRITING TO CSV FILE #####################################################

    writer.writerow([gesture,timestamp,psm1x,psm1y,psm1z,psm1ox,psm1oy,psm1oz,psm1ow,psm2x,psm2y,psm2z,psm2ox,psm2oy,psm2oz,psm2ow,
                     psm1lvx,psm1lvy,psm1lvz,psm1avx,psm1avy,psm1avz,psm2lvx,psm2lvy,psm2lvz,psm2avx,psm2avy,psm2avz,
                     jaw1position,jaw1velocity,jaw1effort,jaw2position,jaw2velocity,jaw2effort,distance])

    ########################################################### MOVING ECM ##########################################################
'''
    # The ecm object has got the RF wrt the cart, se every command should be given using /SUJ * /local coordinates
    try:
        goal = ecm.setpoint_cp()
        if gesture == "Needle Positioning":
            goal.p = 0.7 * ecm_cart + 0.3 * [psm1x, psm1y, psm1z]
        elif gesture == "Suture Throw":
            goal.p = 0.8 * ecm_cart + 0.2 * [psm2x, psm2y, psm2z]
    
        ecm.move_cp(goal)
    except:
        print("Ops! It seems I cannot go there, try again :)")
'''

########################################################## SUBSCRIBERS ##########################################################

f1_cart_sub = rospy.Subscriber("/SUJ/PSM1/local/measured_cp", PoseStamped, psm1_cart_callback)
f2_cart_sub = rospy.Subscriber("/SUJ/PSM2/local/measured_cp", PoseStamped, psm2_cart_callback)
ecm_cart_sub = rospy.Subscriber("/ECM/measured_cp", PoseStamped)

PSM1position_sub = message_filters.Subscriber("/PSM1/local/measured_cp", PoseStamped)
PSM2position_sub = message_filters.Subscriber("/PSM2/local/measured_cp", PoseStamped)
PSM1velocity_sub = message_filters.Subscriber("/PSM1/local/measured_cv", TwistStamped)
PSM2velocity_sub = message_filters.Subscriber("/PSM2/local/measured_cv", TwistStamped)
jaw1_sub = message_filters.Subscriber("/PSM1/jaw/measured_js", JointState)
jaw2_sub = message_filters.Subscriber("/PSM2/jaw/measured_js", JointState)
video_frame_sub = message_filters.Subscriber("/jhu_daVinci/right/decklink/jhu_daVinci_right/image_raw/compressed", CompressedImage)


def main():

    ###################################################### NODE INIZIALIZATION ######################################################

    # rospy.init_node('pyrecorder', anonymous=True)

    writer.writerow(['Predicted Gesture','Timestamps','PSM1x','PSM1y','PSM1z','PSM1_quat_x','PSM1_quat_y','PSM1_quat_z','PSM1_quat_w','PSM2x','PSM2y','PSM2z',
                    'PSM2_quat_x','PSM2_quat_y','PSM2_quat_z','PSM2_quat_w','PSM1_linearv_x','PSM1_linearv_y','PSM1_linearv_z',
                    'PSM1_angularv_x','PSM1_angularv_y','PSM1_angularv_z','PSM2_linearv_x','PSM2_linearv_y','PSM2_linearv_z',
                    'PSM2_angularv_x','PSM2_angularv_y','PSM2_angularv_z','jaw1_position','jaw1_velocity','jaw1_effort',
                    'jaw2_position','jaw2_velocity','jaw2_effort','Distance'])

    ###################################################### TIME SYNCHRONIZATION #####################################################
    rospy.sleep(1)

    ts = message_filters.ApproximateTimeSynchronizer([PSM1position_sub, PSM2position_sub, PSM1velocity_sub, PSM2velocity_sub, jaw1_sub, jaw2_sub, video_frame_sub], 1, 0.1, allow_headerless=True)
    ts.registerCallback(sync_callback)

    rospy.spin()

if __name__ == '__main__':
    main()