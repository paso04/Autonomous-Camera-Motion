#!/usr/bin/env python
import rospy
import numpy as np
from math import sqrt
import csv
import os
import message_filters
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import CompressedImage, JointState, Joy
import rosbag
import keras as tfk
import PyKDL
import dvrk
from tf_conversions import posemath
# from sqlite3 import Timestamp
# from tabnanny import verbose
# import tensorflow as tf
# from std_msgs.msg import Float64MultiArray, String

ECM = dvrk.ecm('ECM')

####################################################### CSV & BAG creation #######################################################

# path = '/home/npasini1/Desktop/Recordings/'
path = '/home/npasini1/Desktop/dVRK_UserStudy/pypredictor_results'
name = input('Please, specify your name: ')
csvname = name + ".csv"
bagname = name + ".bag"
csvFileName = os.path.join(path,csvname)
bagFileName = os.path.join(path,bagname)

bag = rosbag.Bag(bagFileName, 'w')
# with open(csvFileName, 'w', newline='') as f:     # con python3
f = open(csvFileName, 'w')
writer = csv.writer(f, delimiter = ',')

###################################################### USEFUL VARIABLES #########################################################

PSM1_offset = PyKDL.Vector(0.020,0.009,0.0)
PSM2_offset = PyKDL.Vector(0.020,0.009,0.0)
needle_offset = PyKDL.Vector(0.040,0.009,0.02)
stitches = []
coag_points = []
stitches_count = 0
coag_count = 0
gesture_length = 0
sx_gesture_length = 0
dx_gesture_length = 0
flag_new_stitch = False
flag_multiple_stitches = False
flag_coag_pressed = False

# Initialize the gesture as 'Not Holding needle'
# gesture = 'Not Holding needle'
# sx_gesture = 'Not Holding needle'
# dx_gesture = 'Not Holding needle'

####################################### DICTIONARY & EMPTY MATRIX CREATION & MODEL LOADING ######################################
 
# sx_model = tfk.models.load_model('/home/npasini1/Desktop/dVRK_UserStudy/model_checkpoints/batch256/Pranav_sx_5_256.h5')
# dx_model = tfk.models.load_model('/home/npasini1/Desktop/dVRK_UserStudy/model_checkpoints/batch256/Pranav_dx_5_256.h5')
# global X_RealTime
# global window_length
# window_length = 5
# n_features = 33
# X_RealTime = np.empty((window_length,n_features))

# sx_gesture_dictionary = {0: 'Not Holding needle',
#                       1: 'Suture Throw'
#                       }

# dx_gesture_dictionary = {0: 'Not Holding needle',
#                       1: 'Needle Positioning',
#                       2: 'Tissue Bite'
#                       }

####################################################### CALLBACK FUNCTION #######################################################

# Callback function: ogni messaggio viene chiamato simultaneamente con quest acallback cosi' i messaggi saranno tutti sincronizzati
# In ordine abbiamo: posa PSM1, posa PSM2, velocita' PSM1, velocita' PSM2, jaw1_sub, jaw2_sub

# At the first call, let's store the frame for both RCMs of PSM1 and 2

def coag_callback(msg):
    global flag_coag_pressed
    if msg.buttons[0] != 0:
            flag_coag_pressed = True

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

def ECM_cart_callback(msg):
    global ECM_cart
    global ECM_pose_goal
    ECM_pose_goal = posemath.fromMsg(msg.pose)
    ECM_cart = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    ECM_cart_sub.unregister()

def ECM_RCM_cart_callback(msg):
    global ECM_RCM_pose
    ECM_RCM_pose = posemath.fromMsg(msg.pose)

def sync_callback(msg1, msg2, msg3, msg4, msg5, msg6, img_msg):
    # print('callback')
    #Timestamp
    timestamp = img_msg.header.seq

    # PSM1 pose: we need to transform the pose so that it's referred to the cart instead of RCM --> so we can reach the same 3D point even with different insertion points on the patient)
    # f1 = PyKDL.Frame(PyKDL.Rotation.Quaternion(msg1.pose.orientation.x, msg1.pose.orientation.y, msg1.pose.orientation.z, msg1.pose.orientation.w) , PyKDL.Vector(msg1.pose.position.x, msg1.pose.position.y, msg1.pose.position.z))
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
    # f2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(msg2.pose.orientation.x, msg2.pose.orientation.y, msg2.pose.orientation.z, msg2.pose.orientation.w) , PyKDL.Vector(msg2.pose.position.x, msg2.pose.position.y, msg2.pose.position.z))
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
    # print(distance)

    bag.write("image", img_msg)

    ######################################################## FEATURES STORING #######################################################

    # Dimensione delle features: vettore di 1 x n_features --> classificazione 
    # features_to_concat = np.array([[psm1x,psm1y,psm1z,psm1ox,psm1oy,psm1oz,psm1ow,psm2x,psm2y,psm2z,psm2ox,psm2oy,psm2oz,psm2ow,
    #                  psm1lvx,psm1lvy,psm1lvz,psm1avx,psm1avy,psm1avz,psm2lvx,psm2lvy,psm2lvz,psm2avx,psm2avy,psm2avz,
    #                  jaw1position,jaw1velocity,jaw1effort,jaw2position,jaw2velocity,jaw2effort,distance]])
    # global X_RealTime
    # global window_length
    # X_RealTime = np.concatenate((X_RealTime[1:,:],features_to_concat), axis=0)

    # X_position_independent = X_RealTime.astype(float)
    # X_position_independent[:,[0,1,2,7,8,9]] = X_RealTime[:,[0,1,2,7,8,9]] - np.tile(X_RealTime[0,[0,1,2,7,8,9]],(window_length,1))
    # X_RealTime[:,[0,1,2,7,8,9]] = X_RealTime[:,[0,1,2,7,8,9]] - np.tile(X_RealTime[0,[0,1,2,7,8,9]],(window_length,1))
    # print(X_RealTime.shape)

    ###################################################### MODEL CLASSIFICATION #####################################################
    # with graph.as_default():

    # Initialize the gesture_pre as the last recognized gesture
    # global sx_gesture, sx_gesture_length, dx_gesture, dx_gesture_length
    # sx_gesture_pre = sx_gesture
    # dx_gesture_pre = dx_gesture

    # sx_prediction = sx_model.predict(np.expand_dims(X_position_independent[:,[7,8,9,10,11,12,13,20,21,22,23,24,25,29,30,31,32]], 0), verbose=0) # X_realtime dev essere (1,window_length,33)
    # dx_prediction = dx_model.predict(np.expand_dims(X_position_independent[:,[0,1,2,3,4,5,6,14,15,16,17,18,19,26,27,28,32]], 0), verbose=0) # X_realtime dev essere (1,window_length,33)
    # sx_gesture_int = np.argmax(sx_prediction, axis=1)
    # dx_gesture_int = np.argmax(dx_prediction, axis=1)
    # sx_gesture = sx_gesture_dictionary[sx_gesture_int[0]]
    # dx_gesture = dx_gesture_dictionary[dx_gesture_int[0]]
    # if sx_gesture_pre == sx_gesture:
    #     sx_gesture_length += 1
    # else:
    #     sx_gesture_length = 1
    # if dx_gesture_pre == dx_gesture:
    #     dx_gesture_length += 1
    # else:
    #     dx_gesture_length = 1

    # # print(gesture_length)
    # print("SX  -  DX: ", sx_gesture, dx_gesture)
    # gesture = 'GESTURE'

    ###################################################### COMBINING PREDICTIONS #####################################################
    # global gesture, gesture_pre
    # gesture_pre = gesture
    # # if sx_gesture =="Suture Throw" and dx_gesture != "Not Holding needle":
    # #     gesture = "Passing needle"
    # # elif sx_gesture =="Suture Throw" and dx_gesture == "Not Holding needle":
    # #     gesture = "Suture Throw"
    # if sx_gesture =="Suture Throw":
    #     gesture = "Suture Throw"
    # elif dx_gesture == "Needle Positioning":
    #     gesture = "Needle Positioning"
    # elif dx_gesture == "Tissue Bite":
    #     gesture = "Tissue Bite"
    # else:
    #     gesture = "Not Holding needle"

    # print(gesture)
    ####################################################### WRITING TO CSV FILE #####################################################

    writer.writerow([timestamp,psm1x,psm1y,psm1z,psm1ox,psm1oy,psm1oz,psm1ow,psm2x,psm2y,psm2z,psm2ox,psm2oy,psm2oz,psm2ow,
                     psm1lvx,psm1lvy,psm1lvz,psm1avx,psm1avy,psm1avz,psm2lvx,psm2lvy,psm2lvz,psm2avx,psm2avy,psm2avz,
                     jaw1position,jaw1velocity,jaw1effort,jaw2position,jaw2velocity,jaw2effort,distance])

    ################################################## CHECKING IF THERE'S A STITCH #################################################
    # In this section we check if the surgeon is performing a tissue bite because there's a new stitch to be added to the count.
    # We have 3 conditions to satisfy: the gesture must be a Tissue Bite, the gesture_length must be greater than half a second (15
    # time stamps) so that we are able to discard most of the false positive that the model classifies, and the flag_new_stitch must
    # be True, meaning that the surgeon has completed the previous stitch (it's possible that a surgeon wants to reposition the needle
    # even after the Tissue Bite gesture has already started. In that case the model will se in order: TB - NP - TB as gestures. Only
    # the first TB will be consider as a new stitch).flag_multiple_stitches
    # At every iteration the count of the stitches increases and the coordinate of the new stitch is stored
    
    # global stitches_count, stitches, flag_new_stitch

    # if gesture_pre == "Tissue Bite" and gesture == "Suture Throw": # and flag_new_stitch == True:
    #     print('STITCH')
    #     # stitch = psm1_to_cart.p + needle_offset# + psm2_to_cart.p
    #     # stitches.append(psm1_to_cart.p + needle_offset)
    #     stitches.append((psm1_to_cart.p + psm2_to_cart.p)/2 + PSM1_offset)
    #     stitches_count += 1
    #     # flag_new_stitch = False

    ##################################### ASKING THE USER TO REGISTER STARTING AND ENDING POINTS ####################################
    # global coag_count, flag_coag_pressed
    # if flag_coag_pressed == True:
    #     print('NEW POINT')
    #     coag_points.append(psm1_to_cart.p + PSM1_offset)
    #     coag_count += 1
    #     flag_coag_pressed = False

    # global scene_center, flag_multiple_stitches
    # if coag_count < 2:
    #     # print(scene_center)
    #     if gesture == "Needle Positioning" or gesture == "Suture Throw":
    scene_center = (psm1_to_cart.p + psm2_to_cart.p)/2 + PSM1_offset
            # scene_center = psm1_to_cart.p + PSM1_offset
        # elif gesture == "Suture Throw":
        #     scene_center = (psm1_to_cart.p + psm2_to_cart.p)/2 + PSM2_offset
            # flag_new_stitch = True

    # else:
        # print('ELSE')
        # if gesture == "Needle Positioning" or gesture == "Suture Throw":
        #     AP = (psm1_to_cart.p + psm2_to_cart.p)/2 - coag_points[-2]    #vector
        #     # AP = psm1_to_cart.p + PSM1_offset - coag_points[-2]
        #     scene_center = coag_points[-2] + (PyKDL.dot(AP,(coag_points[-1] - coag_points[-2]))/PyKDL.dot((coag_points[-1] - coag_points[-2]),(coag_points[-1] - coag_points[-2])))*(coag_points[-1] - coag_points[-2])   # projection
        #     flag_multiple_stitches = True

    ##################################################### DEFINING CAMERA CENTER ####################################################
    
    # ATTENTION: the ECM object has got the RF wrt the cart, se every command should be given using /SUJ * /local coordinates
    # In this case we need to check whether more than 2 stitches has been performed or not: in case the camera will move only in NP
    # and directly to the next stitch, based on the distance and direction between the 2 previous stitches

    # global scene_center, flag_multiple_stitches
    # if stitches_count < 999:
    #     # print(scene_center)
    #     if gesture == "Needle Positioning":
    #         scene_center = (psm1_to_cart.p + psm2_to_cart.p)/2 + PSM1_offset
    #     elif gesture == "Suture Throw":
    #         scene_center = (psm1_to_cart.p + psm2_to_cart.p)/2 + PSM2_offset
    #         # flag_new_stitch = True

    # else:
    #     # if gesture == "Suture Throw":
    #     #     flag_new_stitch = True
    #     # AP = psm1_to_cart.p - stitches[-1]    #vector
    #     # scene_center = stitches[-1] + (PyKDL.dot(AP,(stitches[-1] - stitches[-2]))/PyKDL.dot((stitches[-1] - stitches[-2]),(stitches[-1] - stitches[-2])))*(stitches[-1] - stitches[-2])    # projection
    #     scene_center = stitches[-1] + (stitches[-1]-stitches[-2])
    #     flag_multiple_stitches = True

        # MODIFICA IN MODO CHE LA CAMERA SI MUOVA PIU' LENTAMENTE E SOLO QUANDO STO FACENDO SUTURE THROW NON QUANDO HO ANCORA TISSUE BITE
    
    ########################################################### MOVING ECM ##########################################################

    try:
        # algorithm: cross product for ECM pose
        ECM_orientation_z = (scene_center - ECM_RCM_pose.p)/(scene_center - ECM_RCM_pose.p).Norm()
        ECM_orientation_x = PyKDL.Vector(1.0,0.0,0.0)
        ECM_orientation_y = ECM_orientation_z*ECM_orientation_x
        ECM_orientation_x = ECM_orientation_y*ECM_orientation_z
        ECM_position = scene_center - (0.10 + 0.35*distance)*((scene_center - ECM_RCM_pose.p)/(scene_center - ECM_RCM_pose.p).Norm())
        ECM_pose_goal.M.UnitX(ECM_orientation_x)
        ECM_pose_goal.M.UnitY(ECM_orientation_y)
        ECM_pose_goal.M.UnitZ(ECM_orientation_z)
        ECM_pose_goal.p = ECM_position
        ECM.move_cp(ECM_pose_goal)

        # # with this if cycle we impose the camera to move only when we start the needle positioning phase, after having completed the second stitch
        # if flag_multiple_stitches == True and gesture == "Needle Positioning":
        #     ECM.move_cp(ECM_pose_goal)
        # elif flag_multiple_stitches == False:
        #     ECM.move_cp(ECM_pose_goal)
    except:
        # print("Ops! It seems I cannot go there, try again :)")
        print('Waiting for next move :)')

########################################################## SUBSCRIBERS ##########################################################

f1_cart_sub = rospy.Subscriber("/SUJ/PSM1/local/measured_cp", PoseStamped, psm1_cart_callback)
f2_cart_sub = rospy.Subscriber("/SUJ/PSM2/local/measured_cp", PoseStamped, psm2_cart_callback)
ECM_cart_sub = rospy.Subscriber("/ECM/measured_cp", PoseStamped, ECM_cart_callback)
ECM_RCM_cart_sub = rospy.Subscriber("/SUJ/ECM/local/measured_cp", PoseStamped, ECM_RCM_cart_callback)
coag_sub = rospy.Subscriber("/footpedals/coag", Joy, coag_callback)

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

    writer.writerow(['SX Gesture','DX Gesture','Timestamps','PSM1x','PSM1y','PSM1z','PSM1_quat_x','PSM1_quat_y','PSM1_quat_z','PSM1_quat_w','PSM2x','PSM2y','PSM2z',
                    'PSM2_quat_x','PSM2_quat_y','PSM2_quat_z','PSM2_quat_w','PSM1_linearv_x','PSM1_linearv_y','PSM1_linearv_z',
                    'PSM1_angularv_x','PSM1_angularv_y','PSM1_angularv_z','PSM2_linearv_x','PSM2_linearv_y','PSM2_linearv_z',
                    'PSM2_angularv_x','PSM2_angularv_y','PSM2_angularv_z','jaw1_position','jaw1_velocity','jaw1_effort',
                    'jaw2_position','jaw2_velocity','jaw2_effort','Distance'])

    ###################################################### TIME SYNCHRONIZATION #####################################################
    rospy.sleep(1)

    ts = message_filters.ApproximateTimeSynchronizer([PSM1position_sub, PSM2position_sub, PSM1velocity_sub, PSM2velocity_sub, jaw1_sub, jaw2_sub, video_frame_sub], 1, 0.1, allow_headerless=True)
    # ts = message_filters.ApproximateTimeSynchronizer([PSM1position_sub, PSM2position_sub, PSM1velocity_sub, PSM2velocity_sub, jaw1_sub, jaw2_sub], 1, 0.1, allow_headerless=True)

    ts.registerCallback(sync_callback)

    rospy.spin()

if __name__ == '__main__':
    main()