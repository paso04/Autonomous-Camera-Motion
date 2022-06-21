#!/usr/bin/env python
from sqlite3 import Timestamp
import rospy
import numpy as np
from math import sqrt
import csv
import os
import message_filters
from geometry_msgs.msg import TransformStamped, TwistStamped
from sensor_msgs.msg import Image, CompressedImage, JointState
import rosbag
import tensorflow as tf
import keras

####################################################### CSV & BAG creation #######################################################


# Problema con python2.7, non si puo' usare input se non con variabili dichiarate in precedenza (non puoi chiamare il file come ti pare)
id1 = "Nicolo"
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

model = keras.models.load_model('/home/npasini1/nicolo_ws/src/pyrecorder/src/mymodel.h5')
global X_RealTime
X_RealTime = np.empty((5,33))
gesture_dictionary = {0: 'BA',
                      1: 'NP',
                      2: 'ST',
                      3: 'TB'
                      }

graph = tf.get_default_graph()

####################################################### CALLBACK FUNCTION #######################################################

# Callback function: ogni messaggio viene chiamato simultaneamente con quest acallback cosi' i messaggi saranno tutti sincronizzati
# In ordine abbiamo: posa PSM1, posa PSM2, velocita' PSM1, velocita' PSM2, jaw1, jaw2

def sync_callback(msg1, msg2, msg3, msg4, msg5, msg6, img_msg):

    #Timestamp
    timestamp = img_msg.header.seq

    # PSM1 pose
    psm1x = msg1.transform.translation.x
    psm1y = msg1.transform.translation.y
    psm1z = msg1.transform.translation.z

    psm1ox = msg1.transform.rotation.x
    psm1oy = msg1.transform.rotation.y
    psm1oz = msg1.transform.rotation.z
    psm1ow = msg1.transform.rotation.w

    # PSM2 pose
    psm2x = msg2.transform.translation.x
    psm2y = msg2.transform.translation.y
    psm2z = msg2.transform.translation.z

    psm2ox = msg2.transform.rotation.x
    psm2oy = msg2.transform.rotation.y
    psm2oz = msg2.transform.rotation.z
    psm2ow = msg2.transform.rotation.w

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

    # Tooltip distance
    distance = sqrt((psm1x-psm2x)**2 + (psm1y-psm2y)**2 + (psm1z-psm2z)**2)

    bag.write("image", img_msg)

    

    ######################################################## FEATURES STORING #######################################################
    # Dimensione delle features: vettore di 1 x n_features --> classificazione 
    features_to_concat = np.array([[psm1x,psm1y,psm1z,psm1ox,psm1oy,psm1oz,psm1ow,psm2x,psm2y,psm2z,psm2ox,psm2oy,psm2oz,psm2ow,
                     psm1lvx,psm1lvy,psm1lvz,psm1avx,psm1avy,psm1avz,psm2lvx,psm2lvy,psm2lvz,psm2avx,psm2avy,psm2avz,
                     jaw1position,jaw1velocity,jaw1effort,jaw2position,jaw2velocity,jaw2effort,distance]])
    global X_RealTime
    X_RealTime = np.concatenate((X_RealTime[1:,:],features_to_concat), axis=0)

    ###################################################### MODEL CLASSIFICATION #####################################################
    with graph.as_default():
        prediction = model.predict(np.expand_dims(X_RealTime,axis=0)) # X_realtime dev essere (1,5,33)
    
    gesture_int = np.argmax(prediction, axis=1)
    gesture = gesture_dictionary[gesture_int[0]]

    print("Gesture performed: ", gesture)

    ####################################################### WRITING TO CSV FILE #####################################################

    writer.writerow([gesture,timestamp,psm1x,psm1y,psm1z,psm1ox,psm1oy,psm1oz,psm1ow,psm2x,psm2y,psm2z,psm2ox,psm2oy,psm2oz,psm2ow,
                     psm1lvx,psm1lvy,psm1lvz,psm1avx,psm1avy,psm1avz,psm2lvx,psm2lvy,psm2lvz,psm2avx,psm2avy,psm2avz,
                     jaw1position,jaw1velocity,jaw1effort,jaw2position,jaw2velocity,jaw2effort,distance])


########################################################## SUBSCRIBERS ##########################################################

PSM1position = message_filters.Subscriber("/PSM1/local/measured_cp", TransformStamped)
PSM2position = message_filters.Subscriber("/PSM2/local/measured_cp", TransformStamped)
PSM1velocity = message_filters.Subscriber("/PSM1/measured_cv", TwistStamped)
PSM2velocity = message_filters.Subscriber("/PSM2/measured_cv", TwistStamped)
jaw1 = message_filters.Subscriber("/PSM1/jaw/measured_js", JointState)
jaw2 = message_filters.Subscriber("/PSM2/jaw/measured_js", JointState)
video_frame = message_filters.Subscriber("/jhu_daVinci/right/decklink/camera/image_raw/compressed", CompressedImage)

def main():

    ###################################################### NODE INIZIALIZATION ######################################################

    rospy.init_node('pyrecorder', anonymous=True)

    writer.writerow(['Predicted Gesture','Timestamps','PSM1x','PSM1y','PSM1z','PSM1_quat_x','PSM1_quat_y','PSM1_quat_z','PSM1_quat_w','PSM2x','PSM2y','PSM2z',
                    'PSM2_quat_x','PSM2_quat_y','PSM2_quat_z','PSM2_quat_w','PSM1_linearv_x','PSM1_linearv_y','PSM1_linearv_z',
                    'PSM1_angularv_x','PSM1_angularv_y','PSM1_angularv_z','PSM2_linearv_x','PSM2_linearv_y','PSM2_linearv_z',
                    'PSM2_angularv_x','PSM2_angularv_y','PSM2_angularv_z','jaw1_position','jaw1_velocity','jaw1_effort',
                    'jaw2_position','jaw2_velocity','jaw2_effort','Distance'])

    ###################################################### TIME SYNCHRONIZATION #####################################################


    ts = message_filters.ApproximateTimeSynchronizer([PSM1position, PSM2position, PSM1velocity, PSM2velocity, jaw1, jaw2, video_frame], 1, 0.1, allow_headerless=True)
    ts.registerCallback(sync_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
