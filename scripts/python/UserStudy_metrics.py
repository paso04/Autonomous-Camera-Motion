#!/usr/bin/env python
import time
import rospy
import numpy as np
import pandas as pd
import os
import message_filters
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Joy
import PyKDL
import dvrk
from tf_conversions import posemath

######################################################### CSV creation ###########################################################

path = '/home/npasini1/Desktop/dVRK_UserStudy/dVRK_metrics/'
name = input('Please, specify your name: ')
csvname = name + ".csv"
csvFileName = os.path.join(path,csvname)

###################################################### METRICS VARIABLES #########################################################

clutch_press = 0
camera_press = 0
flag_first_cb = True
list_current_distance_L = []
list_current_distance_R = []
total_distance_L = 0
total_distance_R = 0
time_pre = 0
cnt = 0
jerk_x_PSM2 = []
jerk_y_PSM2 = []
jerk_z_PSM2 = []
jerk_x_PSM1 = []
jerk_y_PSM1 = []
jerk_z_PSM1 = []
jerk_PSM1 = []
jerk_PSM2 = []
x1 = []
y1 = []
z1 = []
x2 = []
y2 = []
z2 = []
ox1 = []
oy1 = []
oz1 = []
ow1 = []
ox2 = []
oy2 = []
oz2 = []
ow2 = []
xL = []
yL = []
zL = []
xR = []
yR = []
zR = []
oxL = []
oyL = []
ozL = []
owL = []
oxR = []
oyR = []
ozR = []
owR = []
####################################################### CALLBACK FUNCTION #######################################################

# At the first call, let's store the frame for both RCMs of PSM1 and 2

def toCSV():

    PSM1position_sub.sub.unregister()
    PSM2position_sub.sub.unregister()
    PSM1velocity_sub.sub.unregister()
    PSM2velocity_sub.sub.unregister()
    MTMRpose_sub.sub.unregister()
    MTMLpose_sub.sub.unregister()

    time.sleep(2)
    time_t = (time.time() - start_time)
    print("--- %s seconds ---" % time_t)

    df_list_current_distance_L = pd.DataFrame(list_current_distance_L, columns = ['Current distance MTML'])
    df_list_current_distance_R = pd.DataFrame(list_current_distance_R, columns = ['Current distance MTMR'])
    df_jerk_x_PSM1 = pd.DataFrame(jerk_x_PSM1, columns = ['jerk_x_PSM1'])
    df_jerk_y_PSM1 = pd.DataFrame(jerk_y_PSM1, columns = ['jerk_y_PSM1'])
    df_jerk_z_PSM1 = pd.DataFrame(jerk_z_PSM1, columns = ['jerk_z_PSM1'])
    df_jerk_x_PSM2 = pd.DataFrame(jerk_x_PSM2, columns = ['jerk_x_PSM2'])
    df_jerk_y_PSM2 = pd.DataFrame(jerk_y_PSM2, columns = ['jerk_y_PSM2'])
    df_jerk_z_PSM2 = pd.DataFrame(jerk_z_PSM2, columns = ['jerk_z_PSM2'])
    df_jerk_PSM1 = pd.DataFrame(jerk_PSM1, columns = ['jerk_PSM1'])
    df_jerk_PSM2 = pd.DataFrame(jerk_PSM2, columns = ['jerk_PSM2'])
    df_time_t = pd.DataFrame([time_t], columns = ['Execution time'])
    df_clutch_press = pd.DataFrame([clutch_press], columns = ['Clutch presses'])
    df_camera_press = pd.DataFrame([camera_press], columns = ['Camera presses'])
    df_path_length_PSM1 = pd.DataFrame([path_length_PSM1], columns = ['Total PSM1 path length'])
    df_path_length_PSM2 = pd.DataFrame([path_length_PSM2], columns = ['Total PSM2 path length'])
    df_path_length_ECM = pd.DataFrame([path_length_ecm], columns = ['Total ECM path length'])
    df_average_distance_R = pd.DataFrame([average_distance_R], columns = ['Average MTMR comfort zone distance'])
    df_average_distance_L = pd.DataFrame([average_distance_L], columns = ['Average MTML comfort zone distance'])
    df_x1 = pd.DataFrame(x1, columns = ['x_PSM1'])
    df_y1 = pd.DataFrame(y1, columns = ['y_PSM1'])
    df_z1 = pd.DataFrame(z1, columns = ['z_PSM1'])
    df_x2 = pd.DataFrame(x2, columns = ['x_PSM2'])
    df_y2 = pd.DataFrame(y2, columns = ['y_PSM2'])
    df_z2 = pd.DataFrame(z2, columns = ['z_PSM2'])
    df_ox1 = pd.DataFrame(ox1, columns = ['pose_x_PSM1'])
    df_oy1 = pd.DataFrame(oy1, columns = ['pose_y_PSM1'])
    df_oz1 = pd.DataFrame(oz1, columns = ['pose_z_PSM1'])
    df_ow1 = pd.DataFrame(ow1, columns = ['pose_w_PSM1'])
    df_ox2 = pd.DataFrame(ox2, columns = ['pose_x_PSM2'])
    df_oy2 = pd.DataFrame(oy2, columns = ['pose_y_PSM2'])
    df_oz2 = pd.DataFrame(oz2, columns = ['pose_z_PSM2'])
    df_ow2 = pd.DataFrame(ow2, columns = ['pose_w_PSM2'])
    df_xL = pd.DataFrame(xL, columns = ['x_MTML'])
    df_yL = pd.DataFrame(yL, columns = ['y_MTML'])
    df_zL = pd.DataFrame(zL, columns = ['z_MTML'])
    df_xR = pd.DataFrame(xR, columns = ['x_MTMR'])
    df_yR = pd.DataFrame(yR, columns = ['y_MTMR'])
    df_zR = pd.DataFrame(zR, columns = ['z_MTMR'])
    df_oxL = pd.DataFrame(oxL, columns = ['pose_x_MTML'])
    df_oyL = pd.DataFrame(oyL, columns = ['pose_y_MTML'])
    df_ozL = pd.DataFrame(ozL, columns = ['pose_z_MTML'])
    df_owL = pd.DataFrame(owL, columns = ['pose_w_MTML'])
    df_oxR = pd.DataFrame(oxR, columns = ['pose_x_MTMR'])
    df_oyR = pd.DataFrame(oyR, columns = ['pose_y_MTMR'])
    df_ozR = pd.DataFrame(ozR, columns = ['pose_z_MTMR'])
    df_owR = pd.DataFrame(owR, columns = ['pose_w_MTMR'])

    df = pd.concat([df_time_t,df_clutch_press,df_camera_press,df_path_length_PSM1,df_path_length_PSM2,df_path_length_ECM,df_average_distance_R,df_average_distance_L,
                    df_list_current_distance_R,df_list_current_distance_L,df_jerk_PSM1,df_jerk_PSM2,df_jerk_x_PSM1,df_jerk_y_PSM1,df_jerk_z_PSM1,
                    df_jerk_x_PSM2,df_jerk_y_PSM2,df_jerk_z_PSM2,df_x1,df_y1,df_z1,df_x2,df_y2,df_z2,df_ox1,df_oy1,df_oz1,df_ow1,df_ox2,df_oy2,
                    df_oz2,df_ow2,df_xL,df_yL,df_zL,df_xR,df_yR,df_zR,df_oxL,df_oyL,df_ozL,df_owL,df_oxR,df_oyR,df_ozR,df_owR],axis=1,sort=False)

    df.to_csv(csvFileName, index = True)
    print('\n...Saved metrics to CSV file... \n')

def clutch_callback(msg):
    global clutch_press
    if msg.buttons[0] == 1:
        clutch_press += 1

def camera_callback(msg):
    global camera_press
    if msg.buttons[0] == 1:
        camera_press += 1

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

def sync_callback(msg1, msg2, msg3, msg4, msg5, msg6, msg7):
    global psm1x,psm1y,psm1z,psm2x,psm2y,psm2z,ecmx,ecmy,ecmz
    global psm1lvx,psm1lvy,psm1lvz,psm2lvx,psm2lvy,psm2lvz
    global path_length_PSM1,path_length_PSM2,path_length_ecm
    global time_cur,flag_first_cb
    global jerk_x_PSM1,jerk_y_PSM1,jerk_z_PSM1,jerk_PSM1,jerk_x_PSM2,jerk_y_PSM2,jerk_z_PSM2,jerk_PSM2
    global cnt,list_current_distance_L,list_current_distance_R,total_distance_L,total_distance_R,average_distance_L,average_distance_R
    global x_comfort_zone_L,y_comfort_zone_L,z_comfort_zone_L,x_comfort_zone_R,y_comfort_zone_R,z_comfort_zone_R

    #Timestamp
    cnt += 1
    f1 = posemath.fromMsg(msg1.pose)
    psm1_to_cart = f1_cart_frame * f1
    quat1 = psm1_to_cart.M.GetQuaternion()
    f2 = posemath.fromMsg(msg2.pose)
    psm2_to_cart = f2_cart_frame * f2
    quat2 = psm2_to_cart.M.GetQuaternion()

    if flag_first_cb:
        path_length_PSM1 = 0
        path_length_PSM2 = 0
        path_length_ecm = 0
        psm1x = psm1_to_cart.p[0]
        psm1y = psm1_to_cart.p[1]
        psm1z = psm1_to_cart.p[2]
        psm2x = psm2_to_cart.p[0]
        psm2y = psm2_to_cart.p[1]
        psm2z = psm2_to_cart.p[2]
        ecmx = msg7.pose.position.x
        ecmy = msg7.pose.position.y
        ecmz = msg7.pose.position.z
        psm1lvx = msg3.twist.linear.x
        psm1lvy = msg3.twist.linear.y
        psm1lvz = msg3.twist.linear.z
        psm2lvx = msg4.twist.linear.x
        psm2lvy = msg4.twist.linear.y
        psm2lvz = msg4.twist.linear.z
        x_comfort_zone_R = msg5.pose.position.x
        y_comfort_zone_R = msg5.pose.position.y
        z_comfort_zone_R = msg5.pose.position.z
        x_comfort_zone_L = msg6.pose.position.x
        y_comfort_zone_L = msg6.pose.position.y
        z_comfort_zone_L = msg6.pose.position.z
        
        time_cur = time.time()
        flag_first_cb = False
        print('first')

    else:
        psm1x_pre = psm1x
        psm1y_pre = psm1y
        psm1z_pre = psm1z
        psm2x_pre = psm2x
        psm2y_pre = psm2y
        psm2z_pre = psm2z
        ecmx_pre = ecmx
        ecmy_pre = ecmy
        ecmz_pre = ecmz
        psm1x = psm1_to_cart.p[0]
        psm1y = psm1_to_cart.p[1]
        psm1z = psm1_to_cart.p[2]
        psm2x = psm2_to_cart.p[0]
        psm2y = psm2_to_cart.p[1]
        psm2z = psm2_to_cart.p[2]
        ecmx = msg7.pose.position.x
        ecmy = msg7.pose.position.y
        ecmz = msg7.pose.position.z
        psm1lvx_pre = psm1lvx
        psm1lvy_pre = psm1lvy
        psm1lvz_pre = psm1lvz
        psm2lvx_pre = psm2lvx
        psm2lvy_pre = psm2lvy
        psm2lvz_pre = psm2lvz
        psm1lvx = msg3.twist.linear.x
        psm1lvy = msg3.twist.linear.y
        psm1lvz = msg3.twist.linear.z
        psm2lvx = msg4.twist.linear.x
        psm2lvy = msg4.twist.linear.y
        psm2lvz = msg4.twist.linear.z
        time_pre = time_cur
        time_cur = time.time()
        dt = time_cur - time_pre
        jerkx1 = (psm1lvx - psm1lvx_pre) / dt
        jerky1 = (psm1lvy - psm1lvy_pre) / dt
        jerkz1 = (psm1lvz - psm1lvz_pre) / dt
        jerkx2 = (psm2lvx - psm2lvx_pre) / dt
        jerky2 = (psm2lvy - psm2lvy_pre) / dt
        jerkz2 = (psm2lvz - psm2lvz_pre) / dt
        x1.append(psm1x)
        y1.append(psm1y)
        z1.append(psm1z)
        x2.append(psm2x)
        y2.append(psm2y)
        z2.append(psm2z)
        jerk_PSM1.append(np.sqrt(jerkx1**2 + jerky1**2 + jerkz1**2))
        jerk_PSM2.append(np.sqrt(jerkx2**2 + jerky2**2 + jerkz2**2))
        jerk_x_PSM1.append(jerkx1)
        jerk_y_PSM1.append(jerky1)
        jerk_z_PSM1.append(jerkz1)
        jerk_x_PSM2.append(jerkx2)
        jerk_y_PSM2.append(jerky2)
        jerk_z_PSM2.append(jerkz2)
        segment_length_PSM1 = np.sqrt((psm1x-psm1x_pre)**2 + (psm1y-psm1y_pre)**2 + (psm1z-psm1z_pre)**2)
        segment_length_PSM2 = np.sqrt((psm2x-psm2x_pre)**2 + (psm2y-psm2y_pre)**2 + (psm2z-psm2z_pre)**2)
        segment_length_ecm = np.sqrt((ecmx-ecmx_pre)**2 + (ecmy-ecmy_pre)**2 + (ecmz-ecmz_pre)**2)
        path_length_PSM1 = path_length_PSM1 + segment_length_PSM1
        path_length_PSM2 = path_length_PSM2 + segment_length_PSM2
        path_length_ecm = path_length_ecm + segment_length_ecm

    # MTMS and orientations
    quat1 = psm1_to_cart.M.GetQuaternion()
    ox1.append(quat1[0])
    oy1.append(quat1[1])
    oz1.append(quat1[2])
    ow1.append(quat1[3])
    quat2 = psm2_to_cart.M.GetQuaternion()
    ox2.append(quat2[0])
    oy2.append(quat2[1])
    oz2.append(quat2[2])
    ow2.append(quat2[3])
    x_current_R = msg5.pose.position.x
    y_current_R = msg5.pose.position.y
    z_current_R = msg5.pose.position.z
    x_current_L = msg6.pose.position.x
    y_current_L = msg6.pose.position.y
    z_current_L = msg6.pose.position.z
    xR.append(x_current_R)
    yR.append(y_current_R)
    zR.append(z_current_R)
    xL.append(x_current_L)
    yL.append(y_current_L)
    zL.append(z_current_L)
    oxR.append(msg5.pose.orientation.x)
    oyR.append(msg5.pose.orientation.y)
    ozR.append(msg5.pose.orientation.z)
    owR.append(msg5.pose.orientation.w)
    oxL.append(msg6.pose.orientation.x)
    oyL.append(msg6.pose.orientation.y)
    ozL.append(msg6.pose.orientation.z)
    owL.append(msg6.pose.orientation.w)
    current_distance_R = np.sqrt((x_current_R-x_comfort_zone_R)**2 + (y_current_R-y_comfort_zone_R)**2 + (z_current_R-z_comfort_zone_R)**2)
    total_distance_R = total_distance_R + current_distance_R
    list_current_distance_R.append(current_distance_R*100)
    average_distance_R = total_distance_R / cnt
    current_distance_L = np.sqrt((x_current_L-x_comfort_zone_L)**2 + (y_current_L-y_comfort_zone_L)**2 + (z_current_L-z_comfort_zone_L)**2)
    total_distance_L = total_distance_L + current_distance_L
    list_current_distance_L.append(current_distance_L*100)
    average_distance_L = total_distance_L / cnt

########################################################## SUBSCRIBERS ##########################################################

f1_cart_sub = rospy.Subscriber("/SUJ/PSM1/local/measured_cp", PoseStamped, psm1_cart_callback)
f2_cart_sub = rospy.Subscriber("/SUJ/PSM2/local/measured_cp", PoseStamped, psm2_cart_callback)
clutch_sub = rospy.Subscriber("/footpedals/clutch", Joy, clutch_callback)
camera_sub = rospy.Subscriber("/footpedals/camera", Joy, camera_callback)

PSM1position_sub = message_filters.Subscriber("/PSM1/local/measured_cp", PoseStamped)
PSM2position_sub = message_filters.Subscriber("/PSM2/local/measured_cp", PoseStamped)
PSM1velocity_sub = message_filters.Subscriber("/PSM1/local/measured_cv", TwistStamped)
PSM2velocity_sub = message_filters.Subscriber("/PSM2/local/measured_cv", TwistStamped)
MTMRpose_sub = message_filters.Subscriber("/MTMR/measured_cp", PoseStamped)
MTMLpose_sub = message_filters.Subscriber("/MTML/measured_cp", PoseStamped)
ECM_tip_sub = message_filters.Subscriber("/ECM/measured_cp", PoseStamped)

def main():

    ###################################################### NODE INIZIALIZATION ######################################################

    rospy.init_node('pymetric', anonymous=True)
    rospy.sleep(1)

    ###################################################### TIME SYNCHRONIZATION #####################################################

    ts = message_filters.ApproximateTimeSynchronizer([PSM1position_sub, PSM2position_sub, PSM1velocity_sub, PSM2velocity_sub, MTMRpose_sub, MTMLpose_sub, ECM_tip_sub], 1, 0.1, allow_headerless=True)
    ts.registerCallback(sync_callback)
    rospy.on_shutdown(toCSV)
    rospy.spin()

if __name__ == '__main__':
    global start_time
    start_time = time.time()
    main()