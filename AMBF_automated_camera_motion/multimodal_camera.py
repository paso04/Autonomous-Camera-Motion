# Import the Client from ambf_client package
from ambf_client import Client
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.scene import Scene
from ambf_msgs.msg import RigidBodyState, SensorState
from geometry_msgs.msg import TransformStamped, TwistStamped
import tf_conversions.posemath as pm
import numpy as np
import time
import rospy
import PyKDL
import sys
if sys.version_info[0] < 3:
    input = raw_input

# Create a instance of the client
c = Client('MyClient')
c.connect()
time.sleep(2)

print('\n\n----')
print(c.get_obj_names())

# scene = Scene(c)


# ambf_cam_l = ECM(c, "cameraL")
# ambf_cam_r = ECM(c, "cameraR")


ecm = ECM(c, "CameraFrame")

### CANNOT USE PSM CLASS: it will cause collision with the Client called by the mtml and mtmr control scripts (multi_psm_control python script)
# PSM2 = PSM(c, "psm2")
# PSM1 = PSM(c, "psm1")

### NO NEED TO USE CAMR AND CAML SEPARATED: WE CAN DIRECTLY MOVE CAMEFRAME
# camR = c.get_obj_handle('cameraR')
# camL = c.get_obj_handle('cameraL')

sensor1 = c.get_obj_handle('/ambf/env/psm1/Sensor0')
sensor2 = c.get_obj_handle('/ambf/env/psm2/Sensor0')

# entry1 = c.get_obj_handle('Entry1')
# entry2 = c.get_obj_handle('Entry2')
# entry3 = c.get_obj_handle('Entry3')
# entry4 = c.get_obj_handle('Entry4')

# needle = c.get_obj_handle('Needle')

def PSM_cb(msg):

        T_c_w = ecm.get_T_c_w()

        needle = np.array(c.get_obj_pose('/ambf/env/Needle')[:3])
        entry1 = np.array(c.get_obj_pose('/ambf/env/Entry1')[:3])
        entry2 = np.array(c.get_obj_pose('/ambf/env/Entry2')[:3])
        entry3 = np.array(c.get_obj_pose('/ambf/env/Entry3')[:3])
        entry4 = np.array(c.get_obj_pose('/ambf/env/Entry4')[:3])

        squared_dist1 = np.sum((needle-entry1)**2, axis=0)
        dist1 = np.sqrt(squared_dist1)
        squared_dist2 = np.sum((needle-entry2)**2, axis=0)
        dist2 = np.sqrt(squared_dist2)
        squared_dist3 = np.sum((needle-entry3)**2, axis=0)
        dist3 = np.sqrt(squared_dist3)
        squared_dist4 = np.sum((needle-entry4)**2, axis=0)
        dist4 = np.sqrt(squared_dist4)

        entry_all = [entry1,entry2,entry3,entry4]
        dist_all = [dist1,dist2,dist3,dist4]
        idx_min = np.argmin(np.array((dist1,dist2,dist3,dist4)))
        min_dist = dist_all[idx_min]
        min_entry = entry_all[idx_min]

        # print(min_entry)

        if sensor2.get_sensed_object(0) == 'Needle':
                if min_dist > 0.15:
                        # print('RIGHT_NEEDLE_POSITIONING')
                        ### NEEDED????
                        # ambf_cam_l.set_pose_changed()
                        # ambf_cam_r.set_pose_changed()
                        V_RPY_2 = c.get_obj_pose('/ambf/env/psm2/toolyawlink')
                        fnp = PyKDL.Frame(T_c_w.M, PyKDL.Vector(V_RPY_2[0]+0.3,V_RPY_2[1]+0.863076055,V_RPY_2[2]+1.0012601185))
                        ecm.servo_cp(fnp)
                else:
                        # print('RIGHT_TISSUE_BITE')
                        ### NEEDED????
                        # ambf_cam_l.set_pose_changed()
                        # ambf_cam_r.set_pose_changed()
                        ftb = PyKDL.Frame(T_c_w.M, PyKDL.Vector(min_entry[0]+0.05,min_entry[1]+0.663076055,min_entry[2]+0.8712601185))
                        ecm.servo_cp(ftb)
                # else:
                #         print(2)
                #         V_RPY_2 = c.get_obj_pose('/ambf/env/psm2/toolyawlink')
                #         V_RPY_1 = c.get_obj_pose('/ambf/env/psm1/toolyawlink')
                #         T_c_w = ecm.get_T_c_w()
                #         f2 = PyKDL.Frame(T_c_w.M, PyKDL.Vector(V_RPY_2[0]+V_RPY_1[0],(V_RPY_2[0]+V_RPY_1[0])/2 + 0.963076055,(V_RPY_2[0]+V_RPY_1[0])/2 + 1.1712601185))
                #         ecm.servo_cp(f2)
        elif sensor1.get_sensed_object(0) == 'Needle':
                if min_dist > 0.15:
                        # print('LEFT_PULL_SUTURE_THROUGH')
                        V_RPY_1 = c.get_obj_pose('/ambf/env/psm1/toolyawlink')
                        V_RPY_2 = c.get_obj_pose('/ambf/env/psm2/toolyawlink')
                        fps = PyKDL.Frame(T_c_w.M, PyKDL.Vector((V_RPY_2[0]+V_RPY_1[0])/2,(V_RPY_2[1]+V_RPY_1[1])/2 + 0.963076055,(V_RPY_2[2]+V_RPY_1[2])/2 + 1.1712601185))
                        ecm.servo_cp(fps)


sub = rospy.Subscriber("/ambf/env/psm2/toolyawlink/State", RigidBodyState, PSM_cb)
rospy.spin()

# CamFrame get_pos:
# x: -1.396e-06
# y: 0.963076055
# z: 1.6712601185

# CamR get_pos:
# x: 0.02
# y: 0.0
# z: -0.5

# CamL get_pos:
# x: -0.02
# y: 0.0
# z: -0.5