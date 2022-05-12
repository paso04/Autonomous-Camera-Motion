#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2020-2021 Johns Hopkins University (JHU), Worcester Polytechnic Institute (WPI) All Rights Reserved.


#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.


#     \author    <amunawar@jhu.edu>
#     \author    Adnan Munawar
#     \version   1.0
# */
# //==============================================================================
import sys
from ambf_client import Client
from surgical_robotics_challenge.psm_arm import PSM
import time
import rospy
import numpy as np
from PyKDL import Frame, Rotation, Vector, Wrench
from argparse import ArgumentParser
from input_devices.mtm_device_crtk import MTM
from itertools import cycle
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.utils.jnt_control_gui import JointGUI
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
import PyKDL
import message_filters

global camera_pedal
camera_pedal = False

class ControllerInterface:
    def __init__(self, leader, psm_arms, camera):
        self.counter = 0
        self.leader = leader
        self.psm_arms = cycle(psm_arms)
        if sys.version_info[0] >= 3:
            self.active_psm = next(self.psm_arms)
        else:
            self.active_psm = self.psm_arms.next()
        self.gui = JointGUI('ECM JP', 4, ["ecm j0", "ecm j1", "ecm j2", "ecm j3"])

        self.cmd_xyz = self.active_psm.T_t_b_home.p
        self.cmd_rpy = None
        self.T_IK = None
        self._camera = camera

        self._T_c_b = None
        self._update_T_c_b = True

        self.leader.enable_gravity_comp()

    def switch_psm(self):
        self._update_T_c_b = True
        self.active_psm = self.psm_arms.next()
        print('Switching Control of Next PSM Arm: ', self.active_psm.name)

    def update_T_b_c(self):
        if self._update_T_c_b or self._camera.has_pose_changed():
            self._T_c_b = self.active_psm.get_T_w_b() * self._camera.get_T_c_w()
            self._update_T_c_b = False

    def update_camera_pose(self):
        self.gui.App.update()
        self._camera.servo_jp(self.gui.jnt_cmds)

    def update_arm_pose(self):
        self.update_T_b_c()
        if self.leader.coag_button_pressed or self.leader.clutch_button_pressed:
            # self.leader.optimize_wrist_platform()
            f = Wrench()
            self.leader.servo_cf(f)
        else:
            if self.leader.is_active():
                self.leader.servo_cp(self.leader.pre_coag_pose_msg)
        twist = self.leader.measured_cv() * 0.035
        self.cmd_xyz = self.active_psm.T_t_b_home.p
        if not self.leader.clutch_button_pressed:
            delta_t = self._T_c_b.M * twist.vel
            self.cmd_xyz = self.cmd_xyz + delta_t
            self.active_psm.T_t_b_home.p = self.cmd_xyz
        if self.leader.coag_button_pressed:
            self.cmd_rpy = self._T_c_b.M * self.leader.measured_cp().M
            self.T_IK = Frame(self.cmd_rpy, self.cmd_xyz)
            self.active_psm.servo_cp(self.T_IK)
        self.active_psm.set_jaw_angle(self.leader.get_jaw_angle())

    def update_visual_markers(self):
        # Move the Target Position Based on the GUI
        if self.active_psm.target_IK is not None:
            T_t_w = self.active_psm.get_T_b_w() * self.T_IK
            self.active_psm.target_IK.set_pos(T_t_w.p[0], T_t_w.p[1], T_t_w.p[2])
            self.active_psm.target_IK.set_rpy(T_t_w.M.GetRPY()[0], T_t_w.M.GetRPY()[1], T_t_w.M.GetRPY()[2])
        # if self.arm.target_FK is not None:
        #     ik_solution = self.arm.get_ik_solution()
        #     ik_solution = np.append(ik_solution, 0)
        #     T_7_0 = convert_mat_to_frame(compute_FK(ik_solution))
        #     T_7_w = self.arm.get_T_b_w() * T_7_0
        #     P_7_0 = T_7_w.p
        #     RPY_7_0 = T_7_w.M.GetRPY()
        #     self.arm.target_FK.set_pos(P_7_0[0], P_7_0[1], P_7_0[2])
        #     self.arm.target_FK.set_rpy(RPY_7_0[0], RPY_7_0[1], RPY_7_0[2])



    def run(self):
        if self.leader.switch_psm:
            self.switch_psm()
            self.leader.switch_psm = False
        # self.update_camera_pose()
        if not camera_pedal:
            self.update_arm_pose()
        elif camera_pedal:
            cam.servo_cp(fps)
        # else:
        #     V_RPY_2 = c.get_obj_pose('/ambf/env/psm2/toolyawlink')
        #     V_RPY_1 = c.get_obj_pose('/ambf/env/psm1/toolyawlink')
        #     fps = PyKDL.Frame(T_c_w.M, PyKDL.Vector((V_RPY_1[0]+V_RPY_2[0])/2,(V_RPY_1[0]+V_RPY_2[0])/2 + 1.63076055,(V_RPY_1[0]+V_RPY_2[0])/2 + 1.9712601185))
        #     cam.servo_cp(fps)
        #     print("ciao")
            
            # get_pos da entrambi poi fai il punto medio poi lo passi a set_pos / servo_cp per la camera
        # self.update_visual_markers()

######################################################################################################### CAMERA PEDAL #########################################################################################################

def cam_CB(msg1, msg2):
    print('callback')
    global camera_pedal
    global cam_pose
    
    camera_pedal = False
    if msg1.buttons[0] == 1:
        camera_pedal = True
        print("Pressing camera pedal")
        cam_pose = c.get_obj_pose('/ambf/env/CameraFrame')
        start_mtm2_x = msg2.pose.position.x
        start_mtm2_y = msg2.pose.position.y
        start_mtm2_z = msg2.pose.position.z
        global starting_mtm2_position
        starting_mtm2_position = [start_mtm2_x, start_mtm2_y, start_mtm2_z]
        # V_RPY_2 = c.get_obj_pose('/ambf/env/psm2/toolyawlink')
        # V_RPY_1 = c.get_obj_pose('/ambf/env/psm1/toolyawlink')
        # fps = PyKDL.Frame(T_c_w.M, PyKDL.Vector((V_RPY_1[0]+V_RPY_2[0])/2,(V_RPY_1[0]+V_RPY_2[0])/2 + 1.63076055,(V_RPY_1[0]+V_RPY_2[0])/2 + 1.9712601185))
        # cam.servo_cp(fps)
        # print("ciao")
    else:
        print("Not pressing camera pedal")

    return camera_pedal, cam_pose, starting_mtm2_position

def MTM2_CB(msg):
    # global MTM1_position
    # MTM1_position = np.array((msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z))
    if camera_pedal:
        global fps
        fps = PyKDL.Frame(T_c_w.M, PyKDL.Vector(cam_pose[0] + msg.pose.position.x - starting_mtm2_position[0], msg.pose.position.y - starting_mtm2_position[1], msg.pose.position.z - starting_mtm2_position[2]))
            
        print('ciao')
    return fps





######################################################################################################### CAMERA PEDAL #########################################################################################################

cam_sub = message_filters.Subscriber("/footpedals/camera", Joy)
# MTM1_sub = rospy.Subscriber("/MTMR/local/measured_cp", TransformStamped, MTM1_CB)
MTM2_sub = message_filters.Subscriber("/MTML/local/measured_cp", PoseStamped)

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-c', action='store', dest='client_name', help='Client Name', default='ambf_client')
    parser.add_argument('--one', action='store', dest='run_psm_one', help='Control PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='Control PSM2', default=True)
    parser.add_argument('--three', action='store', dest='run_psm_three', help='Control PSM3', default=True)
    parser.add_argument('--mtm', action='store', dest='mtm_name', help='Name of MTM to Bind', default='/dvrk/MTMR/')

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    mtm_valid_list = ['/MTMR/, /MTML/', '/dvrk/MTMR/', '/dvrk/MTML/', 'MTMR', 'MTML']
    if parsed_args.mtm_name in mtm_valid_list:
        if parsed_args.mtm_name in ['MTMR', 'MTML']:
            parsed_args.mtm_name = '/' + parsed_args.mtm_name + '/'
    else:
        print('ERROR! --mtm argument should be one of the following', mtm_valid_list)
        raise ValueError

    if parsed_args.run_psm_one in ['True', 'true', '1']:
        parsed_args.run_psm_one = True
    elif parsed_args.run_psm_one in ['False', 'false', '0']:
        parsed_args.run_psm_one = False

    if parsed_args.run_psm_two in ['True', 'true', '1']:
        parsed_args.run_psm_two = True
    elif parsed_args.run_psm_two in ['False', 'false', '0']:
        parsed_args.run_psm_two = False
    if parsed_args.run_psm_three in ['True', 'true', '1']:
        parsed_args.run_psm_three = True
    elif parsed_args.run_psm_three in ['False', 'false', '0']:
        parsed_args.run_psm_three = False

    c = Client(parsed_args.client_name)
    c.connect()

    cam = ECM(c, 'CameraFrame')
    T_c_w = cam.get_T_c_w()
    time.sleep(0.5)

    controllers = []
    psm_arms = []

    if parsed_args.run_psm_one is True:
        # Initial Target Offset for PSM1
        # init_xyz = [0.1, -0.85, -0.15]
        arm_name = 'psm1'
        print('LOADING CONTROLLER FOR ', arm_name)
        psm = PSM(c, arm_name, add_joint_errors=False)
        if psm.is_present():
            T_psmtip_c = Frame(Rotation.RPY(3.14, 0.0, -1.57079), Vector(-0.2, 0.0, -1.0))
            T_psmtip_b = psm.get_T_w_b() * cam.get_T_c_w() * T_psmtip_c
            psm.set_home_pose(T_psmtip_b)
            psm_arms.append(psm)

    if parsed_args.run_psm_two is True:
        # Initial Target Offset for PSM1
        # init_xyz = [0.1, -0.85, -0.15]
        arm_name = 'psm2'
        print('LOADING CONTROLLER FOR ', arm_name)
        theta_base = -0.7
        psm = PSM(c, arm_name, add_joint_errors=False)
        if psm.is_present():
            T_psmtip_c = Frame(Rotation.RPY(3.14, 0.0, -1.57079), Vector(0.2, 0.0, -1.0))
            T_psmtip_b = psm.get_T_w_b() * cam.get_T_c_w() * T_psmtip_c
            psm.set_home_pose(T_psmtip_b)
            psm_arms.append(psm)

    if parsed_args.run_psm_three is True:
        # Initial Target Offset for PSM1
        # init_xyz = [0.1, -0.85, -0.15]
        arm_name = 'psm3'
        print('LOADING CONTROLLER FOR ', arm_name)
        psm = PSM(c, arm_name, add_joint_errors=False)
        if psm.is_present():
            psm_arms.append(psm)

    if len(psm_arms) == 0:
        print('No Valid PSM Arms Specified')
        print('Exiting')

    else:
        leader = MTM(parsed_args.mtm_name)
        leader.set_base_frame(Frame(Rotation.RPY((3.14 - 0.8) / 2, 0, 0), Vector(0, 0, 0)))
        controller1 = ControllerInterface(leader, psm_arms, cam)
        controllers.append(controller1)

        rate = rospy.Rate(200)

        while not rospy.is_shutdown():
            for cont in controllers:
                cont.run()
            rate.sleep()


ts = message_filters.ApproximateTimeSynchronizer([cam_sub, MTM2_sub], 100, 10, allow_headerless=True)
ts.registerCallback(cam_CB)

rospy.spin()