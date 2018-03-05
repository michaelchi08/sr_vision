#!/usr/bin/env python
#
# Copyright (C) 2017 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
import urllib2
import json
from sr_robot_commander.sr_arm_commander import SrArmCommander
from copy import deepcopy
import numpy as np
import tf2_ros
import tf
import geometry_msgs.msg

plan_b_offset = -0.01
object2grasp_z_offset = 0.2  # 0.1
temp_z_offset = 0  # 0.25

class SimplePickAndPlace():
    def __init__(self):
        self.home_position = {'ra_shoulder_pan_joint': 1.147,
                              'ra_elbow_joint': 1.695,
                              'ra_wrist_1_joint': -1.395,
                              'ra_wrist_2_joint': -1.584,
                              'ra_shoulder_lift_joint': -1.926,
                              'ra_wrist_3_joint': 1.830}
       
        self.z_approach = 0.1
        self.grasp_pose = [0.541867867754, 0.516208852914, 1.035 + plan_b_offset, 0, 1.57, 0]

        self.pre_grasp_pose = self.create_approach_pose_from_top(self.grasp_pose)

        self.release_pose = [0.541867867754, 0.516208852914, 1.035 + plan_b_offset, 0, 1.57, 0]

        self.pre_release_pose = self.create_approach_pose_from_top(self.release_pose)

        # self.arm_commander = SrArmCommander(name='right_arm', set_ground=False)
        # self.arm_commander.set_max_velocity_scaling_factor(0.2)
        # self.arm_commander.set_max_acceleration_scaling_factor(1.0)

    def set_grasp_pose(self, new_pose):
        self.grasp_pose = new_pose
        self.grasp_pose[2] = self.grasp_pose[2] # + object2grasp_z_offset
        self.pre_grasp_pose = self.create_approach_pose_from_top(self.grasp_pose)

    def set_release_pose(self, new_pose):
        self.release_pose = new_pose
        self.pre_release_pose = self.create_approach_pose_from_top(self.release_pose)

    def create_approach_pose_from_top(self, grasp_or_release_pose):
        # print grasp_or_release_pose
        approach_pose = deepcopy(grasp_or_release_pose)
        approach_pose[2] = approach_pose[2] + self.z_approach
        return approach_pose

    def go_home(self):
        self.arm_commander.plan_to_joint_value_target(self.home_position)
        if self.arm_commander.check_plan_is_valid():
            rospy.loginfo("Returning to home position.")
            # raw_input("Check plan and press Enter to continue...")
            self.arm_commander.execute()
        else:
            rospy.logerr("Failed to go to home position - no valid plan found.")
            
    def go_to_pose(self, pose):
        self.arm_commander.plan_to_pose_target(pose)
        if self.arm_commander.check_plan_is_valid():
            rospy.loginfo("Going to position.")
            # raw_input("Check plan and press Enter to continue...")
            self.arm_commander.execute()
        else:
            rospy.logerr("Failed to go to position - no valid plan found.")

    # grasp ids: handH_cylindrical_precision_large, handH_tripod_precision, handH_cylindrical_power
    def set_hand_state(self, grasp_state):
        grasp_cmd = [{"grasp_id": "handH_cylindrical_power", "grasp_state": grasp_state, "max_torque": 200}]
        req = urllib2.Request('http://0.0.0.0:8080/select_grasp')
        req.add_header('Content-Type', 'application/json')
        response = urllib2.urlopen(req, json.dumps(grasp_cmd))
        return response


CAMERA_FRAME = "cameraLeft_optical"
left_image_topic = "/stereo/left/image_raw"
right_image_topic = "/stereo/right/image_raw"
DIR = "/tmp/.X11-unix/iview/"


class Transformations:
    def __init__(self):
        self.tf_br = tf2_ros.StaticTransformBroadcaster()
        # self.t_msg = geometry_msgs.msg.TransformStamped()
        self.trans = geometry_msgs.msg.Transform()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def get_grasp_pose(self):
        # Gets transform from text file and stores in self.trans
        self.put_list_into_transform_type(self.read_object_to_camera_transform_from_text_file())


        # Create tf from camera to object
        self.make_object_tf("object_to_grasp", CAMERA_FRAME, self.trans)

        grasp_pose = self.set_world2object(self.trans)  # returns world -> object transform

        # TEST FOR ALIGNMENT OF WORLD2OBJECT AND CAMERA2OBJECT - Warning: changes self.t_msg
        # self.tf_br2 = tf2_ros.TransformBroadcaster()

        # ra_forearm_link

        # Create tf for world to object (grasp_pose)
        self.make_object_tf("grasp_pose", "world", grasp_pose)
        return grasp_pose

    def make_object_tf(self, name, camera_frame, transform):
        t_msg = geometry_msgs.msg.TransformStamped()
        t_msg.header.stamp = rospy.Time.now()
        t_msg.header.frame_id = camera_frame
        t_msg.child_frame_id = name
        t_msg.transform.translation = transform.translation
        t_msg.transform.rotation = transform.rotation
        # print transform
        self.tf_br.sendTransform(t_msg)

        print "Created tf for: " + name
        # print t_msg
        # print " "

    # get transform from text file generated from Halcon
    def read_object_to_camera_transform_from_text_file(self):
        text_file = "camera_object_halcon.txt"
        transform = []
        with open(DIR + text_file) as fp:
            for count, line in enumerate(fp):
                if line[0] == "2":
                    transform.append(float(line[1:25]))
                if line[0] == "1":
                    break
        return transform

    def put_list_into_transform_type(self, cartesian_transform_list):
        self.trans.translation.x = cartesian_transform_list[0]
        self.trans.translation.y = cartesian_transform_list[1]
        self.trans.translation.z = cartesian_transform_list[2]
        quaternion = tf.transformations.quaternion_from_euler(cartesian_transform_list[3], cartesian_transform_list[4],
                                                              cartesian_transform_list[5])
        self.trans.rotation.x = quaternion[0]
        self.trans.rotation.y = quaternion[1]
        self.trans.rotation.z = quaternion[2]
        self.trans.rotation.w = quaternion[3]

    def set_world2camera(self):
        world2camera = self.tfBuffer.lookup_transform("world", CAMERA_FRAME, rospy.Time(0), rospy.Duration(3.0))
        world2camera_trans = [world2camera.transform.translation.x, world2camera.transform.translation.y,
                              world2camera.transform.translation.z]
        world2camera_rota = [world2camera.transform.rotation.x, world2camera.transform.rotation.y,
                             world2camera.transform.rotation.z, world2camera.transform.rotation.w]
        world2camera_trans_matrix = tf.transformations.translation_matrix(world2camera_trans)
        world2camera_rota_matrix = tf.transformations.quaternion_matrix(world2camera_rota)
        self.world2camera_transform = np.dot(world2camera_trans_matrix, world2camera_rota_matrix)

    def set_camera2object(self, object2camera_transform):
        camera2object_trans = [object2camera_transform.translation.x, object2camera_transform.translation.y,
                               object2camera_transform.translation.z]
        camera2object_rota = [object2camera_transform.rotation.x, object2camera_transform.rotation.y,
                              object2camera_transform.rotation.z, object2camera_transform.rotation.w]
        camera2object_trans_matrix = tf.transformations.translation_matrix(camera2object_trans)
        camera2object_rota_matrix = tf.transformations.quaternion_matrix(camera2object_rota)
        self.camera2object_transform = np.dot(camera2object_trans_matrix, camera2object_rota_matrix)

    def set_world2object(self, object2camera_transform):
        self.set_world2camera()
        self.set_camera2object(object2camera_transform)

        world2object = np.dot(self.world2camera_transform, self.camera2object_transform)


# XXXX Need to include rotation?
        world2object_t = self.trans
        quatern = tf.transformations.quaternion_from_matrix(world2object)
        # euler_from_matrix(matrix, axes='dxyz')

        world2object_t.rotation.x = quatern[0]
        world2object_t.rotation.y = quatern[1]
        world2object_t.rotation.z = quatern[2]
        world2object_t.rotation.w = quatern[3]
        world2object_t.translation.x = world2object[0, 3]
        world2object_t.translation.y = world2object[1, 3]
        world2object_t.translation.z = world2object[2, 3]
        return world2object_t

# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg, left_or_right):
    try:
        # Convert  ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save OpenCV2 image as a png
        print("Saved " + left_or_right + " image!")
        cv2.imwrite(DIR + left_or_right + '.png', cv2_img)
        rospy.sleep(2)

if __name__ == "__main__":

    rospy.init_node("Serfo_demo_script")

    # raw_input("Press Enter when happy with recognition...")
    # exit(0)

    spnp = SimplePickAndPlace()
    transformations = Transformations()

    # print spnp.arm_commander.get_current_pose().position
    # exit(0)

    # left_camera_sub = rospy.Subscriber(left_image_topic, Image, image_callback, "left")
    # right_camera_sub = rospy.Subscriber(right_image_topic, Image, image_callback, "right")

    # spnp.go_home()
    # spnp.set_hand_state("pre_grasp")
    rospy.sleep(2.0)

    raw_input("After placing object, press Enter to continue...")
    # Plan B: Remove subscribers
    # Set up subscribers for left and right camera image taking - XXX Kill subscribers?
    rospy.Subscriber(left_image_topic, Image, image_callback, "left")
    rospy.Subscriber(right_image_topic, Image, image_callback, "right")


    # Plan B: move to below user input - Remove this and cameras in launch file
    while not rospy.is_shutdown():
        inp = raw_input("Press Enter when happy with recognition...")
        if inp == "exit":
            exit(0)
        # gets grasp pose from txt file and publishes tf "grasp_pose"
        grasp_pose = transformations.get_grasp_pose()

        quaternion = (  grasp_pose.rotation.x,
                        grasp_pose.rotation.y,
                        grasp_pose.rotation.z,
                        grasp_pose.rotation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # grasp_pose.translation.y, -grasp_pose.translation.x
        spnp.set_grasp_pose([grasp_pose.translation.x, grasp_pose.translation.y, grasp_pose.translation.z + temp_z_offset,
                             0, 1.571, euler[2] + 1.571])

        # SET PRE-GRASP POSE

        # make tf for pre-grasp pose
        temp = geometry_msgs.msg.Transform()
        temp.translation.x = spnp.pre_grasp_pose[0]
        temp.translation.y = spnp.pre_grasp_pose[1]
        temp.translation.z = spnp.pre_grasp_pose[2]
        quatern = tf.transformations.quaternion_from_euler(spnp.pre_grasp_pose[3],
                                                           spnp.pre_grasp_pose[4],
                                                           spnp.pre_grasp_pose[5])
        temp.rotation.x = quatern[0]
        temp.rotation.y = quatern[1]
        temp.rotation.z = quatern[2]
        temp.rotation.w = quatern[3]
        transformations.make_object_tf("pre-grasp_pose", "world", temp)

        # spnp.arm_commander.plan_to_pose_target(spnp.pre_grasp_pose)

        # rospy.sleep(2)

        # if spnp.arm_commander.check_plan_is_valid():
        #     inp = raw_input("rviz plan ok? (y/n)")
        #     if inp == "y":
        #         spnp.arm_commander.execute()
        #         break
        exit(0)

    raw_input("Press Enter to proceed with grasp...")

    # spnp.go_to_pose(spnp.pre_grasp_pose)

    spnp.go_to_pose(spnp.grasp_pose)


    raw_input("Press Enter when happy object tf location...")


    raw_input("Press Enter to grasp...")
    spnp.set_hand_state("grasp")

    raw_input("Press Enter when happy with grasp...")
    spnp.go_to_pose(spnp.pre_grasp_pose)
    spnp.go_home()

    # arm -> go to release
    raw_input("Press Enter place object...")
    spnp.go_to_pose(spnp.pre_release_pose)
    rospy.sleep(2.0)
    spnp.go_to_pose(spnp.release_pose)

    # hand -> pre grasp
    raw_input("Press Enter to let go...")
    spnp.set_hand_state("pre_grasp")

    raw_input("Press Enter and go to home...")
    spnp.go_home()
    
    #grasp = 'handH_tripod_power'
    #grasp_torque = 200

    exit(0)
    #print spnp.arm_commander.get_current_pose().position
