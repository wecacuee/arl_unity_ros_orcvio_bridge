#!/usr/bin/env python
import os.path
import copy
from functools import partial

import rospy
from geometry_msgs.msg import PoseStamped
from starmap_ros_msgs.msg import TrackedBBoxListWithKeypoints
from arl_unity_ros.msg import ImageDetections

name_list = ["upper_left_windshield","upper_right_windshield",
            "upper_right_rearwindow","upper_left_rearwindow",
            "left_front_light","right_front_light",
            "right_back_trunk","left_back_trunk",
            "left_front_wheel","left_back_wheel",
            "right_front_wheel","right_back_wheel"]


def scriptdir(f=__file__):
    return os.path.dirname(f)

def rel2abs(relpath, refdir=scriptdir()):
    return os.path.join(refdir , relpath)


GLOBAL_POSE_STAMPED_TEMPLATE = None
def load_pose_stamped_msg_template():
    global GLOBAL_POSE_STAMPED_TEMPLATE
    if GLOBAL_POSE_STAMPED_TEMPLATE is None:
        GLOBAL_POSE_STAMPED_TEMPLATE = PoseStamped()
        GLOBAL_POSE_STAMPED_TEMPLATE.deserialize(
            open(rel2abs('pose_msg_template.msg'), 'r').read())
    return copy.deepcopy(GLOBAL_POSE_STAMPED_TEMPLATE)

GLOBAL_KPTS_MSG_TEMPLATE = None
def load_tracked_bbox_msg_template():
    global GLOBAL_KPTS_MSG_TEMPLATE
    if GLOBAL_KPTS_MSG_TEMPLATE is None:
        GLOBAL_KPTS_MSG_TEMPLATE = TrackedBBoxListWithKeypoints()
        GLOBAL_KPTS_MSG_TEMPLATE.deserialize(
            open(rel2abs('kpts_msg_template.msg'), 'r').read())
    return copy.deepcopy(GLOBAL_KPTS_MSG_TEMPLATE)

def on_detection(tracked_bbox_publisher, msg):
    kpts_msg_template = load_tracked_bbox_msg_template()
    bbox_template = kpts_msg_template.bounding_boxes[0].bbox
    keypoint_template = kpts_msg_template.bounding_boxes[0].keypoints[0]
    det_template = kpts_msg_template.bounding_boxes[0]
    kpts_msg_template.header = msg.header
    kpts_msg_template.bounding_boxes = []
    for detection in msg.detections:
        obj_det = copy.deepcopy(det_template)
        obj_det.bbox.xmin = detection.x_min
        obj_det.bbox.ymin = detection.y_min
        obj_det.bbox.xmax = detection.x_max
        obj_det.bbox.ymax = detection.y_max
        obj_det.bbox.id = detection.object_id
        obj_det.bbox.Class = detection.class_name
        obj_det.bbox.lost_flag = True
        obj_det.keypoints = []
        for kpt in detection.kpts:
            obj_kpt = copy.deepcopy(keypoint_template)
            obj_kpt.x = kpt.x
            obj_kpt.y = kpt.y
            obj_kpt.semantic_part_label = kpt.id 
            obj_kpt.semantic_part_label_name = name_list[kpt.id]
            obj_det.keypoints.append(obj_kpt)
        kpts_msg_template.bounding_boxes.append(obj_det)
    tracked_bbox_publisher.publish(kpts_msg_template)


global first_pose_flag
def on_odom(pose_publisher, msg):

    pose_msg_template = load_pose_stamped_msg_template()
    pose_template = pose_msg_template.pose 
    if first_pose_flag:
        initial_z_offset = msg.pose.pose.position.z
        first_pose_flag = False 

    pose_msg_template.header = msg.header
    pose = copy.deepcopy(pose_template)

    pose.position.x = msg.pose.pose.position.x
    pose.position.y = msg.pose.pose.position.y
    pose.position.z = msg.pose.pose.position.z - initial_z_offset + 0.5

    pose.orientation.x = msg.pose.pose.orientation.x
    pose.orientation.y = msg.pose.pose.orientation.y
    pose.orientation.z = msg.pose.pose.orientation.z
    pose.orientation.w = msg.pose.pose.orientation.w

    pose_msg_template.pose = pose
    pose_publisher.publish(pose_msg_template)

def run_node():
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    tracked_bbox_publisher = rospy.Publisher(
        "keypoints", TrackedBBoxListWithKeypoints, queue_size=10)
    sub = rospy.Subscriber("detections", ImageDetections,
                    partial(on_detection, tracked_bbox_publisher))

    rospy.spin()

if __name__ == '__main__':
    run_node()
