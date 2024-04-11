#!/usr/bin/python3
# pip install opencv-contrib-python
# pip install transforms3d==0.4.1

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from time import time 

import sys
import argparse
# Authors: Hamid Ebadi
# Run this after spawn-static-agents.sh script as below:
# python3 ./camera_viewer.py
# Background subtraction: https://docs.opencv.org/4.x/d1/dc5/tutorial_background_subtraction.html


# You may need to update
# - camera 'update_rate'
# - physics real_time_factor
# - physics real_time_update_rate

import cv2 as cv
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)
parameters =  cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)

import cameraCalibration

import shutil, os
DATA_DIR = 'images_data/'
shutil.rmtree(DATA_DIR, ignore_errors = True)
os.mkdir(DATA_DIR)

class MultiCameraSubscriber(Node):
    def __init__(self, camera_ids ):
        super().__init__('image_subscriber')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=len(camera_ids) *3 
        )

        for camera_id in camera_ids:
            camera_conf = cameraCalibration.Camera_config( "intrinsic/"+ camera_id +".yaml", "extrinsic/" + camera_id + ".yaml", camera_id )
            #create Background Subtractor objects
            if args.algo == 'MOG2':
                backSub[camera_id] = cv.createBackgroundSubtractorMOG2(history = 500, varThreshold = 8, detectShadows = False)
            elif args.algo == 'KNN':
                backSub[camera_id] = cv.createBackgroundSubtractorKNN(history=500, dist2Threshold=400.0, detectShadows = False)
            else:
                print ("No BackgroundSubtractor for camera: ",  camera_id)
            self.subscription = self.create_subscription(
                Image,
                '/static_agents/camera_' + camera_id + '/image_raw',
                lambda msg, camera_id=camera_id: self.image_callback(msg, camera_id, camera_conf),
                qos_profile  # QoS
            )
            self.cv_bridge = CvBridge()

    def image_callback(self, msg, camera_id, camera_conf):
        timestamp = str(int(time()))
        print(camera_id, timestamp)
        try:
            # Convert ROS Image message to OpenCV format imgmsg_to_cv
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # np_image = numpy.array(cv_image)
            # detect_aruco(cv_image)
            if args.action == 'save':
                #cv.imshow('raw-image-'  + camera_id , cv_image)
                cv.imwrite(os.path.join(DATA_DIR,  "raw_" + timestamp  + "_" + camera_id + ".png"), cv_image)
                # cv.waitKey(1)
            elif args.action == 'removebg':
                fgMask = backSub[camera_id].apply(cv_image)
                #cv.imshow('mask-image-'  + camera_id , fgMask)
                cv.imwrite(os.path.join(DATA_DIR,  "mask_" + timestamp + "_" + camera_id + ".png"), fgMask)
                # cv.waitKey(1)
            else:
                print("unknown action" + args.action)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")


def main():

    rclpy.init()
    # all cameras
    # cam_list = ['160','161','162','163','164','165','166','167', '168', '169', '170', '171']
    cam_list = ['164','165','166','167', '168']

    try:
        node = MultiCameraSubscriber(cam_list)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--action', help='continuously save camera images (for background extraction)' )
    parser.add_argument('--algo', type=str, help='Background subtraction method (KNN, MOG2).', default='MOG2')
    args = parser.parse_args()
    backSub = {}
    main()


def detect_aruco(cv_image):
    # Aruco detection code is based on https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
    corners, ids, rejectedCandidates = detector.detectMarkers(cv_image)
    if len(corners) > 0:
        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            # print(topRight, bottomRight, bottomLeft, topLeft)
            # draw the bounding box
            cv.line(cv_image, topLeft, topRight, (0, 255, 0), 2)
            cv.line(cv_image, topRight, bottomRight, (0, 255, 0), 2)
            cv.line(cv_image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv.line(cv_image, bottomLeft, topLeft, (0, 255, 0), 2)
            # compute and draw the center (x, y)-coordinates of the ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv.circle(cv_image, (cX, cY), 4, (0, 0, 255), -1)
            cv.putText(cv_image, str(markerID),
                (topLeft[0], topLeft[1] - 15),
                cv.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)