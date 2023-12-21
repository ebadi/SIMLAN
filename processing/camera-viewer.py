import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Authors: Hamid Ebadi
# Run this after spawn-static-agents.sh script as below:
# python3 ./camera_viewer.py


import cv2 as cv
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)
parameters =  cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/static_agents/camera_1/image_raw',
            self.image_callback,
            10
        )
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format imgmsg_to_cv
            cv_image = self.cv_bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')

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
            cv.imshow('Image from ROS2', cv_image)
            cv.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass

    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
