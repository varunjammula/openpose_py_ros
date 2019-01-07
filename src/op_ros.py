#!/usr/bin/env python
import os
import sys
import time
from sys import platform
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, RegionOfInterest
import numpy as np

# dir_path = os.path.dirname(os.path.realpath(__file__))
# print dir_path
# if platform == "win32": sys.path.append(dir_path + '/../../python/openpose/');
# else: sys.path.append('../../python');
# # Option b
# # If you run `make install` (default path is `/usr/local/python` for Ubuntu), you can also access the OpenPose/python module from there. This will install OpenPose and the python library at your desired installation path. Ensure that this is in your python path in order to use it.
sys.path.append('/usr/local/python')

# Parameters for OpenPose. Take a look at C++ OpenPose example for meaning of components. Ensure all below are filled

from openpose import *

#
params = dict()
params["logging_level"] = 3
params["output_resolution"] = "-1x-1"
params["net_resolution"] = "-1x368"
params["model_pose"] = "BODY_25"
params["alpha_pose"] = 0.6
params["scale_gap"] = 0.3
params["scale_number"] = 1
params["render_threshold"] = 0.05
params["tracking"] = 5
params["number_people_max"] = 1
# If GPU version is built, and multiple GPUs are available, set the ID here
params["num_gpu_start"] = 0
params["disable_blending"] = False
# Ensure you point to the correct path where models are located
params["default_model_folder"] = '/home/apg/openpose/models/'
img_topic_name = '/turtlebot1/camera/rgb/image_raw'
# Construct OpenPose object allocates GPU memory
openpose = OpenPose(params)


class ROS2OpenCV2(object):

    def __init__(self, node_name):
        self.node_name = node_name

        rospy.init_node(node_name)
        rospy.loginfo("Starting node at " + str(node_name))

        rospy.on_shutdown(self.cleanup)

        # # Create the main display window
        self.cv_window_name = self.node_name

        # Create the cv_bridge object
        self.bridge = CvBridge()
        self.frame = None
        self.rect = None
        self.start = 0
        self.roi_pub = rospy.Publisher('/roi', RegionOfInterest, queue_size=10)
        self.image_sub = rospy.Subscriber(img_topic_name, Image, self.image_callback)

        while not rospy.is_shutdown():
            ROI = RegionOfInterest()
            self.rect = None

            #openpose-detection
            if self.frame is not None:
                self.start = time.time()
                keypoints = openpose.forward(self.frame, False)
                if len(keypoints) > 0:
                    b_parts = keypoints[0]
                    pts = []
                    for i in range(len(b_parts)):
                        #print 'x: {}, y:{}, confidence: {}'.format(b_parts[i][0], b_parts[i][1], b_parts[i][2])
                        if b_parts[i][2] > 0:
                            pts.append((b_parts[i][0], b_parts[i][1]))

                    points_matrix = np.array(pts).reshape((-1, 1, 2)).astype(np.int32)
                    # bounding-box
                    self.rect = cv2.boundingRect(points_matrix)


                # Compute the time for this loop and estimate CPS as a running average
                end = time.time()
                duration = end - self.start
                fps = int(1.0 / duration)

                cv2.putText(self.frame, "FPS: " + str(fps), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 0))

            if self.rect is not None:
                ROI.x_offset = int(self.rect[0])
                ROI.y_offset = int(self.rect[1])
                ROI.width = int(self.rect[2])
                ROI.height = int(self.rect[3])
                ROI.do_rectify = True
                cv2.rectangle(self.frame, (self.rect[0], self.rect[1]),
                              (self.rect[0] + self.rect[2], self.rect[1] + self.rect[3]), (0, 255, 0), 1)
            self.roi_pub.publish(ROI)

            # Update the image display
            if self.frame is not None:
                cv2.imshow(self.node_name, self.frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                return



    def image_callback(self, data):
        # Convert the ROS image to OpenCV format using a cv_bridge helper function
        # rospy.loginfo("Image callback!!")
        self.frame = self.convert_image(data)

    def convert_image(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return frame
        except CvBridgeError, e:
            print e

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()


def main(args):
    try:
        node_name = "Openpose-Tracker"
        ROS2OpenCV2(node_name)
        #rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ros2opencv node."
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)