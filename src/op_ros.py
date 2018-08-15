#!/usr/bin/env python
import os
import sys
import time
from sys import platform
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from openpose import *
import numpy as np

# dir_path = os.path.dirname(os.path.realpath(__file__))
# print dir_path
# if platform == "win32": sys.path.append(dir_path + '/../../python/openpose/');
# else: sys.path.append('../../python');
# # Option b
# # If you run `make install` (default path is `/usr/local/python` for Ubuntu), you can also access the OpenPose/python module from there. This will install OpenPose and the python library at your desired installation path. Ensure that this is in your python path in order to use it.
#sys.path.append('/usr/local/python')

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
# If GPU version is built, and multiple GPUs are available, set the ID here
params["num_gpu_start"] = 0
params["disable_blending"] = False
# Ensure you point to the correct path where models are located
params["default_model_folder"] = '/home/apg/openpose/models/'
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
        self.start = 0
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

        while not rospy.is_shutdown():
            if self.frame is not None:
                keypoints, output_image = openpose.forward(self.frame, True)
                #output_image = self.frame
                # Compute the time for this loop and estimate CPS as a running average
                end = time.time()
                duration = end - self.start
                fps = int(1.0 / duration)

                cv2.putText(output_image, "CPS: " + str(fps), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 0))

                # Update the image display
                cv2.imshow(self.node_name, output_image)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    return

    def image_callback(self, data):
        # Convert the ROS image to OpenCV format using a cv_bridge helper function
        self.start = time.time()
        self.frame = self.convert_image(data)


    def convert_image(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
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
        node_name = "Openpose"
        ROS2OpenCV2(node_name)
        #rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ros2opencv node."
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)