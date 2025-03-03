#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Initialize the CvBridge class
bridge = CvBridge()

def callback(data):
    try:
        # Convert the ROS Image message to an OpenCV image
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"Error converting image: {e}")
        return

    # Convert the image to the HSV color space
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Define the range of red color in HSV (two ranges to handle red wrapping around HSV space)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    # Create two masks for the red color
    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

    # Combine the two masks
    mask = mask1 + mask2

    # Perform morphological operations to remove noise from the mask
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw bounding boxes around detected red objects
    for contour in contours:
        if cv2.contourArea(contour) > 500:  # Only consider large enough areas
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            rospy.loginfo("Red Object Found")

    # Display the result in a minimized window
    cv2.namedWindow("Red Object Detection for Drone 1", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Red Object Detection for Drone 1", 960, 720)  # Minimized window size
    cv2.imshow("Red Object Detection for Drone 1", cv_image)
    cv2.waitKey(1)

def listener():
    # Initialize the ROS node
    rospy.init_node('drone_img_listener', anonymous=True)

    # Subscribe to the drone's camera topic (adjust the topic if needed)
    rospy.Subscriber("/chase_cam/camera/image_raw", Image, callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    listener()

