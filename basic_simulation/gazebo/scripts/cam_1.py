#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class FlagDetector:
    def __init__(self, number):
        self.id = number
        self.found = False
        self.bridge = CvBridge()
        rospy.init_node(f'drone_img_listener_{self.id}', anonymous=True)
        if self.id == 0:
            self.sub = rospy.Subscriber(f"/chase_cam/camera/image_raw", Image, self.callback)
        else:
            self.sub = rospy.Subscriber(f"/chase_cam_{self.id}/camera_{self.id}/image_raw", Image, self.callback)


    def callback(self, data):
        if self.found:
            print("should-never-execute-this-part-of-callback".upper())
            return
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define the range of red color in HSV
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        
        # Create a mask to filter out only the red regions
        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
        
        # For better detection, define another range for the red color
        lower_red = np.array([170, 120, 70])
        upper_red = np.array([180, 255, 255])
        
        mask2 = cv2.inRange(hsv_image, lower_red, upper_red)
        
        # Combine the two masks
        mask = mask1 + mask2
        
        # Perform morphological operations to remove noise from the mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw bounding boxes around the detected red objects
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Only consider large enough areas
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                print("Flag Detected")
                self.found = True
                flag_found[self.id] = True
                cv2.destroyAllWindows()
                self.sub.unregister()
                return

        
        # Display the result in a minimized window
        cv2.namedWindow("Red Object Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Red Object Detection", 320, 240)  # Set the window size to be minimized
        cv2.imshow("Red Object Detection", cv_image)
        cv2.waitKey(1)

    def smth():
        print("smth")


obj = FlagDetector(0)
obj.smth()

# detectors = []
# for i in range(4):
#     detectors[i] = FlagDetector(number=i)  # Pass a number to the class constructor