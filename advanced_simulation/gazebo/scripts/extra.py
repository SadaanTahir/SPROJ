import threading
import time
import rospy
from sensor_msgs.msg import Image, NavSatFix


def image_callback(msg, drone_id):
    print("img-callback",msg)


def global_position_callback(msg, drone_id):
    print("global-pos",msg)

# Example functions to run concurrently
def function_one():
    rospy.init_node('yolo_v8_multi_drone_detector', anonymous=True)
    rospy.Subscriber("/chase_cam/camera/image_raw", Image, image_callback, callback_args=1)
    rospy.Subscriber("/drone1/mavros/global_position/global", NavSatFix, global_position_callback, callback_args=1)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
    rospy.spin()

def function_two():
    for i in range(1000):
        print(f"Function Two: {i}")
        time.sleep(1)  # Simulate work with a sleep

# Create threads
thread1 = threading.Thread(target=function_one)
thread2 = threading.Thread(target=function_two)

# Start threads
thread1.start()
thread2.start()

# Wait for both threads to complete
thread1.join()
thread2.join()

print("Both functions have completed.")
