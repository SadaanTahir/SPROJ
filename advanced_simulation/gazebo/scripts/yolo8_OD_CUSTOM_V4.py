import rospy
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from geopy.distance import distance
import math
import os
import sys
import csv

model = YOLO('/home/sadaan/Desktop/sim2/marker2by2_dataset/runs/detect/train/weights/best.pt')
bridge = CvBridge()

drone_positions = {
    1: {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0},
    2: {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0},
    3: {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0},
    4: {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0}
}

detection_flags = {1: False, 2: False, 3: False, 4: False}

csv_file = 'marker_cordinates.csv'

if not os.path.exists(csv_file):
    with open(csv_file, mode='w') as file:
        writer = csv.writer(file)
        writer.writerow(["Drone_ID", "Marker_Latitude", "Marker_Longitude"])

def move_gps(lat, lon, meters, dir):
    start_point = (lat, lon)
    new_point = distance(meters=meters).destination(start_point, dir)
    return [new_point.latitude, new_point.longitude]

def global_position_callback(msg, drone_id):
    # rospy.loginfo(f"Updating position for drone {drone_id}")
    drone_positions[drone_id]['latitude'] = msg.latitude
    drone_positions[drone_id]['longitude'] = msg.longitude
    drone_positions[drone_id]['altitude'] = msg.altitude

def image_callback(msg, drone_id):
    try:
        # rospy.loginfo(f"Processing image for drone {drone_id}")
        sys.stdout = open(os.devnull, 'w')
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        results = model(cv_image)
        sys.stdout = sys.__stdout__

        img_height, img_width, _ = cv_image.shape
        horizontal_fov = math.radians(0.5)
        focal_length = (img_width / 2) / math.tan(horizontal_fov / 2)

        for result in results:
            for box in result.boxes:
                conf = box.conf.item()
                if conf > 0.8:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls = int(box.cls)
                    label = f"{model.names[cls]} {conf:.2f}"

                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    marker_x = (center_x - img_width / 2) * drone_positions[drone_id]['altitude'] / focal_length
                    marker_y = (center_y - img_height / 2) * drone_positions[drone_id]['altitude'] / focal_length

                    marker_latitude, marker_longitude = move_gps(
                        drone_positions[drone_id]['latitude'], drone_positions[drone_id]['longitude'], marker_y, 0
                    )
                    marker_latitude, marker_longitude = move_gps(
                        marker_latitude, marker_longitude, marker_x, 90
                    )

                    rospy.loginfo(f"Drone {drone_id}: Marker detected at Latitude {marker_latitude}, Longitude {marker_longitude}")

                    with open(csv_file, mode='r') as file:
                        rows = list(csv.reader(file))

                    with open(csv_file, mode='w') as file:
                        writer = csv.writer(file)
                        updated = False

                        for row in rows:
                            if row[0] == str(drone_id):
                                row[1] = marker_latitude
                                row[2] = marker_longitude
                                updated = True
                            writer.writerow(row)

                        if not updated:
                            writer.writerow([drone_id, marker_latitude, marker_longitude])

    except Exception as e:
        rospy.logerr(f"Error processing image from drone {drone_id}: {e}")

def print_detection_status():
    for drone_id in range(1, 5):
        detection_status = "YES" if detection_flags[drone_id] else "NO"
        rospy.loginfo(f"Drone {drone_id} - Marker Detection: {detection_status}")
    rospy.loginfo("\n")

def main():
    rospy.init_node('yolo_v8_multi_drone_detector', anonymous=True)
    rospy.Subscriber("/chase_cam/camera/image_raw", Image, image_callback, callback_args=1)
    rospy.Subscriber("/chase_cam_1/camera_1/image_raw", Image, image_callback, callback_args=2)
    rospy.Subscriber("/chase_cam_2/camera_2/image_raw", Image, image_callback, callback_args=3)
    rospy.Subscriber("/chase_cam_3/camera_3/image_raw", Image, image_callback, callback_args=4)
    rospy.Subscriber("/drone1/mavros/global_position/global", NavSatFix, global_position_callback, callback_args=1)
    rospy.Subscriber("/drone2/mavros/global_position/global", NavSatFix, global_position_callback, callback_args=2)
    rospy.Subscriber("/drone3/mavros/global_position/global", NavSatFix, global_position_callback, callback_args=3)
    rospy.Subscriber("/drone4/mavros/global_position/global", NavSatFix, global_position_callback, callback_args=4)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print_detection_status()
        rate.sleep()
        
    rospy.spin()

if __name__ == '__main__':
    main()
