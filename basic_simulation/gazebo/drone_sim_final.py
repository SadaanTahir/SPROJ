import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from utils import *
import rospy
from sensor_msgs.msg import Image

global flag_found
flag_found = [False]*4

class FlagDetector:
    def __init__(self, number):
        self.id = number
        self.found = False
        self.bridge = CvBridge()
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
        cv2.imshow(f"Red Object Detection {self.id}", cv_image)
        cv2.waitKey(1)



def find_enclosing_rectangle(boundary_points, target_point):

    """
        Given drone location & all boundary edges/points after dividing original area into diff regions
        Returns 4 correct boundary points as long as the drone is not located on one of the boundaries,
        If drone is on the boundary, then it just returns 2 points (the start & end points of the same boundary)
    """

    x, y = target_point

    # Initialize min and max values
    min_x = float('inf')
    max_x = float('-inf')
    min_y = float('inf')
    max_y = float('-inf')

    # Find the closest boundary points in each direction
    for point in boundary_points:
        px, py = point

        if px <= x and px > max_x:
            max_x = px
        if px >= x and px < min_x:
            min_x = px

        if py <= y and py > max_y:
            max_y = py
        if py >= y and py < min_y:
            min_y = py

    # Return the four corner points of the enclosing rectangle
    return list(set([
        (min_x, min_y),
        (min_x, max_y),
        (max_x, max_y),
        (max_x, min_y)
    ]))

def divide_into_starting_points(boundary_points):
    """
        Given the number of regions divided & all boundary points after dividing original area into diff regions
        Returns starting points of all the drones, after moving them a little ahead of BOTTOM RIGHT of their respective region
    """
    rectangles = []
    # Extract unique x coordinates and y coordinates
    x_coords = set(point[0] for point in boundary_points)
    y_coords = set(point[1] for point in boundary_points)

    # Convert sets to sorted lists (optional)
    x_coords = sorted(x_coords)
    y_coords = sorted(y_coords)

    # Print the sorted x and y coordinates
    # print("Sorted x coordinates:", x_coords)
    # print("Sorted y coordinates:", y_coords)

    if y_coords:
        y_coords.pop()  # Removes the last element from y_coords
    if x_coords:
        x_coords.pop(0)  # Removes the first element from x_coords

    # print("Updated X coordinates:", x_coords)
    # print("Updated Y coordinates:", y_coords)

    THRESHOLD = 1.0 # initial distance from the boundary
    points = [(x, y) for x in x_coords for y in y_coords]
    for i,point in enumerate(points):
      px,py = point
      points[i] = (px - THRESHOLD, py + THRESHOLD)

    return points


def getboundary(division_lines,n):
  """
        Returns boundary points of the entire area, starting from initial 4 points of the rectangle
  """

  boundary_points = [(Minx,Miny), (Maxx,Maxy),(Minx,Maxy),(Maxx,Miny)]

  if n == 1:
    return boundary_points

  for i, line in enumerate(division_lines):
      p1, p2 = line
      boundary_points.append(p1)
      boundary_points.append(p2)

  return boundary_points


def validmove(pt,xmin,ymin,ymax):
  if pt[0] <= xmin or pt[1] >= ymax or pt[1] <= ymin:
    print("invalid move",pt)
    return False
  return True


def getnextmove(bdry, start, lastmove):

  # Get min and max values from the boundary
  min_x, min_y = bdry[0]
  max_x, max_y = bdry[0]
  for x, y in bdry[1:]:
      min_x = min(min_x, x)
      max_x = max(max_x, x)
      min_y = min(min_y, y)
      max_y = max(max_y, y)


  # print(start, bdry)

  if lastmove == "":
    print("SHOULD NEVER COME TO THIS POINT OF CODE")
    return "", start


  if lastmove == "left":
    start[1] -= 2
    if validmove(start,min_x,min_y,max_y):
      return "backward", tuple(start)
    else:
      start[1] +=2
      return "", tuple(start)

  elif lastmove == "right":
    start[1] +=2
    if validmove(start,min_x,min_y,max_y):
      return "forward", tuple(start)
    else:
      start[1] -=2
      return "", tuple(start)

  elif lastmove == "forward":
    if start[1] + 2 < max_y:
      start[1] +=2
      if validmove(start,min_x,min_y,max_y):
        return "forward", tuple(start)
      else:
        start[1] -=2
        return "", tuple(start)
    else:
      start[0]-=2
      if validmove(start,min_x,min_y,max_y):
        return "left", tuple(start)
      else:
        start[0] +=2
        return "", tuple(start)


  elif lastmove == "backward":
    if start[1] - 2 > min_y:
      start[1] -=2
      if validmove(start,min_x,min_y,max_y):
        return "backward", tuple(start)
      else:
        start[1] +=2
        return "", tuple(start)
    else:
      start[0]-=2
      if validmove(start,min_x,min_y,max_y):
        return "right", tuple(start)
      else:
        start[0] +=2
        return "", tuple(start)

  else:       #first move is a valid move
    start[1] +=2
    return "forward", tuple(start)


# def simultaneous_move(thismove, prev, connections, notmoving):  #move the drones simultaneously if they havent landed yet
#   with ThreadPoolExecutor(max_workers=len(connections)) as executor:
#     futures = [executor.submit(movewrtlocal, connections[i], thismove[i], prev[i]) for i,_ in enumerate(connections) if notmoving[i]==0]

def simultaneous_move(connections, notmoving, newcoords):  #move the drones simultaneously if they havent landed yet
  with ThreadPoolExecutor(max_workers=len(connections)) as executor:
    futures = [executor.submit(moveglobal, connections[i], newcoords[i][0], newcoords[i][1]) for i,_ in enumerate(connections) if notmoving[i]==0]


# def cvflagcheck():


def traverse(starting_points, division_lines, boundary_points, N):

  geofence = []
  start = []
  n = len(starting_points)

  connections = make_connections()
  coords = [[ -35.3632709,149.1653365],[-35.3631819,149.1653374],[-35.3631816,149.1652272],[-35.3632709,149.1652273]]  #lat,long
  handle_simultaneous_landing(connections,coords)
    
  for i,pt in enumerate(starting_points):
    bdry = find_enclosing_rectangle(boundary_points, pt)
    geofence.append(bdry)
    start.append(pt)
    print("drone",i, "with geofencing", bdry)

  thismove = [None] * 4
  notmoving = [0] * 4
  prev = [[0,0] for _ in range(4)]
  tmp = [[0,0] for _ in range(4)]
  newcoords = [[ -35.3632709,149.1653365],[-35.3631819,149.1653374],[-35.3631816,149.1652272],[-35.3632709,149.1652273]]  #lat,long
  # prev = [[7.126,13.6],[8.526,24.766],[-0.218,23.5],[-1.46,12.26]]
  # tmp = [[7.126,13.6],[8.526,24.766],[-0.218,23.5],[-1.46,12.26]]
  print("start ... ",start)
  # print("need to have separate moves for all drones, as some might stop to catch a flag while others may keep continuing")s


  rospy.init_node(f'drone_img_listener', anonymous=True)
  detectors = []
  for i in range(4):
    # detectors[i] = FlagDetector(number=i)  # Callback functions setup 
    detectors.append(FlagDetector(number=i))
  

  while True:
    # cvflagcheck()

    if "" == thismove[0] == thismove[1] == thismove[2] == thismove[3]:
      break

    for i,pt in enumerate(starting_points):   # update this drone's next location and its next move
      if flag_found[i] == True:
        if notmoving[i] == 0:
          notmoving[i] = 1
          print("stopping drone - found flag ",i)
          land(connections[i])
          connections[i].close()
        continue

      
      thismove[i] , start[i] = getnextmove(geofence[i], list(start[i]) , thismove[i]) 
      #updating prev local location of drone i
      print(thismove,prev)
      px,py = prev[i]
      oldlat, oldlon = newcoords[i]
      if thismove[i] == "forward":
        newcoords[i] = move_gps(oldlat, oldlon, 2, 270)
        px-=2
      elif thismove[i] == "backward":
        newcoords[i] = move_gps(oldlat, oldlon, 2, 90) 
        px+=2
      elif thismove[i] == "left":
        newcoords[i] = move_gps(oldlat, oldlon, 1, 180)
        py+=2
      elif thismove[i] == "right":
        newcoords[i] = move_gps(oldlat, oldlon, 1, 180)
        py-=2
      else:         #nomove
        notmoving[i] = 1
        print("stopping drone ",i)
        land(connections[i])
        connections[i].close()
        continue
      tmp[i] = [px,py]

    # simultaneous_move(thismove, prev, connections, notmoving)
    simultaneous_move(connections, notmoving, newcoords)
    for j,_ in enumerate(tmp):
      prev[j] = tmp[j]

  print("Simulation ending...")


if __name__ == "__main__":

  # Initial area bounds - works for rectangles/squares - currently hardcoded, will later fetch it from map
  Minx = -20
  Miny = -20
  Maxx =  20
  Maxy =  20

  for i in range(1,5):

    N = i    # Number of regions/drones (works for any positive int N )

    if i != 4:
      continue #works for only 4 DRONES

    rows = cols = 0
    if N == 1:     # Edge case: N = 1, no division needed
        print(f"N = 1")
        boundary_points = getboundary([],N)
        starting_points = divide_into_starting_points(boundary_points)
        traverse(starting_points,[], boundary_points,N)
        continue
    elif N % 2 == 0:
        rows = 2
        cols = N // 2
    else:
        rows = 0
        cols = N

    # print("\nN = ", N)

    # Calculate the starting and ending points of vertical/horizontal lines that will divide the region equally
    division_lines = []

    if rows == 2:                   # Compute horizontal line for the rows
        row_height = (Maxy - Miny) / rows
        y_line = Miny + row_height
        division_lines.append(((Minx, y_line), (Maxx, y_line)))  # Horizontal line

    col_width = (Maxx - Minx) / cols
    for i in range(1, cols):          # Compute vertical lines for the columns
        x_line = Minx + i * col_width
        division_lines.append(((x_line, Miny), (x_line, Maxy)))  # Vertical lines

    boundary_points = getboundary(division_lines,N)
    starting_points = divide_into_starting_points(boundary_points)
    traverse(starting_points,division_lines, boundary_points,N)


    """
        After initialising the drones to the BOTTOM RIGHT of their boundaries,
        We can now implement a basic traversal algorithm that moves the drone
        upwards till its boundary, turn left , then downwards till its boundary, turn right,
        AND REPEAT until it reaches the BOTTOM LEFT of its boundary or it finds the flag first.
        This will ensure geofencing of the drone as well as it will keep searching for the flag
    """