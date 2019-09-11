#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
# from cv_detection.color_detection import color_detection
from std_msgs.msg import Bool
# from cv_detection import feature_detection
# from cv_detection import detect_window


d = detect_window()
f = feature_detection()



c = color_detection("red")
img_topic = "/bebop/image_raw"#"/usb_cam/image_raw"#
bridge = CvBridge()
rospy.init_node('test_ros_image', anonymous=True)
ref_pub = rospy.Publisher('/control/align_reference/ref_point', Point, queue_size=1)


 
# Set up tracker.
# Instead of MIL, you can also use
 
tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'CSRT', 'MOSSE']
tracker_type = tracker_types[4]

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

if int(minor_ver) < 3:
    tracker = cv2.Tracker_create(tracker_type)
else:
    if tracker_type == 'BOOSTING':
        tracker = cv2.TrackerBoosting_create()
    if tracker_type == 'MIL':
        tracker = cv2.TrackerMIL_create()
    if tracker_type == 'KCF':
        tracker = cv2.TrackerKCF_create()
    if tracker_type == 'TLD':
        tracker = cv2.TrackerTLD_create()
    if tracker_type == 'MEDIANFLOW':
        tracker = cv2.TrackerMedianFlow_create()
    if tracker_type == 'CSRT':
        tracker = cv2.TrackerCSRT_create()
    if tracker_type == 'MOSSE':
        tracker = cv2.TrackerMOSSE_create()

    
tracking_count = 0
bbox = (287, 23, 86, 320)


def img_callback(data):
    
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    (rows, cols, channels) = cv_image.shape
    if not (cols > 60 and rows > 60):  # returns if data have unvalid shape
        return
    # print("ok")
    # frame = cv_image.copy()
    
    # feature(cv_image)
    # track(cv_image)
    # color(cv_image)
    d.update(cv_image)
    k = cv2.waitKey(1) & 0xff
    # if k == 27 : break
def feature(frame):
    f.get_rect(frame)

def color(cv_image):
    center,radius,rect = c.get_rect(cv_image)
    if center != None:
        
        img_crop, img_rot = c.crop_rect(cv_image,rect)
        p = Point()
        p.x = center[0]
        p.y = center[1]
        p.z = radius
        print(radius)
        if radius > 30: 
            ref_pub.publish(p)   

def track(frame):
    global tracking_count, bbox

    tracking_count += 1
    if tracking_count == 1:
        bbox = cv2.selectROI(frame, False)
        ok = tracker.init(frame, bbox)
    # Start timer
    timer = cv2.getTickCount()

    # Update tracker
    ok, bbox = tracker.update(frame)

    # Calculate Frames per second (FPS)
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);

    # Draw bounding box
    if ok:
        # Tracking success
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
    else :
        # Tracking failure
        cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

    # Display tracker type on frame
    cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
    
    # Display FPS on frame
    cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);

    # Display result
    cv2.imshow("Tracking", frame)


print("init")

image_sub = rospy.Subscriber(
            img_topic, Image, img_callback, queue_size=None)
# aligned_sub = rospy.Subscriber("/control/align_reference/aligned", Bool, take_box, queue_size=1)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
cv2.destroyAllWindows()

