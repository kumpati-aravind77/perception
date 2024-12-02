#!/usr/bin/env python3

import cv2
import os
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Import the String message type
from std_msgs.msg import Float32MultiArray  # Import the Float32MultiArray message type
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pypylon import pylon
from datetime import datetime
from ultralytics import YOLO
import math

# Initialize YOLO model
model = YOLO("yolov8s.pt")
classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]
# classNames = ["person", "bicycle", "car", "motorbike", "truck"]


# conecting to the first available camera
#camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
# Specify the serial number of the camera you want to read from


# Connect to the specified camera
target_serial_number = "24442428"


tl_factory = pylon.TlFactory.GetInstance()
device_info_list = tl_factory.EnumerateDevices()
print(len(device_info_list))
for device_info in device_info_list:
    print(device_info.GetSerialNumber())
    if device_info.GetSerialNumber() == target_serial_number:
        camera = pylon.InstantCamera(tl_factory.CreateDevice(device_info))
        print("Inside the target serial number ", target_serial_number)
        break
else:
    
        raise ValueError("Camera with serial numbe not found.")


# Connect to camera
#camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
# camera.AcquisitionFrameRate.SetValue(30)  # Set frame rate to 30 H
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
d = camera.ResultingFrameRateAbs.Value
print(d)
# camera.AcquisitionFrameRate.SetValue(30)  # Set frame rate to 30 H
converter = pylon.ImageFormatConverter()                   
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

# Initialize the ROS node
rospy.init_node('image_publisher_front', anonymous=True)

# Create a publisher with the appropriate topic and message type for the image

image_pub = rospy.Publisher('basler_front', Image, queue_size=10)
bridge = CvBridge()

# Create a publisher for bounding box details
bbox_pub = rospy.Publisher('object_topic_front', Float32MultiArray, queue_size=10)
# Create a publisher for the timestamp
time_pub = rospy.Publisher("time_topic_front", String, queue_size=10)

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height
    return img_msg

def flatten_bbox_details(bbox_details):
    flattened_data = []
    for cls, x1, y1, x2, y2 in bbox_details:
        flattened_data.extend([cls, x1, y1, x2, y2, -1])  # Add separator -1
    return flattened_data

while camera.IsGrabbing():
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
 
    if grabResult.GrabSucceeded():
        # Access the image data
        image = converter.Convert(grabResult)
        img = image.GetArray()
        img = cv2.resize(img, (640,480), interpolation=cv2.INTER_AREA)

        # Convert the image to a ROS message
        ros_image = cv2_to_imgmsg(img)

        # Publish the image message
        image_pub.publish(ros_image)

        # Publish the timestamp
        timestamp = datetime.now().strftime("%H:%M:%S.%f")
        time_pub.publish(String(timestamp))

        # Perform object detection
        results = model(img, stream=True)

        # Prepare bounding box details
        bbox_details = []
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                conf = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])
                if cls==0 or cls==1 or cls==2 or cls==3 or cls==7:
                    class_name = classNames[cls]
                    bbox_details.append([cls, x1, y1, x2, y2])

                    # Draw bounding box on image
                    cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
                    label = f'{class_name}{conf}'
                    t_size = cv2.getTextSize(label, 0, fontScale=1, thickness=2)[0]
                    c2 = x1 + t_size[0], y1 - t_size[1] - 3
                    cv2.rectangle(img, (x1, y1), c2, [255, 0, 255], -1, cv2.LINE_AA)  # filled
                    cv2.putText(img, label, (x1, y1 - 2), 0, 1, [255, 255, 255], thickness=1, lineType=cv2.LINE_AA)

        # Flatten and publish bounding box details
        flattened_bbox_details = flatten_bbox_details(bbox_details)
        bbox_pub.publish(Float32MultiArray(data=flattened_bbox_details))

        # Display image with bounding boxes
        cv2.imshow("Image", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    grabResult.Release()

# Release resources
camera.StopGrabbing()
cv2.destroyAllWindows()
