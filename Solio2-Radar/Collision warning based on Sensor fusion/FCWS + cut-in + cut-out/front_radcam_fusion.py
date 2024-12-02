import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Float32MultiArray, String
from conti_radar.msg import radar_obj
from datetime import datetime
import numpy as np

rospy.init_node('listener', anonymous=True)

a = []  # List to store radar data timestamps
b = []  # List to store camera data timestamps
i = 0   # Flag to indicate the presence of camera data
current_x = None
current_y = None  # Variable to store the current radar data
range = None
RCS = None
SNR = None
current_bounding_boxes = None  # Variable to store the current bounding boxes

# Load camera projection matrix
ndlt = np.load('/home/orin/Downloads/NDLT_matrix_solio_new.npy')

# Radius of the circles to be drawn
circle_radius = 4

# Color in (B, G, R) format for drawing circles
colors = (0, 0, 255)

def compute_world2img_projection(world_points, M, is_homogeneous=False):
    if not is_homogeneous:
        points_h = np.vstack((world_points[:3, :], np.ones(world_points.shape[1])))

    h_points_i = M @ points_h

    h_points_i[0, :] = h_points_i[0, :] / h_points_i[2, :]
    h_points_i[1, :] = h_points_i[1, :] / h_points_i[2, :]

    points_i = h_points_i[:2, :]

    return points_i

def callback(data):
    timestamp = datetime.now().strftime('%H:%M:%S.%f')
    # print(timestamp)
    a.append(timestamp)
    global current_x, current_y, vabsx, vabsy, arelx, arely, range
    current_x, current_y, vabsx, vabsy = data.f_DistX, data.f_DistY, data.f_VabsX, data.f_VabsY

def time_callback(msg):
    global b, i
    # rospy.loginfo("Received Timestamp for Image %s", msg.data)
    b.append(msg.data)

def bbox_callback(msg):
    global current_bounding_boxes
    list_of_lists = unflatten_data(msg.data)
    # rospy.loginfo("################Received data of bbox_callback: %s", list_of_lists)
    current_bounding_boxes = list_of_lists

def put_text_center(img, text, font_scale, color, thickness):
    # Get the size of the text
    text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]

    # Calculate the position to place the text at the center of the image
    text_x = (img.shape[1] - text_size[0]) // 2
    text_y = (img.shape[0] + text_size[1]) // 2

    # Put the text on the image
    cv2.putText(img, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)

def image_callback(msg):
    global cv_image, current_bounding_boxes, current_x, current_y, range

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    global check
    check = 0
    vehicle_cmd_pub = rospy.Publisher('/fcws_commands', String, queue_size=10)
    if cv_image is not None and current_y is not None:
        # Extract relevant radar data
        x = np.asarray(current_x)
        y = -np.asarray(current_y)
        range = np.sqrt(np.square(x) + np.square(y))
        velp = np.asarray(vabsx)
        velq = np.asarray(vabsy)
        vel = np.sqrt(np.square(velp) + np.square(velq))

        # Project radar points onto the image
        predictions = compute_world2img_projection(np.vstack((x, y, np.ones_like(x) * 0.78)), ndlt)
        predictions = np.round(predictions.T)

        # Iterate over each bounding box
        for class_name, x_min, y_min, x_max, y_max in current_bounding_boxes:
            x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)

            # Find the radar point with the shortest range within the bounding box
            min_range_index = None
            min_range = float('inf')
            for i, (px, py) in enumerate(predictions):
                px, py = int(px), int(py)
                if x_min <= px <= x_max and y_min <= py <= y_max:
                    if range[i] < min_range and -2.0 <= current_y[i] <= 2.0:   #road width
                        min_range = range[i]
                        min_range_index = i

            # If a radar point is found within the bounding box, display it
            if min_range_index is not None:
                px, py = predictions[min_range_index]
                r = range[min_range_index]
                r = r - 4.0
                v = vel[min_range_index]
                print("#########################",class_name)
                # Print information about the closest radar point
                print(f"Bounding Box: {class_name} ({x_min}, {y_min}) - ({x_max}, {y_max})")
                print(f"Closest Point: ({px}, {py}), Range: {r:.2f} m, Velocity: {v:.1f} m/s")
                v= v*3.6
                
                # Draw the bounding box on the image
                cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

                # Draw a circle and text for the closest radar point
                cv2.circle(cv_image, (int(px), int(py)), circle_radius, colors, -1)
                text = f"Range: {r:.2f} m"
                velocity = f"Velocity: {v: .1} kmph"
                cv2.putText(cv_image, text, (int(px), int(py) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors, 2)
                cv2.putText(cv_image, velocity, (int(px), int(py) +40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors, 2)
                if class_name == 0.0:
                    out = "PEDESTRIAN AHEAD"
                elif class_name == 1.0:
                    out = "BICYCLE AHEAD"
                elif class_name == 2.0:
                    out = "CAR AHEAD"
                elif class_name == 7.0:
                    out = "TRUCK AHEAD"
                else:
                    out = "Unknown Object Ahead"

                # Apply the condition for y value
                if 0 <= r <= 10:
                    text = out + " STOP"
                    vehicle_cmd_pub.publish("STOP")
                    put_text_center(cv_image, text, 1, (0, 0, 255), 2)
                    check = 1
                elif 10 < r <= 20:
                    text = out + " SLOW DOWN"
                    vehicle_cmd_pub.publish("SLOW")
                    put_text_center(cv_image, text, 1, (0, 0, 255), 2)
                    check = 2
                else:
                    text = out + " GO"
                    vehicle_cmd_pub.publish("GO")
                    put_text_center(cv_image, text, 1, (0, 255, 0), 2)

                # if 0 <= r <= 10:
                #     text = "VEHICLE AHEAD - STOP"
                #     vehicle_cmd_pub.publish("STOP")
                #     put_text_center(cv_image, text, 1, (0, 0, 255), 2)
                # elif 9 <= r <= 20:
                #     text = "VEHICLE AHEAD - SLOW DOWN"
                #     vehicle_cmd_pub.publish("SLOW DOWN")
                #     put_text_center(cv_image, text, 1, (0, 0, 255), 2)
                # else:
                #     vehicle_cmd_pub.publish("NO CHANGE")
        if check == 0:
            vehicle_cmd_pub.publish("GO")
        # Show the modified image
        cv2.imshow("Received image", cv_image)
        cv2.waitKey(1)

def unflatten_data(flattened_data):
    list_of_lists = []
    sublist = []
    for item in flattened_data:
        if item == -1:
            list_of_lists.append(sublist)
            sublist = []
        else:
            sublist.append(item)
    return list_of_lists

rospy.Subscriber("/radar_lrr_front_obj", radar_obj, callback)
rospy.Subscriber("/object_topic_front", Float32MultiArray, bbox_callback)
rospy.Subscriber("/time_topic_front", String, time_callback)
rospy.Subscriber("/basler_front", Image, image_callback)

rospy.spin()
