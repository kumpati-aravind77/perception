import pyzed.sl as sl
import cv2
import numpy as np
import torch
from ultralytics import YOLO
import math
import threading
import rospy
from std_msgs.msg import String,Float32,Int32
from multiprocessing import Queue

def main(frame_queue, flags_queue,ped_intent_queue,sb_queue):

# Initialize the ROS node for the algorithm
    rospy.init_node("DL_models", anonymous=True)

    # Create publisher for intention decision
    pedestrian_intention = rospy.Publisher('/intention',  String, queue_size=10)

    # Initialize empty lists
    detections1 = []
    detections2 = []
    CW_detections3 = []
    SB_detections4 = []

    # Load YOLO models onto GPU
    model1 = YOLO('/home/s186/Desktop/models_checkpoint/traffic_light_v8_small_200epochs.pt')
    model2 = YOLO('/home/s186/Desktop/models_checkpoint/Traffic_sign_Yolov8.pt')
    model3 = YOLO('/home/s186/Desktop/models_checkpoint/collision_warning_yolov8.pt')
    model4 = YOLO('/home/s186/Desktop/models_checkpoint/SB_PT.pt')
    model5 = YOLO('/home/s186/Desktop/models_checkpoint/ped_intent.pt')

    # # Load YOLO models onto GPU
    # model1 = YOLO('models_checkpoint/traffic_light_v8_small_200epochs.pt').cuda()
    # model2 = YOLO('models_checkpoint/Traffic_sign_Yolov8.pt').cuda()
    # model3 = YOLO('models_checkpoint/collision_warning_yolov8.pt').cuda()
    # model4 = YOLO('models_checkpoint/SB_PT.pt').cuda()
    # model5 = YOLO('models_checkpoint/ped_intent.pt').cuda()

    # Load YOLO models onto GPU
    # model1 = YOLO('/home/suzuki/Desktop/Solio-S186/models_checkpoint/traffic_light_v8_small_200epochs.engine')
    # model2 = YOLO('/home/suzuki/Desktop/Solio-S186/models_checkpoint/Traffic_sign_Yolov8.engine')
    # model3 = YOLO('/home/suzuki/Desktop/Solio-S186/models_checkpoint/collision_warning_yolov8.engine')
    # model4 = YOLO('/home/suzuki/Desktop/Solio-S186/models_checkpoint/SB_PT.engine')
    # model5 = YOLO('/home/suzuki/Desktop/Solio-S186/models_checkpoint/ped_intent.engine')
    #Traffic Light
    traffic_light_type_pub = rospy.Publisher('Traffic_light_type', String, queue_size=10)
    traffic_light_depth_pub = rospy.Publisher('Traffic_light_depth', Float32, queue_size=10)
    #Traffic sign
    traffic_sign_type_pub = rospy.Publisher('Traffic_sign_type', String, queue_size=10)
    traffic_sign_depth_pub = rospy.Publisher('Traffic_sign_depth', Float32, queue_size=10)
    #CW
    cw_type_pub = rospy.Publisher('CW_type', String, queue_size=10)
    cw_depth_pub = rospy.Publisher('CW_depth', Float32, queue_size=10)
    cw_side_dist_pub = rospy.Publisher('CW_side_dist', Float32, queue_size=10)
    cw_angle_pub = rospy.Publisher('CW_angle', Float32, queue_size=10)
    #SpeedBump & Pothole
    SB_Pothole_type = rospy.Publisher('SB_Pothole_type',String,queue_size=10)
    SB_Pothole_depth = rospy.Publisher('SB_Pothole_depth',Float32,queue_size=10)
    #Pedestrian Intention
    pedestrian_intention = rospy.Publisher('/intention',  String, queue_size=10)

    # Serial number of the ZED camera
    serial_number = 36659489
    #serial_number = 35633216
    #serial_number = 32088047

    # Create a ZED Camera object
    zed = sl.Camera()

    # Configure initialization parameters

    init_params = sl.InitParameters()
    # init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # Highest QUALITY
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init_params.depth_maximum_distance = 40
    init_params.set_from_serial_number(serial_number)  # Set the serial number
    print("30 fps, HD720, Neural Depth Mode, Coordinate System: RIGHT_HANDED_Z_UP_X_FWD, Depth Max Distance: 40m")
    print("Parameters Setting done....")

    # Open the ZED camera
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera")
        exit(1)

    left_image = sl.Mat()
    point_cloud = sl.Mat()

    # Zed's Oject Class <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    image_left_tmp = sl.Mat()

    positional_tracking_parameters = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(positional_tracking_parameters)
    obj_param = sl.ObjectDetectionParameters()
    obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
    obj_param.enable_tracking = True
    zed.enable_object_detection(obj_param)
    objects = sl.Objects()
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()

    def xywh2abcd(xywh, im_shape):
        output = np.zeros((4, 2))
        # Center / Width / Height -> BBox corners coordinates
        x_min = (xywh[0] - 0.5*xywh[2]) #* im_shape[1]
        x_max = (xywh[0] + 0.5*xywh[2]) #* im_shape[1]
        y_min = (xywh[1] - 0.5*xywh[3]) #* im_shape[0]
        y_max = (xywh[1] + 0.5*xywh[3]) #* im_shape[0]
        # A ------ B
        # | Object |
        # D ------ C
        output[0][0] = x_min
        output[0][1] = y_min
        output[1][0] = x_max
        output[1][1] = y_min
        output[2][0] = x_min
        output[2][1] = y_max
        output[3][0] = x_max
        output[3][1] = y_max
        return output

    def detections_to_custom_box(detections, im0):
        output = []
        for i, det in enumerate(detections):
            xywh = det.xywh[0]
            # Creating ingestable objects for the ZED SDK
            obj = sl.CustomBoxObjectData()
            obj.bounding_box_2d = xywh2abcd(xywh, im0.shape)
            obj.label = det.cls
            obj.probability = det.conf
            obj.is_grounded = False
            output.append(obj)
        return output

    def torch_thread(image_net, model ,detections):
        print("Initializing Network...")
        img = cv2.cvtColor(image_net, cv2.COLOR_BGRA2RGB)
        det = model.predict(img, classes = [0], save=False, imgsz=416, conf=0.6, iou=0.45)[0].cpu().numpy().boxes
        # ZED CustomBox format (with inverse letterboxing tf applied)
        detections.extend(detections_to_custom_box(det, image_net))
        # sleep(0.01)

    #function for processing all the objects for ROI
    def check_object_crossing(obj_data, ped_intent_queue):
        state = obj_data["action_state"]  # IDLE or MOVING : string output
        position = obj_data["position"]  # x, y, z : coordinates
        # print("position:", position)
        # print("state:", state)

        x = position[0]
        y = position[1]

        YELLOW_COLOR = (0, 255, 255)
        RED_COLOR = (0, 0, 255)
        GREEN_COLOR = (0, 255, 0)
        WHITE_COLOR = (255, 255, 255)
        GREY_COLOR = (128, 128, 128)

        # Determine the pedestrian's crossing state
        if 1.2 >= y >= -1.2:
            if x <= 9.5:
                color = RED_COLOR
                crossing_state = 'Crossing'
            elif 9.5 < x <= 14.0:
                color = YELLOW_COLOR
                crossing_state = 'Intend_of_Crossing'
            else:
                color = GREEN_COLOR
                crossing_state = 'Not_Crossing_X'
        elif (1.2 < y <= 3.0) or (-1.2 > y >= -3.0):
            if x <= 11.0:
                color = YELLOW_COLOR
                crossing_state = 'Intend_of_Crossing'
            else:
                color = (255, 255, 255)
                crossing_state = 'Not_Crossing_X'
        else:
            color = (255, 255, 255)
            crossing_state = 'Not_Crossing_X'

        # print(f"Pedestrian state: {crossing_state}")
        print(x,y)
    
    # Add the color to the queue if it's not full
        if not ped_intent_queue.full():
            ped_intent_queue.put(color)
    
        return crossing_state

    # Camera pose
    cam_w_pose = sl.Pose()
    #------------------------------------------------------------

    # Define Model Execution Functions
    def model_inference(frame, model):
        results = model.predict(frame, conf=0.55)
        return results[0].boxes

    # Define Model Execution Functions, just for particular classes
    def collisionWarning_inference_specific(frame, model):
        results = model.predict(frame, conf=0.7, classes=[1,2,3,5,6,7, 10, 12, 14, 15, 19, 20, 21])
        # results = model.predict(frame, conf=0.5, classes=[0, 1, 2, 3, 5, 6, 12, 13, 14, 15, 19, 20, 21, 25])
        return results[0].boxes

    classes = ['Person', 'Bicycle', 'Car', 'Motorcycle', 'Route board', 'Bus', 'Commercial vehicle', 'Truck', 'Traffic sign', 'Traffic light', 'Autorickshaw', 'Stop sign', 'Ambulance', 'bench', 'construction vehicle', 'Animal', 'unmarked speed bump', 'marked speed bump', 'pothole', 'Police vehicle', 'Tractor', 'Pushcart', 'temporary traffic barrier', 'rumblestrips', 'traffic cone', 'Zebra crossing']


    # cv2.namedWindow("Perception - CollisionWarning + Pedestrian Intention + SB/Pothole + Traffic Light & Sign", cv2.WINDOW_NORMAL)
    # cv2.setWindowProperty("Perception - CollisionWarning + Pedestrian Intention + SB/Pothole + Traffic Light & Sign", 1024,768) 
    # cv2.moveWindow("Perception - CollisionWarning + Pedestrian Intention + SB/Pothole + Traffic Light & Sign", 250, 0)


    # Function to perform inference in a separate thread
    def inference_thread(frame, model, detections_list):
        detections_list.extend(model_inference(frame, model))

    def traffic_light_label_mapper(int_label):
         # input type: integer value of the labels
         label_map = {0: 'Go',1: 'Stop',2: 'Stop Left',3: 'Go Left',4: 'Warning',5: 'WarningLeft'}
         # return type: string
         return label_map[int_label]

    def SpeedBump_pothole_label_mapper(int_label,sb_queue):
         # input type: integer value of the labels
         label_map = {0: 'Pothole', 1: 'Speed Bump'}
         # return type: string
         if not sb_queue.full():
            sb_queue.put(label_map[int_label])
         return label_map[int_label]

    def traffic_sign_label_mapper(int_label):
         # input type: integer value of the labels
         label_map = {0: 'Traffic signal-c', 1: 'u-turn prohibited-m', 2: 'Stop', 3: 'round about-c', 4: 'bus stop-I', 5: 'informatory board-I', 6: 't-Junction-c', 7: 'right Reverse Bend-c', 8: 'right Hand Curve-c', 9: 'no Entry-m', 10: 'give way-m', 11: 'pedestrian Crossing-c', 12: 'no Parking-m', 13: 'speed limit-m', 14: 'compulsary turn left ahead-m', 15: 'restrictions ends', 16: 'horn prohibited-m', 17: 'side road left-c', 18: 'side road right-c', 19: 'school-zone-c', 20: 'speed bump-c', 21: 'right turn prohibited-m', 22: 'left hand curve-c', 23: 'u-turn', 24: 'children at play-c', 25: 'y-junction-c', 26: 'parking lot bike-I', 27: 'parking lot car-I', 28: 'parking lot cycles-I', 29: 'parking lot-I', 30: 'hump or rough road-c', 31: 'chevron sign right', 32: 'national highway route marker-I', 33: 'chevron sign left', 34: 'gap in median-c', 35: 'school ahead-c', 36: 'cross road-c', 37: 'merging traffic from left-c', 38: 'go slow-c', 39: 'overtaking prohibited-m', 40: 'narrow road ahead-c', 41: 'staggered intersection-c', 42: 'men at work-c', 43: 'two way-c', 44: 'accident prone zone-c', 45: 'compulsory keep left-m', 46: 'truck prohibited-m', 47: 'left turn prohibited-m', 48: 'direction sign-I', 49: 'no stopping or no standing-m', 50: 'one way-m', 51: 'all vehicles prohibited-m', 52: 'falling rocks-c', 53: 'left hair pin bend-c', 54: 'right hair pin bend-c', 55: 'narrow bridge-c', 56: 'quaryside or river bank-c', 57: 'steep ascent-c', 58: 'steep descent-c', 59: 'compulsary turn right ahead-m', 60: 'priority for oncoming traffic-m', 61: 'pedestrians prohibited-m', 62: 'compulsory ahead or turn right-m', 63: 'hospital-I', 64: 'load limit-m', 65: 'parking lot auto riskshaw-I', 66: 'cattle-c', 67: 'pass either side-m', 68: 'compulsory horn-m', 69: 'petrol pump-I', 70: 'guarded level crossing-c'}
         # return type: string
         return label_map[int_label]

    def nearest_traffic_light_OR_sign(lights, depth):
        # if there are no traffic-signs in the frame
        # print(lights, depth)
        if not lights:
            print("No Traffic Light or Sign in the frame")
            return ("",0.0)
        else:
            light_depth_dict = dict(zip(lights, depth))
            # Sort the new dictionary in ascending order by values
            sorted_tuples = {k: light_depth_dict[k] for k, v in sorted(light_depth_dict.items(), key=lambda item: item[1])}
            sorted_list = [(k, v) for k, v in sorted_tuples.items()]
            return sorted_list[0] # Traffi-light Class (string), Depth (float)

    def process_collision_warning(CW_objects, distance_to_object, side_distance, angle_to_object):
        # if there are no objects in the frame
        # print(objects, distance_to_object)
        # print(f"CW Objects: {CW_objects}, Distances: {distance_to_object}")
        if not CW_objects:
            print("No CW objects in the frame")
            return ("",0.0, 0.0, 0.0)
        else:
            object_depth_angle_side_dict = dict(zip(CW_objects, zip(distance_to_object, angle_to_object, side_distance)))
            # Sort the new dictionary in ascending order by values
            sorted_tuples = {k: object_depth_angle_side_dict[k] for k, v in sorted(object_depth_angle_side_dict.items(), key=lambda item: item[1][0])}
            sorted_list = [(k, v[0], v[1], v[2]) for k, v in sorted_tuples.items()]
            return sorted_list[0] # Object Class (string), Depth (float), Angle (float), Side Distance (float)

    def collision_warning_boundingbox_color(type, CW_distance, CW_side,flags_queue):
        CW_distance = float(CW_distance)
        CW_side = float(CW_side)
        # CW_angle = float(CW_angle)

        YELLOW_COLOR = (0, 255, 255)
        RED_COLOR = (0, 0, 255)
        GREEN_COLOR = (0, 255, 0)
        GREY_COLOR = (128, 128, 128)

        side_distance = 1.1
        detecting_distance = 35
        stop_distance = 10
        caution_distance = 20 

        if CW_distance < detecting_distance:
            if abs(CW_side) <= (side_distance + 0.5) and stop_distance <= CW_distance <= caution_distance:
                color = YELLOW_COLOR     # YELLOW
                print("C3-distance")
                # store_collision_warning([type,CW_distance, CW_side, "YELLOW"])
                #return color
            elif abs(CW_side) <= side_distance and 0.5 <= CW_distance < stop_distance:
                color = RED_COLOR    # RED
                print("C4-stop")
                # store_collision_warning([type,CW_distance, CW_side, "RED"])
                #return color
            else:
                color = GREEN_COLOR
                print("C5-GO")
                # store_collision_warning([type,CW_distance, CW_side, "GREEN"])
                #return color    # GREEN
        else:   
            color = GREEN_COLOR
            print("C5-else")
        #     store_collision_warning([type,CW_distance, CW_side, "GREEN"])
        # # Put the determined color into the flags_queue if it's not full
        if not flags_queue.full():
            flags_queue.put(color)
        
        return color    # GREEN 

    # def store_collision_warning(warning):
    #     with open('collision_warnings-case', 'a') as f:
    #         f.write(str(warning) + '\n')


    def nearest_speed_bump_or_pothole(SB_Pothole_objects, distance_to_object):
        # if there are no objects in the frame
        # print(objects, distance_to_object)
        if not SB_Pothole_objects:
            print("No SB objects in the frame")
            return ("",0.0)
        else:
            object_depth_dict = dict(zip(SB_Pothole_objects, distance_to_object))
            # Sort the new dictionary in ascending order by values
            sorted_tuples = {k: object_depth_dict[k] for k, v in sorted(object_depth_dict.items(), key=lambda item: item[1])}
            sorted_list = [(k, v) for k, v in sorted_tuples.items()]
            return sorted_list[0] # Object Class (string), Depth (float)
    # Function to perform inference in a separate thread just for particular classes
    def inference_thread_specific(frame, model, detections_list):
        detections_list.extend(collisionWarning_inference_specific(frame, model))

    # Main loop for tracking
    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(left_image, sl.VIEW.LEFT)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU)

            # Convert the image to a format suitable for YOLO
            image_data = left_image.get_data()
            image_data = image_data[:, :, :3]
            img1 = image_data.copy()

            # Execute models on GPU using threading
            threads = []
            detections1 = []
            detections2 = []
            CW_detections3 = []
            SB_detections4 = []
            detections5=[]#<<<<<<<<<<<<<

            t1 = threading.Thread(target=inference_thread, args=(img1, model1, detections1))
            t2 = threading.Thread(target=inference_thread, args=(img1, model2, detections2))
            t3 = threading.Thread(target=inference_thread_specific, args=(img1, model3, CW_detections3))
            t4 = threading.Thread(target=inference_thread, args=(img1, model4, SB_detections4))
            t5 = threading.Thread(target=torch_thread, args=(img1,model5,detections5))#<<<<<<<<<

            threads.extend([t1, t2, t3, t4, t5])

            for thread in threads:
                thread.start()

            for thread in threads:
                thread.join()

            lights = []
            depth = []

            # -- Ingest detections<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            zed.ingest_custom_box_objects(detections5)
            zed.retrieve_objects(objects, obj_runtime_param)
            #---------------------------------------------------
            for box in detections1:
                # Draw bounding boxes around detected objects for model 1
                coords = torch.round(box.xyxy).to(torch.int).tolist()[0]
                cls_type = torch.round(box.cls).to(torch.int).tolist()[0]
                confidence = box.conf.tolist()
                conf = float(confidence[0]*100)
                conf = "{:.1f}".format(conf)

                x1, y1, x2, y2 = coords
                cls = traffic_light_label_mapper(cls_type)
                center_x = round((x1 + x2) / 2)
                center_y = round((y1 + y2) / 2)

                # Point cloud value at the center of the bounding box
                err, point_cloud_value = point_cloud.get_value(center_x, center_y)
                center_3d_x = float("{:.1f}".format(point_cloud_value[0]))
                center_3d_y = float("{:.1f}".format(point_cloud_value[1]))
                center_3d_z = float("{:.1f}".format(point_cloud_value[2]))
                try:
                    distance = round(math.sqrt(pow(center_3d_x, 2) + pow(center_3d_y, 2) ))    #+ pow(center_3d_z, 2)), 2)
                except:
                    center_3d_x = 50.0
                    center_3d_y = 50.0
                    distance = round(math.sqrt(pow(center_3d_x, 2) + pow(center_3d_y, 2) ))
                    # time.sleep(3)

                text_pos = (x1, y1 - 8)
                depth_1 = "{:.1f}".format(distance)
                if depth_1 == "inf":
                    continue
                # text = cls + ":" + str(depth_1) + " m"
                # text = cls + "(" + conf + ")"
                text = cls

                color = (255,255,0)       # BLUE
                lights.append(cls)
                depth.append(distance)
                # cv2.putText(img1, text, text_pos, cv2.FONT_HERSHEY_PLAIN, 2.5, color, 3)
                # cv2.rectangle(img1, (x1, y1), (x2, y2), color, 2)

            publish_light, publish_depth = nearest_traffic_light_OR_sign(lights, depth)
            if publish_depth != 0.0:
                print(f"TF: {publish_light}T, {publish_depth}D")
            traffic_light_type_pub.publish(publish_light)
            traffic_light_depth_pub.publish(publish_depth)


            signs = []
            depth_sign = []
            for box in detections2:
                # Draw bounding boxes around detected objects for model 2
                coords = torch.round(box.xyxy).to(torch.int).tolist()[0]
                cls_type = torch.round(box.cls).to(torch.int).tolist()[0]
                confidence = box.conf.tolist() 
                conf = float(confidence[0]*100)
                conf = "{:.1f}".format(conf)

                x1, y1, x2, y2 = coords
                cls = traffic_sign_label_mapper(cls_type)
                center_x = round((x1 + x2) / 2)
                center_y = round((y1 + y2) / 2)

                # Point cloud value at the center of the bounding box
                err, point_cloud_value = point_cloud.get_value(center_x, center_y)
                center_3d_x = point_cloud_value[0]
                center_3d_y = point_cloud_value[1]
                center_3d_z = point_cloud_value[2]

                distance = round(math.sqrt(pow(center_3d_x, 2) + pow(center_3d_y, 2) + pow(center_3d_z, 2)), 2)
                text_pos = (x1, y1 - 8)
                depth_1 = "{:.1f}".format(distance)
                if depth_1 == "inf":
                    continue
                # text = cls + ":" + str(depth_1) + " m"
                text = cls
                color = (255, 255, 255)      # WHITE
                signs.append(cls)
                depth_sign.append(distance)
                cv2.putText(img1, text, text_pos, cv2.FONT_HERSHEY_PLAIN, 2.5, color, 3)
                cv2.rectangle(img1, (x1, y1), (x2, y2), color, 2)
            publish_sign, publish_depth_sign = nearest_traffic_light_OR_sign(signs, depth_sign)
            if publish_depth_sign != 0.0:
                print(f"TS: {publish_sign}T, {publish_depth_sign}D")
            traffic_sign_type_pub.publish(publish_sign)
            traffic_sign_depth_pub.publish(publish_depth_sign)


            cw_objects = []
            distance_to_object = []
            side_distance_to_object = []
            angle_to_object = []
            for box in CW_detections3:
                # Draw bounding boxes around detected objects for model 3
                # print(CW_detections3)
                coords = torch.round(box.xyxy).to(torch.int).tolist()[0]
                cls_type = torch.round(box.cls).to(torch.int).tolist()[0]
                confidence = box.conf.tolist()
                conf = float(confidence[0]*100)
                conf = "{:.1f}".format(conf)

                x1, y1, x2, y2 = coords
                cls = str(cls_type)

                class_name = classes[cls_type]

                center_x = round((x1 + x2) / 2)
                center_y = round((y1 + y2) / 2)

                # Point cloud value at the center of the bounding box
                err, point_cloud_value = point_cloud.get_value(center_x, center_y)
                center_3d_x = point_cloud_value[0]
                center_3d_y = point_cloud_value[1]
                center_3d_z = point_cloud_value[2]

                distance = round(math.sqrt(pow(center_3d_x, 2) + pow(center_3d_y, 2) + pow(center_3d_z, 2)), 1)
                sideDistance = round(center_3d_y,1)
                angle = math.degrees(math.atan2(center_3d_x, center_3d_y))
                angle = "{:.0f}".format(angle)

                text_pos = (x1, y1 - 30)
                depth_1 = "{:.1f}".format(distance)
                sideDistance = "{:.1f}".format(sideDistance)
                sideDistance = float(sideDistance)

                if depth_1 == "inf":
                    continue
                text1 = class_name
                # text1 = class_name + "(" + conf +"%)"
                # text2 = "Depth:" + str(depth_1) + " m | Angle:" + str(angle) + " deg"  
                # text2 = "D:" + str(depth_1) + ", S:" + str(sideDistance)   
                # store_collision_warning([class_name, distance, sideDistance, angle])
                text2 = "D:" + str(depth_1)
                if cls_type==0:
                    text2=""
                    text1=""
                color = collision_warning_boundingbox_color(class_name, depth_1, sideDistance,flags_queue) 

                cw_objects.append(class_name)
                distance_to_object.append(distance)
                side_distance_to_object.append(sideDistance)
                angle_to_object.append(angle)
                cv2.putText(img1, text1, text_pos, cv2.FONT_HERSHEY_PLAIN, 2.5, color, 3)
                cv2.putText(img1, text2, (x1, y1 -8), cv2.FONT_HERSHEY_PLAIN, 1.5, color, 2)
                cv2.rectangle(img1, (x1, y1), (x2, y2), color, 2)

            publish_cw, publish_depth_cw, publish_angle_cw, publish_side_dist_cw = process_collision_warning(cw_objects, distance_to_object, side_distance_to_object, angle_to_object)
            publish_angle_cw = float(publish_angle_cw)

            print(f"CW: {publish_cw}(T), {publish_depth_cw}FD, {publish_side_dist_cw}SD, {publish_angle_cw}A")
            cw_type_pub.publish(publish_cw)
            cw_depth_pub.publish(publish_depth_cw)
            cw_side_dist_pub.publish(publish_side_dist_cw)
            cw_angle_pub.publish(publish_angle_cw)


            SB_Pothole_objects = []
            distance_to_sb_pothole = []
            for box in SB_detections4:
                # Draw bounding boxes around detected objects for model 4
                coords = torch.round(box.xyxy).to(torch.int).tolist()[0]
                cls_type = torch.round(box.cls).to(torch.int).tolist()[0]
                confidence = box.conf.tolist()
                conf = float(confidence[0]*100)
                conf = "{:.1f}".format(conf)


                x1, y1, x2, y2 = coords
                cls = SpeedBump_pothole_label_mapper(cls_type,sb_queue)         
                center_x = round((x1 + x2) / 2)
                center_y = round((y1 + y2) / 2)

                # Point cloud value at the center of the bounding box
                err, point_cloud_value = point_cloud.get_value(center_x, center_y)
                center_3d_x = point_cloud_value[0]
                center_3d_y = point_cloud_value[1]
                center_3d_z = point_cloud_value[2]

                distance = round(math.sqrt(pow(center_3d_x, 2) + pow(center_3d_y, 2) + pow(center_3d_z, 2)), 2)
                text_pos = (x1, y1 - 8)
                depth_1 = "{:.2f}".format(distance)
                if depth_1 == "inf":
                    continue

                text1 = str(cls) #+ "(" + conf +"%)"

                color = (255, 165, 0)    # ORANGE

                SB_Pothole_objects.append(cls)
                distance_to_sb_pothole.append(distance)
                cv2.putText(img1, text1, text_pos, cv2.FONT_HERSHEY_PLAIN, 2.5, color, 3)
                cv2.rectangle(img1, (x1, y1), (x2, y2), color, 2)

            publish_SB_Pothole, publish_depth_SB_Pothole = nearest_speed_bump_or_pothole(SB_Pothole_objects, distance_to_sb_pothole)
            if publish_depth_SB_Pothole != 0.0:
                print(f"SB: {publish_SB_Pothole}T, {publish_depth_SB_Pothole}D")
            SB_Pothole_type.publish(publish_SB_Pothole)
            SB_Pothole_depth.publish(publish_depth_SB_Pothole)

            # Pedestrian Intention Part<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            cross=0
            intend_to_cross = 0 
            # not_cross =0
            if objects.is_new:
                obj_array = objects.object_list
                # print("object array:", obj_array)
                # frame_object_data = {}  # Dictionary to store object information for this frame
                if len(obj_array) > 0:
                    i = 0
                    crossing_intents= []

                    for obj in obj_array:# iterating over all the objects in the frame   
                        obj_data = {}  # Dictionary to store individual object data
                        position = obj.position
                        # print("position:",position)
                        # depth = -position[2]  # Depth is the negative of the third coordinate of the position
                        # obj_data["depth"] = depth
                        obj_data["Tracking ID"] = int(obj.id)
                        obj_data["action_state"] = repr(obj.action_state)# IDLE or MOVING : string output
                        obj_data["position"] = obj.position
                        obj_data["euclid"] = np.nan if np.isnan(obj.position).any() else np.linalg.norm(obj.position)
                        obj_data["velocity"] = obj.velocity
                        if obj.mask.is_init():
                            obj_data["2D_mask_available"] = True
                        else:
                            obj_data["2D_mask_available"] = False
                        # frame_object_data[depth] = obj_data
                        bounding_box_2d = obj.bounding_box_2d
                        i += 1
                        x1, y1 = int(bounding_box_2d[0][0]), int(bounding_box_2d[0][1])
                        x2, y2 = int(bounding_box_2d[2][0]), int(bounding_box_2d[2][1])

                        # Finding the intention of present pedestrian
                        # intention = check_object_crossing(obj, repr(obj.action_state))
                        intention = check_object_crossing(obj_data,ped_intent_queue)
                        if intention == "Crossing":
                            crossing_intents.append(obj_data)
                        # print(intention)
                        obj_data["Intention"] = intention

                        # Determine intention and color code accordingly ; BGR format
                        if intention == 'Crossing':
                            cross = cross +1
                            color = (0, 0, 255)  # Light Red for crossing; reserved Red for Pedestrian which will be crossing
                        elif intention == 'Intend_of_Crossing':
                            intend_to_cross = intend_to_cross + 1
                            color = (0, 255, 255)  # Yellow for intending to cross
                        elif intention == 'Not_Crossing_X':
                            # not_cross = not_cross + 1
                            color = (255,255,255) # white for not_crossing_x
                        # elif intention == 'Not_in_ROI':
                        else:
                            color = (0, 0, 0)  # Black for not in ROI 

                        # Draw the bounding box
                        cv2.rectangle(img1, (x1, y1), (x2, y2), color, 2) 
                        # Label the bounding box with information
                        cv2.putText(img1, "State :"+repr(obj.action_state), (x1, y1 - 40), cv2.FONT_HERSHEY_PLAIN, 1, color, 1)
                        cv2.putText(img1, "Intention :" + str(intention) , (x1, y1 - 20), cv2.FONT_HERSHEY_PLAIN, 1.5, color, 2)
                        # cv2.putText(img1, "Position: x:" + "{:.2f}".format(obj.position[0]) +
                        #                 ", y:" + "{:.2f}".format(obj.position[1]) + ", z:" + "{:.2f}".format(obj.position[2]), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        # cv2.putText(img1, "Velocity: Vx:" + "{:.2f}".format(obj.velocity[0]) +
                                        # ", Vy:" + "{:.2f}".format(obj.velocity[1]) + ", Vz:" + "{:.2f}".format(obj.velocity[2]), (x1, y1 - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    # Print info of all pedestrians who are trying to cross
                    # print("Crossing Intent array:",crossing_intents) 

            # Navigation Decision
            if cross>0:
                print("Stop Vehicle")
                pedestrian_intention.publish("Crossing")
            elif intend_to_cross>0:
                print("Slow Down Vehicle")
                pedestrian_intention.publish("Intend_To_Cross")
            else:
                print("Go Vehicle")
                pedestrian_intention.publish("Not_Crossing")

            if not frame_queue.full():
                frame_queue.put(img1)

    # Close the ZED camera
    zed.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    import sys
    from multiprocessing import Queue
    frame_queue = Queue(maxsize=100)
    flags_queue = Queue(maxsize=1) 
    ped_intent_queue=Queue(maxsize=1)
    sb_queue=Queue(maxsize=1)
    # traffic_light_queue=Queue(maxsize=1)
    main(frame_queue,flags_queue,ped_intent_queue,sb_queue)

