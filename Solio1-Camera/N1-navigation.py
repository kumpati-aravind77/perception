''' 
    Navigation using GNSS data and Perception data    
    Last Author: Rishav KUMAR(Mtech AI - ai22mtech12003@iith.ac.in) 
'''

import math
import os
import logging
import datetime
import socket
import time
import novatel_oem7_msgs
import numpy as np
import rospy
from novatel_oem7_msgs.msg import BESTPOS, BESTVEL, INSPVA
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String, Int32

# Define the MABX (vehicle control controller) IP address and port for sending data
MABX_IP = "192.168.50.1"
MABX_PORT = 30000

# Define buffer size and local interface
BUFFER_SIZE = 4096
LOCAL_INTERFACE = "eth0"

# Conversion factor for latitude and longitude to meters
LAT_LNG_TO_METER = 1.111395e5

# Flasher dictionary
FLASHER_DICT = {"None": 0, "Left": 1, "Right": 2, "Both": 3}

# Initialize the ROS node for the algorithm
rospy.init_node("GNSS_navigation", anonymous=True)

# Initialize the UDP socket for MABX communication
mabx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mabx_addr = (MABX_IP, MABX_PORT)

# Initialize global variables
CW_type, TL_type, TS_type, SB_pothole = "", "", "", ""
CW_distance, CW_side = 0.0, 0.0
TL_distance, TS_distance, SB_pothole_distance = 0.0, 0.0, 0.0
lat, lng, last_cw_stop, last_cw_slow = 0, 0, None, None
pedestrian_intention = "Not_Crossing"   # initially no pedestrian is present

def setup_logging(file_path):
    """Setup logging configuration."""
    log_dir = os.path.expanduser("~/Desktop/Solio-ADAS/Solio-Suzuki/navigation/logs")
    logFileName = os.path.splitext(os.path.basename(file_path))[0]
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    log_format = "%(asctime)s [%(levelname)s]: %(message)s"
    logging.basicConfig(level=logging.DEBUG, format=log_format)
    current_time = datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
    log_filename = f"{logFileName}-{current_time}.log"
    log_path = os.path.join(log_dir, log_filename)
    file_handler = logging.FileHandler(log_path)
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(logging.Formatter(log_format))
    logger = logging.getLogger("")
    logger.addHandler(file_handler)
    return logger

def log_and_print(message):
    """Log and print a message."""
    logger.info(message)
    print(message)

def get_coordinates(file_path):
    """Parse and retrieve coordinates from a file."""
    coordinates_list = []
    try:
        with open(file_path, "r") as file:
            for line in file:
                try:
                    coordinates = [float(coord) for coord in line.strip().strip("[],").split(",")]
                    coordinates_list.append(coordinates)
                except ValueError:
                    print(f"Error: Unable to convert coordinates in line '{line}' to float.")
    except FileNotFoundError:
        print(f"Error: The file '{file_path}' could not be found.")
    except Exception as e:
        print(f"An error occurred: {e}")
    return coordinates_list


def ROS_collision_warning_type(data):
    global CW_type
    CW_type = data.data

def ROS_collision_warning_distance(data):
    global CW_distance
    CW_distance = data.data

def ROS_collision_warning_side(data):
    global CW_side
    CW_side = data.data

def ROS_traffic_light_type(data):
    global TL_type
    TL_type = data.data

def ROS_traffic_light_distance(data):
    global TL_distance
    TL_distance = data.data

def ROS_traffic_sign_type(data):
    global TS_type
    TS_type = data.data

def ROS_traffic_sign_distance(data):
    global TS_distance
    TS_distance = data.data

def ROS_SB_pothole(data):
    global SB_pothole
    SB_pothole = data.data

def ROS_SB_pothole_distance(data):
    global SB_pothole_distance
    SB_pothole_distance = data.data

def ROS_pedestrian_intention(data):
    global pedestrian_intention
    pedestrian_intention = data.data

# Subscribers for the ROS topics (Perception)
rospy.Subscriber("/CW_type", String, ROS_collision_warning_type)
rospy.Subscriber("/CW_depth", Float32, ROS_collision_warning_distance)
rospy.Subscriber("/CW_side_dist", Float32, ROS_collision_warning_side)
rospy.Subscriber("/SB_Pothole_type", String, ROS_SB_pothole)
rospy.Subscriber("/SB_Pothole_depth", Float32, ROS_SB_pothole_distance)
rospy.Subscriber("/Traffic_light_type", String, ROS_traffic_light_type)
rospy.Subscriber("/Traffic_light_depth", Float32, ROS_traffic_light_distance)
rospy.Subscriber("/Traffic_sign_type", String, ROS_traffic_sign_type)
rospy.Subscriber("/Traffic_sign_depth", Float32, ROS_traffic_sign_distance)
rospy.Subscriber("/intention", String, ROS_pedestrian_intention)

def callback_velocity(data):
    """Callback function to handle velocity data."""
    global current_vel
    current_vel = 3.6 * data.hor_speed  # Convert m/s to km/h

def callback_heading(data):
    """Callback function to handle heading data."""
    global heading
    heading = data.azimuth

def callback_latlng(data):
    """Callback function to handle latitude and longitude data."""
    global lat, lng, lat_delta, lng_delta
    lat = data.lat
    lng = data.lon
    lat_delta = data.lat_stdev
    lng_delta = data.lon_stdev

def callback_gnss_imu(data):
    """Callback function to handle GNSS IMU data."""
    global acc_z
    acc_z = data.linear_acceleration.z


rospy.Subscriber("/novatel/oem7/bestvel", BESTVEL, callback_velocity)
rospy.Subscriber("/novatel/oem7/inspva", INSPVA, callback_heading)
rospy.Subscriber("/novatel/oem7/bestpos", BESTPOS, callback_latlng)
rospy.Subscriber("/imu/data_raw", Imu, callback_gnss_imu)

time.sleep(0.1)

def set_angle(current_angle, angle_change):
    """Calculate and set steering angle based on current angle and target angle."""
    gear_ratio = 17.75
    current_angle = max(min(current_angle, 40 * gear_ratio), -40 * gear_ratio)
    scaled_angle = (current_angle / gear_ratio - (-65.536)) / 0.002
    high_angle_byte, low_angle_byte = int(scaled_angle) >> 8, int(scaled_angle) & 0xFF
    return high_angle_byte, low_angle_byte

def calc_checksum(message_bytes):
    """Calculate checksum for the message bytes."""
    checksum = sum(message_bytes) & 0xFF
    return (0x00 - checksum) & 0xFF

def set_speed(speed):
    """Set speed for the vehicle."""
    speed = speed * 128
    high_byte_speed = int(speed) >> 8
    low_byte_speed = int(speed) & 0xFF
    return high_byte_speed, low_byte_speed

def get_speed(high_byte_speed, low_byte_speed):
    """Get speed from the high and low bytes."""
    speed = ((high_byte_speed << 8) | low_byte_speed) / 128
    return speed

def send_message_to_mabx(speed, current_angle, delta_angle, flasher_light, message_counter):
    """Send message to MABX."""
    H_Angle, L_Angle = set_angle(current_angle, -1 * delta_angle)
    H_Speed, L_Speed = set_speed(speed)
    message_bytes = [
        1, message_counter, 0, 1, 52, 136, 215, 1, H_Speed, L_Speed,
        H_Angle, L_Angle, 0, flasher_light, 0, 0, 0, 0
    ]
    message_bytes[2] = calc_checksum(message_bytes)
    message = bytearray(message_bytes)
    log_and_print("=================================")
    return message

def calculate_steer_output(currentLocation, Current_Bearing):
    """Calculate steer output based on current location and bearing."""
    global wp
    RAD_TO_DEG_CONVERSION = 57.2957795
    STEER_GAIN = 800 # For Tight Turns 1200 can be used or 900 in general
    off_y = -currentLocation[0] + waypoints[wp][0]
    off_x = -currentLocation[1] + waypoints[wp][1]
    target_bearing = 90.00 + math.atan2(-off_y, off_x) * RAD_TO_DEG_CONVERSION
    if target_bearing < 0:
        target_bearing += 360.00
    Current_Bearing = heading
    while Current_Bearing is None:
        Current_Bearing = heading
    Current_Bearing = float(Current_Bearing)
    bearing_diff = Current_Bearing - target_bearing
    if bearing_diff < -180:
        bearing_diff += 360
    elif bearing_diff > 180:
        bearing_diff -= 360
    if abs(bearing_diff) < 2:
        STEER_GAIN = 250
    elif abs(bearing_diff) > 20:
        STEER_GAIN = 1300
    steer_output = STEER_GAIN * np.arctan(-1 * 2 * 3.5 * np.sin(np.radians(bearing_diff)) / 8)
    # print(f"Steer GAIN : {STEER_GAIN} | Bearing Diff: {bearing_diff:.0f}")
    return steer_output, bearing_diff

def calculate_bearing_difference_for_speed_reduction(currentLocation, Current_Bearing):
    """Calculate bearing difference for speed reduction before Turning."""
    global wp
    RAD_TO_DEG_CONVERSION = 57.2957795
    next_wp = 7
    if wp + next_wp < wp_len:
        off_y = -currentLocation[0] + waypoints[wp + next_wp][0]
        off_x = -currentLocation[1] + waypoints[wp + next_wp][1]
    else:
        off_y = -currentLocation[0] + waypoints[wp][0]
        off_x = -currentLocation[1] + waypoints[wp][1]
    target_bearing = 90.00 + math.atan2(-off_y, off_x) * RAD_TO_DEG_CONVERSION
    if target_bearing < 0:
        target_bearing += 360.00
    Current_Bearing = heading
    while Current_Bearing is None:
        Current_Bearing = heading
    Current_Bearing = float(Current_Bearing)
    future_bearing_diff = Current_Bearing - target_bearing
    if future_bearing_diff < -180:
        future_bearing_diff += 360
    elif future_bearing_diff > 180:
        future_bearing_diff -= 360
    return future_bearing_diff

def process_collision_warning(CW_type, CW_distance, CW_side):
    """Process collision warning."""
    side_distance = 1.1
    detecting_distance = 35
    stop_distance = 10
    caution_distance = 20
    if CW_distance < detecting_distance:
        if abs(CW_side) <= side_distance and stop_distance <= CW_distance <= caution_distance:
            return "SLOW"
        elif abs(CW_side) <= side_distance and 1 <= CW_distance < stop_distance:
            return "STOP"
        else:
            return "GO"
    return "GO"

def process_traffic_light(TL_type, TL_distance):
    """Process traffic light."""
    threshold_distance = 21
    if TL_type == "Stop" and TL_distance < threshold_distance:
        return "STOP"
    else:
        return "GO"
    
def process_speedbump(SB_pothole, SB_distance):
    """Speed Bump"""
    sb_threshold_distance = 17
    if 1 <= SB_distance < sb_threshold_distance and SB_pothole !="Pothole":
        return "SLOW"
    else:
        return "GO"
    

def navigation_output(latitude, longitude, Current_Bearing):
    """Main navigation output function."""
    global counter, speed, wp, const_speed
    global CW_type, CW_distance, CW_side, TL_type, TL_distance, last_cw_slow, last_cw_stop, perceptionFlag
    perceptionFlag = False
    counter = (counter + 1) % 256
    log_and_print(f"2D Standard Deviation (in cm): {100 * lat_delta:.2f} cm")
    print("---------- Perception INPUT -----------")
    print(f"Collision Warning: {CW_type} ({CW_distance:.1f})")
    print(f"Pedestrian Intention: {pedestrian_intention}")
    print(f"SB+pothole: {SB_pothole} ({SB_pothole_distance:.1f})") 
    print(f"Traffic Light: {TL_type} ({TL_distance:.1f})")
    print(f"Traffic Sign: {TS_type} ({TS_distance:.1f})")
    print("-------------------------------------")
    
    currentLocation = [latitude, longitude]
    distance_to_final_waypoint = (np.linalg.norm(np.array(currentLocation) - waypoints[-1]) * LAT_LNG_TO_METER)
    
    if distance_to_final_waypoint > 1 and wp < wp_len:
        steer_output, bearing_diff = calculate_steer_output(currentLocation, Current_Bearing)
        # print(f"Steer Output: {steer_output:.2f} | Bearing Difference: {bearing_diff:.2f}")
        steer_output *= -1.0
        future_bearing_diff = calculate_bearing_difference_for_speed_reduction(currentLocation, Current_Bearing)
        #print(f"Testing Future Bearing : {future_bearing_diff:.0f}")
        if wp < 10 or wp +10 > wp_len :
            const_speed = turning_factor * speed
        sustain_time = 1.0
        

        if process_collision_warning(CW_type, CW_distance, CW_side) == "STOP" or process_traffic_light(TL_type, TL_distance) == "STOP" or pedestrian_intention=="Crossing":
            print("$ACTUATION: CW/TF/PI STOP")
            const_speed = 0
            perceptionFlag = True
            last_cw_stop = time.time()
        elif last_cw_stop is not None and abs(time.time() - last_cw_stop) < sustain_time:
            print("$ACTUATION$ : CW/TF/PI STOP++")
            const_speed = 0
            perceptionFlag = True
        elif process_collision_warning(CW_type, CW_distance, CW_side) == "SLOW" or pedestrian_intention=="Intend_To_Cross" or process_speedbump(SB_pothole, SB_pothole_distance) == "SLOW":
            print("$ACTUATION$ : CW/PI/SB SLOW")
            const_speed = min(turning_factor * speed, 7)
            if process_speedbump(SB_pothole, SB_pothole_distance) == "SLOW":
                const_speed = min(turning_factor * speed, 5)
            perceptionFlag = True
            last_cw_slow = time.time()
        elif last_cw_slow and time.time() - last_cw_slow < sustain_time+2:
            print("$ACTUATION$ : CW/PI/SB SLOW++")
            perceptionFlag = True
            const_speed = min(turning_factor * speed, 8)
        elif abs(bearing_diff) > 5 and not perceptionFlag:
            print("$ACTUATION$ : Turning")
            const_speed = max(turning_factor * speed, 9)
        elif abs(future_bearing_diff) > 8 and not perceptionFlag:
            print("$ACTUATION$ : Approaching Turn")
            const_speed = 9
        else:
            # if not perceptionFlag:
            const_speed = speed
            print(f"$ACTUATION$ : Going Straight")
        
        
        distance_to_nextpoint = (np.linalg.norm(np.array(currentLocation) - waypoints[wp]) * LAT_LNG_TO_METER)
        log_and_print(f"{wp} out of {wp_len} | Next Coordinate distance : {distance_to_nextpoint:.1f} m")
        if wp < wp_len and distance_to_nextpoint < LOOK_AHEAD_DISTANCE:
            wp += 1
    else:
        log_and_print(f"----- FINISHED  -----")
        log_and_print(f"Brake Activated")
        steer_output = 0
        const_speed = 0
        message = send_message_to_mabx(const_speed, steer_output, 0, FLASHER_DICT["Both"], counter)
        mabx_socket.sendto(message, mabx_addr)
    # log_and_print(f"Current Speed (MABX): {const_speed:.0f} kmph")

    if current_vel is not None and current_vel >= 2:
        log_and_print(f"Current Speed (GNSS): {current_vel:.0f} kmph")
    try:
        message = send_message_to_mabx(const_speed, steer_output, 0, FLASHER_DICT["Both"], counter)
        mabx_socket.sendto(message, mabx_addr)
    except Exception as e:
        log_and_print(f"Error sending message to MABX: {e}")

def mainLoop():
    """Main loop for navigation."""
    while not rospy.is_shutdown():
        try:
            log_and_print(f"Current Coordinate No. : {wp}")
            log_and_print(" ")
            latitude = float(lat)
            longitude = float(lng)
            Current_Bearing = float(heading)
            navigation_output(latitude, longitude, Current_Bearing)
            time.sleep(SLEEP_INTERVAL / 1000)
        except ValueError as ve:
            log_and_print(f"ValueError occurred: {ve}")
        except IOError as ioe:
            log_and_print(f"IOError occurred: {ioe}")
        except KeyboardInterrupt:
            log_and_print("Autonomous Mode is terminated manually!")
            message = send_message_to_mabx(0, 0, 0, 0, counter)
            mabx_socket.sendto(message, mabx_addr)
            raise SystemExit
        except Exception as e:
            log_and_print(f"An error occurred: {e}")

if __name__ == "__main__":
    global speed, reduction_factor, steer_output, counter, wp, file_path

    #file_path = "/home/suzuki/Desktop/Solio-ADAS/Solio-Suzuki/Final_Demo/waypoints/waypoints---TB2M.txt"
    file_path = "/home/suzuki/Desktop/Solio-ADAS/Solio-Suzuki/Final_Demo/waypoints/waypoints---MG2TB.txt"
    # file_path = "/home/suzuki/Desktop/Solio-S186/waypoints/waypoints-vehicledynamics.txt"


    logger = setup_logging(file_path)
    logger.info("Main Code Starting")
    SLEEP_INTERVAL = 100
    LOOK_AHEAD_DISTANCE = 3
    speed = 18
    turning_factor = 0.6
    wp = 0
    steer_output = 0
    counter = 0
    waypoints = get_coordinates(file_path)
    wp_len = len(waypoints)
    mainLoop()
