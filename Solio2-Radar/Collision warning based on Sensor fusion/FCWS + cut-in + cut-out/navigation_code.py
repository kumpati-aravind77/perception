
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
from std_msgs.msg import Float32, String

# Define the MABX(vehicle control controller) IP address and port for sending data
mabx_IP = "192.168.50.1"
mabx_PORT = 30000

# Define buffer size and local interface
BUFFER_SIZE = 4096
local_interface = "eth0"

# Flasher dictionary
FLASHER_DICT = {"None": 0, "Left": 1, "Right": 2, "Both": 3}

# Conversion factor for latitude and longitude to meters
LAT_LNG_TO_METER = 1.111395e5
CW_flag = 0
radar_flag = ""


# Initialize the ROS node for the algorithm
rospy.init_node("GNSS_navigation", anonymous=True)

# Initialize the UDP socket for MABX communication
mabx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mabx_addr = (mabx_IP, mabx_PORT)


def setup_logging(file_path):
    log_dir = os.path.expanduser(
        "~/Desktop/Solio-ADAS/Solio-Suzuki/navigation/Development/devLogs"
    )
    base_filename = os.path.basename(file_path)
    # Remove the file extension to get the desired string
    logFileName = os.path.splitext(base_filename)[0]

    # Create the log directory if it does not exist
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # Set up the logging format
    log_format = "%(asctime)s [%(levelname)s]: %(message)s"
    logging.basicConfig(level=logging.DEBUG, format=log_format)

    # Create a log file with the current timestamp as the name
    current_time = datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
    log_filename = f"{logFileName}-{current_time}.log"
    log_path = os.path.join(log_dir, log_filename)

    # Add a file handler to save logs to the file
    file_handler = logging.FileHandler(log_path)
    file_handler.setLevel(logging.DEBUG)

    # Set the log format for the file handler
    file_handler.setFormatter(logging.Formatter(log_format))

    # Add the file handler to the logger
    logger = logging.getLogger("")
    logger.addHandler(file_handler)

    return logger


def log_and_print(str):
    logger.info(str)
    print(str)


# Function to parse and retrieve coordinates from a file
def get_coordinates(file_path):
    coordinates_list = []
    try:
        with open(file_path, "r") as file:
            for line in file:
                try:
                    coordinates = [
                        float(coord) for coord in line.strip().strip("[],").split(",")
                    ]
                    coordinates_list.append(coordinates)
                except ValueError:
                    # Handle the exception if a value cannot be converted to float
                    print(
                        f"Error: Unable to convert coordinates in line '{line}' to float."
                    )
    except FileNotFoundError:
        # Handle the exception if the file is not found
        print(f"Error: The file '{file_path}' could not be found.")
    except Exception as e:
        # Handle any other unexpected exceptions
        print(f"An error occurred: {e}")

    return coordinates_list


def callback_flag(data):
    global CW_flag
    CW_flag = 0
    CW_flag = data.data

def callback_radar(data):
    global radar_flag
    radar_flag = data.data


    
rospy.Subscriber("/vehicle_commands", String, callback_radar)



# Callback functions for handling GNSS data
def callback_velocity(data):
    global current_vel
    current_vel = 3.6 * data.hor_speed


def callback_heading(data):
    global heading
    heading = (
        data.azimuth
    )  # Left-handed rotation around z-axis in degrees clockwise from North.


def callback_latlng(data):
    global lat, lng, lat_delta, lng_delta
    lat = data.lat
    lng = data.lon
    lat_delta = data.lat_stdev
    lng_delta = data.lon_stdev


def callback_gnss_imu(data):
    global acc_z
    acc_z = data.linear_acceleration.z

rospy.Subscriber("/novatel/oem7/bestvel", BESTVEL, callback_velocity)
rospy.Subscriber("/novatel/oem7/inspva", INSPVA, callback_heading)
rospy.Subscriber("/novatel/oem7/bestpos", BESTPOS, callback_latlng)
rospy.Subscriber("/imu/data_raw", Imu, callback_gnss_imu)

time.sleep(0.1)


# Function to calculate and set steering angle based on current angle and target angle
def set_angle(current_angle, angle_change):
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
    speed = speed * 128
    high_byte_speed = (int)(speed) >> 8
    low_byte_speed = (int)(speed) & 0xFF
    return high_byte_speed, low_byte_speed


def get_speed(high_byte_speed, low_byte_speed):
    speed = (high_byte_speed << 8) | low_byte_speed
    speed = speed / 128
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


def navigation_output(latitude, longitude, Current_Bearing):
    global counter, speed, wp, CW_flag, radar_flag, saw_pothole, const_speed, pothole_flag, acc_z, frame_count, pot_time, hit_time
    # pothole_flag == 0
    flasher = 3  # 0 None, 1 Left, 2 Right, 3 Both ; For Indicator
    counter = (counter + 1) % 256

    # log_and_print(f"Current :- Latitude: {latitude} , Longitude: {longitude}")
    log_and_print(f"2D Standard Deviation(in cms): {100*lat_delta:.2f} cm")

    currentLocation = [latitude, longitude]
    distance_to_final_waypoint = (np.linalg.norm(np.array(currentLocation) - waypoints[-1]) * LAT_LNG_TO_METER)
   
    if (distance_to_final_waypoint > 1 and wp < wp_len):  # to check if the final point is not less than 1m

        steer_output, bearing_diff = calculate_steer_output(currentLocation, Current_Bearing)
        steer_output *= -1.0

        future_bearing_diff = calculate_bearing_difference_for_speed_reduction(
            currentLocation, Current_Bearing
        )
        if wp < 10 or wp +10 > wp_len :
            const_speed = turning_factor * speed
        sustain_time = 1.0
        # next_bearing_diff = bearing_diff - future_bearing_diff
        # log_and_print(f"Future & Current Bearing diff : {next_bearing_diff:.1f}")

        const_speed = speed

        #if wp < 10 or wp_len - 10 < wp:  # Slow start and end in the waypoints
        #    const_speed = turning_factor * speed

        if abs(bearing_diff) > 5:
            const_speed = max(turning_factor * speed, 9)
            log_and_print(f"Curve Speed from code : {const_speed:.0f} kmph")

        if radar_flag == "STOP":
            const_speed = 0
        elif radar_flag == "SLOW":
            const_speed = turning_factor * speed
        else:
            const_speed = speed

        distance_to_nextpoint = (np.linalg.norm(np.array(currentLocation) - waypoints[wp]) * LAT_LNG_TO_METER)
        log_and_print(f"{wp} out of {wp_len} | Next Coordinate distance : {distance_to_nextpoint:.1f} m")  # For Testing
        if wp < wp_len and distance_to_nextpoint < LOOK_AHEAD_DISTANCE:
                wp += 1
    else:
        log_and_print(f"----- FINISHED  -----")
        log_and_print(f"Brake Activated")
        steer_output = 0
        const_speed = 0
        message = send_message_to_mabx(const_speed, steer_output, 0, FLASHER_DICT["Both"], counter)
        mabx_socket.sendto(message, mabx_addr)


    if current_vel is not None and current_vel >= 2:
        log_and_print(f"Current Speed (GNSS): {current_vel+3:.0f} kmph")
  
    try:
        message = send_message_to_mabx(const_speed, steer_output, 0, FLASHER_DICT["Both"], counter)
        mabx_socket.sendto(message, mabx_addr)
    except Exception as e:
        log_and_print(f"Error sending message to MABX: {e}")



def mainLoop():
    while not rospy.is_shutdown():
        try:
            log_and_print(f"Current Coordinate No. : {wp}")
            log_and_print(" ")
            # log_and_print(f"Velocity in kmph as per GNSS= {current_vel:.0f} kmph")

            latitude = float(lat)
            longitude = float(lng)
            Current_Bearing = float(heading)

            # time.sleep(SLEEP_INTERVAL/1000)
            navigation_output(latitude, longitude, Current_Bearing)
            time.sleep(SLEEP_INTERVAL / 1000)
        except ValueError as ve:
            log_and_print(f"ValueError occurred: {ve}")
        except IOError as ioe:
            log_and_print(f"IOError occurred: {ioe}")
        except KeyboardInterrupt:  # Currently not working
            log_and_print("Autonomous Mode is terminated manually!")
            message = send_message_to_mabx(0, 0, 0, 0, counter)
            mabx_socket.sendto(message, mabx_addr)
            raise SystemExit
        except Exception as e:
            log_and_print(f"An error occurred: {e}")


if __name__ == "__main__":
    global speed, reduction_factor, steer_output, counter, wp, file_path

    # Define the path(not relative path) to the waypoints file
    #file_path = '/home/orin/basler_v8/Waypoints/waypoints-maingate_to_testbed.txt'
    file_path = "/home/orin/basler_v8/Waypoints/DEMO.txt"
    #file_path="/home/s186/Desktop/Solio-ADAS/Solio-Suzuki/navigation/Waypoints/waypoints-keshav4.txt"
   #file_path ="/home/s186/Desktop/Solio-ADAS/Solio-Suzuki/navigation/Waypoints/waypoints-keshav2.txt"

    # log_dir = "devLogs"
    logger = setup_logging(file_path)
    logger.info("Development Code Starting")

    # Set sleep interval and lookahead distance
    SLEEP_INTERVAL = 100  # CHANGED FROM 5 TO 100
    LOOK_AHEAD_DISTANCE = 3

    # Define initial speeds, pothole_speed, turning_factor
    speed = 16
    turning_factor = 0.6
    wp = 0
    steer_output = 0
    counter = 0
    waypoints = get_coordinates(file_path)
    wp_len = len(waypoints)

    mainLoop()


