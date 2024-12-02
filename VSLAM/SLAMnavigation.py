#!/usr/bin/env python3
# Author: Raj Parikh
# Author email ID: rajparikh2513@gmail.com

import math
import os
import logging
import datetime
import socket
import time
import numpy as np
import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# To get localisation values from /zed/zed_node/pose. 
from readlocalization import SubPoseTopic

# Define the MABX IP address and port for sending data
mabx_IP = "192.168.50.1"
mabx_PORT = 30000

# Define buffer size and local interface
BUFFER_SIZE = 8192
local_interface = 'eth0'

# Conversion factor for latitude and longitude to meters
# LAT_LNG_TO_METER = 1.111395e5
LAT_LNG_TO_METER = 1

get_localisation_from_zed_pose = SubPoseTopic()

# Initialize the ROS node for the algorithm
rospy.init_node('navigation', anonymous=True)

# Initialize the UDP socket for MABX communication
mabx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mabx_addr = (mabx_IP, mabx_PORT)

# Initialize actuation count and total count
actuation_count = 0
total_count = 0

# Function to parse and retrieve coordinates from a file
def get_coordinates(file_path):
    coordinates_list = []
    try:
        with open(file_path, 'r') as file:
            for line in file:
                try:
                    coordinates = [float(coord) for coord in line.strip().strip('[],').split(',')]
                    coordinates_list.append(coordinates)
                except ValueError:
                    # Handle the exception if a value cannot be converted to float
                    print(f"Error: Unable to convert coordinates in line '{line}' to float.")
    except FileNotFoundError:
        # Handle the exception if the file is not found
        print(f"Error: The file '{file_path}' could not be found.")
    except Exception as e:
        # Handle any other unexpected exceptions
        print(f"An error occurred: {e}")
    
    return coordinates_list

def localisation_pose():
    global heading

    lat, lng, orientation = get_localisation_from_zed_pose.fetch_localisation_values_from_loc_pose_cov()
    # lat, lng, orientation = get_localisation_from_zed_pose.fetch_localisation_values_from_loc_pose_cov()


    # Convert quaternion to Euler angles (roll, pitch, yaw)
    t0 = +2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    t1 = +1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
    yaw_rad = math.atan2(t0, t1)
    
    # Convert yaw from radians to degrees
    heading = math.degrees(yaw_rad)
    print(f"            HEADING = {heading}         yaw = {yaw_rad}")
    return lat, lng, heading



time.sleep(0.1)

# Function to calculate and set steering angle based on current angle and target angle
def set_angle(current_angle, angle_change):
    # Steering gear ratio
    gear_ratio = 17.75
    # Limit steering angle within a range
    if (40 * gear_ratio < current_angle):
        current_angle = 40 * gear_ratio
    elif (-40 * gear_ratio > current_angle):
        current_angle = -40 * gear_ratio
    # Calculate scaled angle for transmission
    scaled_angle = (current_angle / gear_ratio - (-65.536)) / 0.002
    high_angle_byte, low_angle_byte = (int)(scaled_angle) >> 8, (int)(scaled_angle) & 0xff
    return high_angle_byte, low_angle_byte

def calc_checksum(message_bytes):
    checksum = 0
    for m in message_bytes:
        checksum += m
    checksum = (0x00 - checksum) & 0x000000FF
    checksum = checksum & 0xFF
    return checksum

def set_speed(vehicle_speed):
    vehicle_speed = vehicle_speed * 128
    high_byte_speed = (int)(vehicle_speed) >> 8
    low_byte_speed = (int)(vehicle_speed) & 0xff
    return high_byte_speed, low_byte_speed

def send_message_to_mabx(speed, current_angle, target_angle, flasher_light, message_counter):
    H_Angle, L_Angle = set_angle(current_angle, -1 * target_angle)
    H_Speed, L_Speed = set_speed(speed)

    message_bytes = [1, message_counter, 0, 1, 52, 136, 215, 1, H_Speed, L_Speed, H_Angle, L_Angle, 0, flasher_light, 0, 0, 0, 0]
    message_bytes[2] = calc_checksum(message_bytes)
    message = bytearray(message_bytes)
    #print("Reading the sent Speed from MABX: ", message[8], message[9])
    #print("Reading the sent Angle from MABX: ", message[10], message[11])
    print("*********************************************************************")
    return message

def calculate_steer_output(currentLocation, Current_Bearing):
    global wp, actuation_count, total_count
    RAD_TO_DEG_CONVERSION = 57.2957795
    STEER_GAIN = 750

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

    if abs(bearing_diff) >= 1:
        actuation_count += 1


    steer_output = STEER_GAIN * np.arctan(-1 * 2 * 3.5 * np.sin(np.radians(-bearing_diff)) / 8)
    return steer_output, bearing_diff

def navigation_output(latitude, longitude, Current_Bearing):
    global counter, speed, wp, const_speed, actuation_count, total_count
    flasher = 3
    counter = (counter + 1) % 256
    total_count+=1

    print(f"X: {latitude} , Y: {longitude}, BEAR: {Current_Bearing}") 
    currentLocation = [latitude, longitude]
    distance_to_final_waypoint = np.linalg.norm(np.array(currentLocation) - waypoints[-1]) * LAT_LNG_TO_METER

    # Calculate actuation percentage
    actuation_percentage = (actuation_count / total_count) * 100 if total_count != 0 else 0

    # Print or return actuation percentage as needed
    print(f"Actuation Percentage: {actuation_percentage:.2f}%")

    if (distance_to_final_waypoint> 1 and wp < wp_len):
            
        steer_output, bearing_diff = calculate_steer_output(currentLocation, Current_Bearing)
        steer_output *= -1.0        
        const_speed = speed

        if wp < 10 or wp_len - 10 < wp:
            const_speed = turning_factor * speed

        if abs(bearing_diff) > 5:
            const_speed = turning_factor * speed
        
        distance_to_nextpoint = np.linalg.norm(np.array(currentLocation) - waypoints[wp]) * LAT_LNG_TO_METER
        print(f"{wp} out of {wp_len} | Next Coordinate distance : {distance_to_nextpoint:.1f} m")
        try:
            if wp < wp_len and distance_to_nextpoint < LOOK_AHEAD_DISTANCE:
                wp += 1
        except IndexError:
            print("Waypoint index out of range!")
    else:
        print("----- FINISHED  -----")
        print("Brake Activated")
        steer_output = 0                                                                                                                          
        const_speed = 0
        flasher = 0
    
    
    print(f"Speed set by code: {const_speed:.0f} kmph")
    try:
        message = send_message_to_mabx(const_speed, steer_output, 0, flasher, counter)  
        mabx_socket.sendto(message, mabx_addr)
    except Exception as e:
        print(f"Error sending message to MABX: {e}")

def mainLoop(): 
    while not rospy.is_shutdown():
        try:
            print(f"Current Coordinate No. : {wp}")
            print(" ")
            latitude, longitude, heading = localisation_pose()


            Current_Bearing = float(heading)
            print(f"{latitude}  | {longitude} \n")

            # time.sleep(SLEEP_INTERVAL/1000)
            navigation_output(latitude, longitude, Current_Bearing)
            time.sleep(SLEEP_INTERVAL/1000)           
        except KeyboardInterrupt:
            print("Autonomous Mode is terminated manually!")
            message = send_message_to_mabx(0, 0, 0, 0, counter)
            mabx_socket.sendto(message, mabx_addr)  
            raise SystemExit
        
        except Exception as e:
            print(f"An error occurred: {e}")

if __name__ == '__main__':

    global speed, reduction_factor, steer_output, counter, curr_time, wp, file_path

    # Define the path(not relative path) to the waypoints file
    file_path = '/home/tihan/Map based Navigation/waypoints_new.txt'
    
    log_dir = "logs"

    # Set sleep interval and lookahead distance
    SLEEP_INTERVAL = 5
    LOOK_AHEAD_DISTANCE = 5

    # Define initial speeds, pothole_speed, turning_factor
    speed = 7
    turning_factor = 0.6    
    wp = 0
    steer_output = 0    
    counter = 0
    
    # Get the list of waypoints from the file
    waypoints = get_coordinates(file_path)
    wp_len = len(waypoints)

    print(f" Speed : {speed} , Turning Factor : {turning_factor} , Sleep : {SLEEP_INTERVAL}, Look ahead distance : {LOOK_AHEAD_DISTANCE}")       # Test SteerGAIN here
    # Start the main loop
    mainLoop()