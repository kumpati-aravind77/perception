import datetime
import sys
import time

import rospy
from novatel_oem7_msgs.msg import BESTPOS
from std_msgs.msg import Float32, Float32MultiArray, Float64, String

# Global variables to store GPS and heading information
global lat, lng, heading, current_vel, vel_head
global new_file

# Initialize ROS node for recording waypoints
rospy.init_node('record_wp', anonymous=True)

# Callback function for handling GPS data
def callback_latlng(data):
    global lat, lng, lat_delta, lng_delta, num_svs, num_sol_svs
    lat = data.lat
    lng = data.lon
    lat_delta = data.lat_stdev
    lng_delta = data.lon_stdev
    num_svs = data.num_svs
    num_sol_svs = data.num_sol_svs

# Subscribe to the BESTPOS topic to receive GPS data
rospy.Subscriber("/novatel/oem7/bestpos", BESTPOS, callback_latlng)

# Check if a file name is provided as a command line argument
if len(sys.argv) > 1:
    user_input = sys.argv[1]
    new_file = f"waypoints-{user_input}.txt"
    print(f"Waypoints will be saved as : {new_file}")
else:
    # Generate a timestamp-based file name
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H:%M:%S")
    new_file = f"new-waypoints_{timestamp}.txt"
    print(f"File name not provided: Waypoints will be saved as : {new_file}")

# Function to save waypoint coordinates to a file
def save_waypoint_to_file(lat, lng, new_file_name):
    with open(new_file_name, "a") as file:
        file.writelines(f"[{lat},{lng}],\n")

# Open the file in write mode to store waypoints
file = open(new_file, "w")

# Counter to keep track of captured waypoints
wp_counter = 0

# Main loop to capture and save waypoints
while True:
    try:
        time.sleep(1)  # Delay of 1 second between capturing waypoints
        print(lat, lng)
        
        # Update the waypoint counter
        wp_counter += 1
        
        print(f"=== {wp_counter}, waypoint captured @ {datetime.datetime.now().strftime('%H:%M:%S')} || Standard Deviation : {100*lat_delta:.4f} cm | Satellites Locked : {num_sol_svs}/{num_svs} ===")
        save_waypoint_to_file(lat, lng, new_file)

    except KeyboardInterrupt:
        # Handle keyboard interrupt (e.g., when Ctrl+C is pressed to stop the program)
        print("Program interrupted by user. Exiting...")
        break
    
    except Exception as e:
        # Handle any other unexpected exceptions
        print(f"An error occurred: {e}")
    finally:
        # Close the file
        file.close()

# Code written by Rishav
