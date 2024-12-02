from gps_common.msg import GPSFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
from novatel_oem7_msgs.msg import BESTPOS, BESTVEL, INSPVA
import socket
import time
import numpy as np
import datetime
import socket
import argparse
import rospy
from novatel_oem7_msgs.msg import BESTVEL, BESTPOS

import math
import os
import logging
import datetime
import time
import novatel_oem7_msgs
import numpy as np

# Initialize the ROS node for the algorithm
rospy.init_node("feedback", anonymous=True)
logger = None


class MessageCounterFilter(logging.Filter):
    def __init__(self):
        super().__init__()
        self.counter = 0

    def filter(self, record):
        self.counter += 1
        record.msg = f"{self.counter:05d}: {record.msg}"
        return True


def setup_logging(file_path):
    log_dir = os.path.expanduser(
        "~/Desktop/Solio-ADAS/Solio-Suzuki/navigation/SingletonCodes/vehicleDynamics")
    base_filename = os.path.basename(file_path)
    # Remove the file extension to get the desired string
    logFileName = os.path.splitext(base_filename)[0]

    # Create the log directory if it does not exist
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # Set up the logging format
    log_format = "%(asctime)s :- %(message)s"
    logging.basicConfig(level=logging.DEBUG, format=log_format)

    # Create a log file with the current timestamp as the name
    current_time = datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
    # log_filename = f"{logFileName}-{current_time}.log"
    log_filename = f"{logFileName}.log"
    log_path = os.path.join(log_dir, log_filename)

    # Add a file handler to save logs to the file
    file_handler = logging.FileHandler(log_path)
    file_handler.setLevel(logging.DEBUG)

    # Set the log format for the file handler
    file_handler.setFormatter(logging.Formatter(log_format))

    # Add the file handler to the logger
    logger = logging.getLogger("")
    logger.addHandler(file_handler)

    # Add the message counter filter to the logger
    message_counter_filter = MessageCounterFilter()
    logger.addFilter(message_counter_filter)

    return logger


def log_it(str):
    global logger
    logger.info(str)
    # print(str)


def callback_velocity(data):
    global current_vel
    current_vel = 3.6 * data.hor_speed


def callback_heading(data):
    global heading
    heading = (
        data.azimuth
    )  # Left-handed rotation around z-axis in degrees clockwise from North.


def callback_gnss_imu(data):
    global acc_z
    acc_z = 0
    acc_z = data.linear_acceleration.z


def callback_latlng(data):
    global lat_delta, lng_delta
    lat_delta = data.lat_stdev
    lng_delta = data.lon_stdev


def callback_gps(data):
    global secs, latitude, longitude, roll, pitch
    secs = data.header.stamp.secs
    latitude = data.latitude
    longitude = data.longitude
    roll = data.roll
    pitch = data.pitch


rospy.Subscriber("/novatel/oem7/bestvel", BESTVEL, callback_velocity)
rospy.Subscriber("/novatel/oem7/inspva", INSPVA, callback_heading)
rospy.Subscriber("/novatel/oem7/bestpos", BESTPOS, callback_latlng)
rospy.Subscriber("/imu/data_raw", Imu, callback_gnss_imu)
rospy.Subscriber("/gps/gps", GPSFix, callback_gps)

time.sleep(1)

UDP_IP = "192.168.50.2"  # IP for receiving
UDP_PORT = 51001        # port for receiving

# Define constants for message structure
NUM_BYTES = [1, 1, 1, 1, 2, -1, 2, -1, 1, 1]

SHIFT_DICT = {0: "Shift in progress", 1: "L", 8: "R",
              9: "N", 10: "P", 11: "D", 13: "S", 14: "M"}
FLASHER_DICT = {0: "None", 1: "Left", 2: "Right", 3: "Both"}


def decode_message(hex_str):
    i = 0
    j = 0
    decoded_val = []

    while i < 20:
        n_bytes = NUM_BYTES[j]
        byte_val = hex_str[i:i + 2 * n_bytes]

        if n_bytes == 2:
            byte_val = byte_val[2:4] + byte_val[0:2]

        decimal = int("0x" + byte_val, 16)
        decoded_val.append(decimal)

        i += 2 * n_bytes
        j += n_bytes

    return decoded_val


def get_speed(high_byte_speed, low_byte_speed):
    """
    Args:
    high_byte_speed: The high byte of the scaled speed.
    low_byte_speed: The low byte of the scaled speed.
    """

    # Combine the high and low bytes into a single integer.
    speed = (high_byte_speed << 8) | low_byte_speed

    # Divide the speed by 128 to convert it back to km/h.
    speed = speed / 128
    # return int(np.around(speed, 1))
    return (np.around(speed, 1))


def get_tire_angle(high_byte_angle, low_byte_angle):
    # Combine the high and low bytes into a single integer.
    angle = (high_byte_angle << 8) | low_byte_angle

    # Apply offset and convert to degrees (LSB = 0.002)
    tire_angle = (angle) * 0.002 - 65.536

    return np.round(tire_angle, 1)


def feedback_mabx():
    global result
    print("     From MABX: ")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    # print("Ready to receive..")
    bytestr, addr = sock.recvfrom(1024)
    # print("Receiving..")

    byte_array = bytearray(bytestr)
    hex_str = byte_array.hex()
    if len(hex_str) == 20:

        # print("  Ignoring message of size 10 ")
        decoded_val = decode_message(hex_str)
        # print(f"{bytestr}")
        # print(f"   Decoded Values: {decoded_val}")
        # print("ID: ", decoded_val[0])
        print("Rolling counter: ", decoded_val[1])
        # print("Current Mode: ", bytestr[2])
        print("Current Speed: ", get_speed(bytestr[4], bytestr[5]))
        print("Current Tire Angle: ", get_tire_angle(bytestr[6], bytestr[7]))
        # print("Current Shift: ", SHIFT_DICT.get(decoded_val[6], "Unknown"))
        # print("Current Flasher: ", FLASHER_DICT.get(decoded_val[7], "Unknown"))
        result = f"{decoded_val[1]:03d} , {get_speed(bytestr[4], bytestr[5]):05.2f} , {get_tire_angle(bytestr[6], bytestr[7]):05.2f} || "
        # log_it(f"{decoded_val[1]} | {get_speed(bytestr[4], bytestr[5])} , {get_tire_angle(bytestr[6], bytestr[7])}")

    sock.close()


def feedback_loop():
    global result
    # while not rospy.is_shutdown():
    print("     From GNSS-IMU: ")
    print(
        f"UNIX time : {secs} , [{latitude:.12f}, {longitude:.12f}] , {lat_delta:4.2f}  , {lng_delta:4.2f}  | {current_vel:3.2f} kmph , {acc_z:4.2f} m/sec^2   |  {roll:.2f}R , {pitch:.2f}P , {heading:.2f}H ")
    result += f"{secs}, {latitude:.12f} , {longitude:.12f}, {lat_delta:4.2f} , {lng_delta:4.2f} ,{current_vel:3.2f} , {acc_z:4.2f} , {roll:4.2f} , {pitch:4.2f} , {heading:.2f}"
    # log_it(f"{secs} , [{latitude:.12f}, {longitude:.12f}]   | {current_vel:3.2f} , {acc_z:4.2f}  |  {roll:.2f} , {pitch:.2f} , {heading:.2f}")
    log_it(f"{result}")

    print("====================================================================================================")


def main():
    global logger, sock, result, seq
    seq = 0
    # Create the parser
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--name', type=str, help='The name of the file')
    # Parse the arguments
    args = parser.parse_args()
    # Now you can use args.file_name to get the file name
    file_name = args.name

    # take file name for logging
    # file_name = "feedback-testbed.txt"

    logger = setup_logging(file_name)
    logger.info("Feedback Code Starting")

    while True:
        seq += 1
        # log_it(f"Seq: {seq}")
        print(f"Seq: {seq}")
        time.sleep(20/1000)
        feedback_mabx()
        feedback_loop()

    # sock.close()


if __name__ == '__main__':
    main()
