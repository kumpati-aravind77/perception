from collections import namedtuple

# SPEED VALUES
BRAKE_SPEED = 0
TOP_SPEED = 8
# SPEED VALUES

# MABX CONFIG
MABX_IP = "192.168.50.1"  # Mabx IP for sending from Pegasus
MABX_PORT = 30000  # Mabx port for sending from Pegasus
BUFFER_SIZE = 4096  # packet size
LOCAL_INTERFACE = 'eno1'
NAVIGATION_DATA = 'data.csv'

# WAYPOINT_FILENAME = '/usr/local/zed/samples/object-avoidance-zed-suzuki/waypoints_navigation_20_12.txt'


# WAYPOINT_FILENAME = '/usr/local/zed/samples/object-avoidance-zed-suzuki/waypoints_2023-12-23.txt'

WAYPOINT_FILENAME = '/home/tihan/Music/Solio-ADAS/Solio-Suzuki/navigation/solio-waypoints/waypoints-aravind-gate-side.txt'

# WAYPOINT_FILENAME = '/usr/local/zed/samples/object-avoidance-zed-suzuki/waypoints-curve-helipad.txt'

# WAYPOINT_FILENAME = '/usr/local/zed/samples/object-avoidance-zed-suzuki/waypoints-lap1.txt'

#WAYPOINT_FILENAME = '/usr/local/zed/samples/object-avoidance-zed-suzuki/waypoints-22-12.txt'

# WAYPOINT_FILENAME = '/usr/local/zed/samples/object-avoidance-zed-suzuki/waypoints_2023-12-22-16:36:50.txt'

# OVERTAKE_WAYPOINT_FILENAME = '/usr/local/zed/samples/object-avoidance-zed-suzuki/overtake_waypoints_2023-12-15-16:52:40.txt'
# MABX CONFIG

# NAVIGATION
STEER_GAIN = 750          # For Tight Turns 1200 can be used
TURN_BEARNG_THRESHOLD = 6
# Conversion factor for latitude and longitude to meters
LAT_LNG_TO_METER = 1.111395e5
WP_DIST = 3
RAD_TO_DEG_CONVERSION = 57.2957795
# NAVIGATION

# COLLISION WARNING
MAX_CLASS_ID = 7
MAX_DEPTH = 20
NUM_INTERPOLATED_POINTS = 500

OBJ_CLASS_CAR = 2
OBJ_CLASS_CYCLE = 1
EXTEND_WAYPOINTS_LONG_OBS = 3

CLASSES = [ 'person', 'bicycle', 'car', 'motocycle', 'route board',                             #5
            'bus', 'commercial vehicle', 'truck', 'traffic sign', 'traffic light',              #5
            'autorickshaw','stop sign', 'ambulance', 'bench', 'construction vehicle',           #5
            'animal', 'unmarked speed bump', 'marked speed bump', 'pothole', 'police vehicle',  #5
            'tractor', 'pushcart', 'temporary traffic barrier', 'rumblestrips', 'traffic cone', #5
            'pedestrian crossing']

REQ_CLASSES = [0,1,2,3,4,7,8,9,10,11,12,13,15,16,17,18,19,20,25,26]
# PERSONS_VEHICLES_CLASSES = [0,1,2,3,4,6,7,8,11,13,15,20,21]
PERSONS_VEHICLES_CLASSES = [0,1,2,3,6,7,25]
DRIVING_LANE_SPACE = 10
OVERTAKE_LANE_SPACE = 5
NUM_INTERPOLATED_POINTS = 500
LEFT_RIGHT_DISTANCE = 1.6		#in meteres in either side
# STOP_DISTANCE = 3		#in meteres in front of car     #actual 5.5
# STOP_DISTANCE = 10		#in meteres in front of car     #actual 5.5
STOP_DISTANCE = 10		#in meteres in front of car     #actual 5.5
# STOP_DISTANCE = 8		#in meteres in front of car     #actual 5.5
DETECTING_DISTANCE = 20     #
CAUTION_DISTANCE = 15 
CLOSENESS_THRES = 50000
# COLLISION WARNING

# DRIVESPACE
LABEL = namedtuple( "LABEL", [ "name", "train_id", "color"])
DRIVABLES = [ 
             LABEL("direct", 0, (0, 255, 0)),        # green
             LABEL("alternative", 1, (0, 0, 255)),   # blue
             LABEL("background", 2, (0, 0, 0)),      # black          
            ]
# DRIVESPACE

# OVERTAKE PATH CONFIG
TARGET_REACH = 1
BEARING_ZERO = 0

OVERTAKE_WAYPOINT_DIST = 4
WAIT_TIME = 1800
# OVERTAKE PATH CONFIG

# LANE VELOCITY
DRIVE_SPEED = 10
CHANGE_SPEED = 4
# CHANGE_SPEED = 8
OVERTAKE_SPEED = 5
# LANE VELOCITY

## STATES

# COLLISION WARNING STATES
NO_WARNING = 100
MID_WARNING = 101
URGENT_WARNING = 102
# COLLISION WARNING STATES

# COLLISION AVOIDANCE
OVERTAKE = 200
CONTINUE = 201
SWITCH = 202
# COLLISION AVOIDANCE

# LANE STATES
DRIVING_LANE = 300
CHANGE_LANE = 301
OVERTAKE_LANE = 302
# LANE STATES

# OPTICAL FLOW
TRAFFIC_FROM_LEFT = 400
SAFE_TO_OVERTAKE = 401
TRAFFIC_FROM_RIGHT = 402
TIME_TO_CHECK_FOR_TRAFFIC_FROM_BACK = 5
# OPTICAL FLOW

STATE_DICT = {
    100: "NO_WARNING",
    101: "MID_WARNING",
    102: "URGENT_WARNING",
    200: "OVERTAKE",
    201: "CONTINUE",
    202: "SWITCH",
    300: "DRIVING_LANE",
    301: "CHANGE_LANE",
    302: "OVERTAKE_LANE",
    400: "TRAFFIC_FROM_LEFT",
    401: "SAFE_TO_OVERTAKE",
    402: "TRAFFIC_FROM_RIGHT",
}

DECISION_THRESHOLD = 15

ZERO_STEET_OUTPUT = 0

# OBS_LAT = 17.602051094458595
# OBS_LON = 78.12708143925757
global OBS_LAT, OBS_LON

OBS_LAT = 17.602029105524696 
OBS_LON = 78.12708292791199

FILENAME_TRAIL = f"TRAIL_OF_WAYPOINTS_GEN.txt"

# Set sleep interval and lookahead distance
SLEEP_INTERVAL = 100            # CHANGED FROM 5 TO 100
LOOK_AHEAD_DISTANCE = 3
LOOK_AHEAD_DISTANCE_DURING_OVERTAKE = 6
OVERTAKE_LOOK_AHEAD_DISTANCE = 3
TURNING_FACTOR = 0.6

NO_INDICATOR = 0
LEFT_INDICATOR = 1
RIGHT_INDICATOR = 2
BOTH_INDICATOR = 3

BEARING_DIFF_THRESHOLD = 5

SPEED_REDUCTION_FACTOR = 0.8


DISTANCE_PED =  [0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.0005]
BEARING_PED  =  [90.24 , 18.24, 21.24, 5.24,  5.24, 5.24, 5.24, 5.24]

DISTANCE_CYCLE =  [0.002, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.0005]
BEARING_CYCLE  =  [90.24 , 18.24, 21.24, 21.24, 5.24,  5.24, 5.24, 5.24]

DISTANCE_CAR =  [0.002,  0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.0005]
BEARING_CAR  =  [70.24 , 18.24,   21.24,  5.24,  5.24,  5.24,  5.24,  5.24,  5.24,  5.24,  5.24,  5.24,  5.24, 5.24, 5.24, 5.24]

# DISTANCE_M =  [0.002, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.0005]
# BEARING_CYCLE  =  [90.24 , 18.24, 21.24, 21.24, 5.24,  5.24, 5.24, 5.24]

## During curve
# DISTANCE_CAR =  [0.002,  0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.0005]
# BEARING_CAR  =  [50.24 , 18.24,   21.24,  5.24,  5.24,  5.24,  5.24,  5.24,  5.24,  5.24,  5.24,  5.24,  5.24,  5.24, 5.24, 5.24, 5.24, 5.24, 5.24, 5.24]

# DISTANCE_CAR =  [0.002,  0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.0005]
# BEARING_CAR  =  [50.24 , 18.24,   21.24,  5.24,  5.24,  5.24,  5.24,  5.24,  5.24,  5.24,  5.24,  5.24, 5.24, 5.24, 5.24]


def set_dynamic_obstacle(_lat, _lon):
    global OBS_LAT, OBS_LON
    print(f"OBS_LAT = {OBS_LAT} OBS_LON = {OBS_LON}")
    OBS_LAT = _lat
    OBS_LON = _lon
