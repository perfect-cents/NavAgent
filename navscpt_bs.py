"""A base template for copter NavAgent Scripts.

"""


import argparse
import math
import time

from dronekit import Command, connect, LocationGlobal, LocationGlobalRelative, Vehicle, VehicleMode
from pymavlink import mavutil



PARSER = argparse.ArgumentParser(description='External Argument Parser for NavAgent.')
PARSER.add_argument('--connect',
                    help=("vehicle connection target string. "
                          "If not specified, SITL automatically started and used."))

PARSER_ARGS = PARSER.parse_args()

CONNECTION_STRING = PARSER_ARGS.connect
CONNECTION_STRING = 'udp:127.0.0.1:14550'   # LAZY! change

SITL = None

if not CONNECTION_STRING: # Start SITL if no connection string specified
    import dronekit_sitl
    SITL = dronekit_sitl.start_default()
    CONNECTION = SITL.connection_string()

print 'Connecting to vehicle on: %s' % CONNECTION_STRING
VEHICLE = connect(CONNECTION_STRING, wait_ready=True, vehicle_class=Vehicle)

print  'Setting Home Location...'
VEHICLE.home_location = VEHICLE.location.global_frame

CMDS = VEHICLE.commands

print "Clearing any existing commands..."
CMDS.clear()

#### Input Initial Commands ####

print "Uploading new commands..."
CMDS.upload()

def get_location(meters_north, meters_east,
                 original_location=VEHICLE.location.global_relative_frame):
    """ Input the a global_frame or a global_relative_frame, and offsets in meters.
    Returns a new global_frame or global_relative_frame.
    """

    earth_radius = 6378137.0 #Radius of "spherical" earth

    lat_dist = meters_north/earth_radius
    lon_dist = meters_east/(earth_radius*math.cos(math.pi*original_location.lat/180))

    new_lat = original_location.lat + (lat_dist * 180/math.pi)
    new_lon = original_location.lon + (lon_dist * 180/math.pi)

    if isinstance(original_location) is LocationGlobal:
        target_location = LocationGlobal(new_lat, new_lon, original_location.alt)
    elif isinstance(original_location) is LocationGlobalRelative:
        target_location = LocationGlobalRelative(new_lat, new_lon, original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return target_location

def get_distance(a_location, b_location):
    """ Input two global_frame or global_relative_frames.
    Returns the distance as a float.
    """

    d_lat = b_location.lat - a_location.lat
    d_long = b_location.lon - a_location.lon

    return math.sqrt((d_lat*d_lat) + (d_long*d_long)) * 1.113195e5

def get_bearing(a_location=VEHICLE.location.global_relative_frame,
                b_location=VEHICLE.commands.next):
    """ Input two global_frame or global_relative_frames.
    Returns the yaw/bearing in degrees starting clockwise from the north.
    """

    off_x = b_location.lon - a_location.lon
    off_y = b_location.lat - a_location.lat

    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795

    if bearing < 0:
        bearing += 360.00

    return bearing

def distance_to_next_waypoint():
    """ Returns the distance to the next command i.e. next waypoint in meters."""

    if NEXT_WAYPOINT == 0:
        return None

    cmd = VEHICLE.commands[NEXT_WAYPOINT-1] #commands are zero indexed
    lat = cmd.x
    lon = cmd.y
    alt = cmd.z
    target_waypoint_location = LocationGlobalRelative(lat, lon, alt)
    distance_to_waypoint = get_distance(VEHICLE.location.global_frame, target_waypoint_location)

    return distance_to_waypoint


def arm_and_takeoff(target_alt):
    """ Arms the Copter for Takeoff and Launches it to a target_alt."""

    print "basic pre-arm checks..."
    # don't let the user try to arm until autopilot is ready
    while not VEHICLE.is_armable:
        print " waiting for vehicle to initialise..."
        time.sleep(1)

    print "arming motors..."
    # copter should arm in guided mode
    VEHICLE.mode = VehicleMode("GUIDED")
    VEHICLE.armed = True

    while not VEHICLE.armed:
        print "waiting for arming..."
        time.sleep(1)

    print "taking off from ({},{})".format(VEHICLE.home_location.lat,
                                           VEHICLE.home_location.lon)
    VEHICLE.simple_takeoff(target_alt)

    while True:
        print "altitude: ", VEHICLE.location.global_relative_frame.alt
        if VEHICLE.location.global_relative_frame.alt >= target_alt * .95:
            print "reached target altitude"
            break
        time.sleep(1)

def closing_seq():
    #print 'Returning to Launch...'
    #VEHICLE.mode = VehicleMode("RTL")

    print "Setting LAND mode..."
    VEHICLE.mode = VehicleMode("LAND")

    print "Close vehicle object"
    VEHICLE.close()

    if SITL is not None:
        SITL.stop()


def insert_waypoint(lat, lon, alt, nxt):
    CMDS.download()
    CMDS.wait_ready() # BIG FIX!!!: Incorperate threading

    cmd_list = []
    for cmd in CMDS:
        cmd_list.append(cmd)

    cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                  0, 0, 0, 0, 0, 0, lat, lon, alt)

    cmd_list.insert(nxt, cmd)

    CMDS.clear()

    for cmd in cmd_list:
        CMDS.add(cmd)

    CMDS.upload()

def go_up(lat=VEHICLE.location.global_relative_frame.lat,
          lon=VEHICLE.location.global_relative_frame.lon,
          alt=VEHICLE.location.global_relative_frame.alt,
          nxt=0):
    CMDS.download()
    CMDS.wait_ready() # BIG FIX!!!: Incorperate threading

    cmd_list = []
    for cmd in CMDS:
        cmd_list.append(cmd)

    cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                  0, 0, 0, 0, 0, 0, lat, lon, alt)

    cmd_list.insert(nxt, cmd)

    CMDS.clear()

    for cmd in cmd_list:
        CMDS.add(cmd)

    CMDS.upload()

arm_and_takeoff(10)

print "Starting mission"
VEHICLE.commands.next = 0 # Reset mission set to first (0) waypoint
NEXT_WAYPOINT = VEHICLE.commands.next


VEHICLE.mode = VehicleMode("AUTO")  # Set mode to AUTO to start mission

while True:
    break


closing_seq()


def rotate_point(origin, point, angle):
    o_lat = origin.lat
    o_lon = origin.lon

    p_lat = point.lat
    p_lon = point.lon

    y = o_lat - p_lat
    x = o_lon - p_lon

    theta = math.radians(angle)

    x_p = x*math.cos(theta) - y*math.sin(theta)
    y_p = y*math.cos(theta) + x*math.sin(theta)

    new_lat = y_p + o_lat
    new_lon = x_p + o_lon

    return LocationGlobalRelative(new_lat, new_lon, point.alt)
