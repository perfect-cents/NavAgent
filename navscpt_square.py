"""A simple square NavAgent Script for copter.

"""


import argparse
import math
import time

from dronekit import Command, connect, LocationGlobal, LocationGlobalRelative, \
Vehicle, VehicleMode
from pymavlink import mavutil


PARSER = argparse.ArgumentParser(description="External Argument Parser for NavAgent.")
PARSER.add_argument("--connect",
                    help=("vehicle connection target string. "
                          "If not specified, SITL automatically started and used."))

PARSER_ARGS = PARSER.parse_args()

CONNECTION_STRING = PARSER_ARGS.connect
CONNECTION_STRING = "udp:127.0.0.1:14550"   # LAZY! change

print "Connecting to vehicle on: %s" % CONNECTION_STRING
VEHICLE = connect(CONNECTION_STRING, wait_ready=True, vehicle_class=Vehicle)

print "Setting Home Location..."
VEHICLE.home_location = VEHICLE.location.global_frame

CMDS = VEHICLE.commands
CMD_LIST = []

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

    if isinstance(original_location, LocationGlobal):
        target_location = LocationGlobal(new_lat, new_lon, original_location.alt)
    elif isinstance(original_location, LocationGlobalRelative):
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

def get_bearing(a_location, b_location):
    """ Input two global_frame or global_relative_frames.
    Returns the yaw/bearing in degrees starting clockwise from the north.
    """

    off_x = b_location.lon - a_location.lon
    off_y = b_location.lat - a_location.lat

    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795

    if bearing < 0:
        bearing += 360.00

    return bearing

def arm_and_takeoff(takeoff_alt=2):
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

    print "taking off from ({},{})".format(VEHICLE.home_location.lat, VEHICLE.home_location.lon)
    VEHICLE.simple_takeoff(takeoff_alt)

    while True:
        print "altitude: ", VEHICLE.location.global_relative_frame.alt
        if VEHICLE.location.global_relative_frame.alt >= takeoff_alt*.90:
            print "reached target altitude"
            break
        time.sleep(1)

def dist_till_wp():
    """ Returns the distance to the next next waypoint in meters."""

    if CMDS.next == 0:
        return None

    cmd = CMDS[CMDS.next-1] #commands are zero indexed
    lat = cmd.x
    lon = cmd.y
    alt = cmd.z
    target_waypoint_location = LocationGlobalRelative(lat, lon, alt)
    distance_to_waypoint = get_distance(VEHICLE.location.global_relative_frame,
                                        target_waypoint_location)

    return distance_to_waypoint

def add_cmds(cmd_list):
    """ Clears and Adds a cmd_list to the CMDS."""
    CMDS.clear()

    for cmd in cmd_list:
        CMDS.add(cmd)

    CMDS.upload()

def rotate_point(center, point, angle):
    """ Rotates a point around a center counter-clockwise for a specified angle in degrees.
    """

    #angle %= 360
    #if angle < 0: angle += 360

    lon = point.lon - origin.lon
    lat = point.lat - origin.lat

    new_lon = lon*math.cos(math.radians(angle)) - lat*math.sin(math.radians(angle)) + origin.lon
    new_lat = lon*math.sin(math.radians(angle)) + lat*math.cos(math.radians(angle)) + origin.lat

    return LocationGlobalRelative(new_lat, new_lon, point.alt)

def rotate_points(origin, points, angle):
    rotated_points = []

    for point in points:
        rotated_points.append(rotate_point(origin, point, angle))

    return rotated_points

def square(side_len, angle=0, corner_location=VEHICLE.home_location):
    """ Makes a cmd_list for a square pattern."""

    cmd_list = []

    points = [get_location(side_len, 0, corner_location),
              get_location(side_len, side_len, corner_location),
              get_location(0, side_len, corner_location),
              corner_location]

    points = rotate_points(points[-1], points, angle)

    cmd_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0,
                            0, 0, 2))
    for point in points:
        cmd_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                                point.lat, point.lon, 2))


    return cmd_list

print "Clearing any existing commands..."
CMDS.clear()

CMD_LIST = square(25, 0) #Adjust angle
CMD_LIST.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                        VEHICLE.home_location.lat, VEHICLE.home_location.lon, 2))
add_cmds(CMD_LIST)

print "Uploading new commands..."
CMDS.upload()

arm_and_takeoff()

VEHICLE.commands.next = 0 # Reset mission set to first (0) waypoint

VEHICLE.mode = VehicleMode("AUTO")  # Set mode to AUTO to start mission

print "Mission Started!"

while VEHICLE.system_status == 'ACTIVE':
    print 'Distance to waypoint {}: {}'.format(CMDS.next, dist_till_wp())

    if CMDS.next == CMDS.count:
        print "Exiting Standard Mission State..."
        break
    time.sleep(1)


#print 'Returning to Launch...'
#VEHICLE.mode = VehicleMode("RTL")

print "Setting LAND mode..."
VEHICLE.mode = VehicleMode("LAND")

print "Close vehicle object"
VEHICLE.close()
