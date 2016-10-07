"""A simple lownmower for copter NavAgent Scripts.

"""


import argparse
import math
import time

from dronekit import Command, connect, LocationGlobal, LocationGlobalRelative, Vehicle, VehicleMode
from pymavlink import mavutil


PARSER = argparse.ArgumentParser(description="External Argument Parser for NavAgent.")
PARSER.add_argument("--connect",
                    help=("vehicle connection target string. "
                          "If not specified, SITL automatically started and used."))

PARSER_ARGS = PARSER.parse_args()

CONNECTION_STRING = PARSER_ARGS.connect
#CONNECTION_STRING = "udp:127.0.0.1:14550"   # LAZY! change

'''

SITL = None

if not CONNECTION_STRING: # Start SITL if no connection string specified
    import dronekit_sitl
    SITL = dronekit_sitl.start_default()
    CONNECTION = SITL.connection_string()

'''

print "Connecting to vehicle on: %s" % CONNECTION_STRING
VEHICLE = connect(CONNECTION_STRING, wait_ready=True, vehicle_class=Vehicle)

print "Setting Home Location..."
VEHICLE.home_location = VEHICLE.location.global_frame

CMDS = VEHICLE.commands

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

def dist_till_wp():
    """ Returns the distance to the next command i.e. next waypoint in meters."""

    if VEHICLE.commands.next == 0:
        return None

    cmd = VEHICLE.commands[VEHICLE.commands.next-1] #commands are zero indexed
    lat = cmd.x
    lon = cmd.y
    alt = cmd.z
    target_waypoint_location = LocationGlobalRelative(lat, lon, alt)
    distance_to_waypoint = get_distance(VEHICLE.location.global_relative_frame,
                                        target_waypoint_location)

    return distance_to_waypoint

def add_cmd_list(cmd_list):
    CMDS.clear()

    for cmd in cmd_list:
        CMDS.add(cmd)

    CMDS.upload()

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

def rotate_point(origin, point, angle):
    o_lon = origin.lon
    o_lat = origin.lat

    p_lon = point.lon
    p_lat = point.lat

    x = o_lon - p_lon
    y = o_lat - p_lat

    theta = math.radians(angle)

    x_p = x*math.cos(theta) - y*math.sin(theta)
    y_p = y*math.cos(theta) + x*math.sin(theta)

    new_lon = x_p + o_lon
    new_lat = y_p + o_lat

    return LocationGlobalRelative(new_lat, new_lon, point.alt)

def rotate_points(origin, points, angle):
    result = []
    for point in points:
        result.append(rotate_point(origin, point, angle))
    return result

def lawn_mower_pattern(ln_nm=None, ln_wdth=None, ln_hght=None, angle=0):
    """ Input the number of cycles the mower pattern goes through, the eastward
    distance and the northward distance the mower travels.
    The Method will add the commands to the command que but not upload them.
    """

    cmd_list = []

    place_holder_point = VEHICLE.location.global_frame
    start_point = place_holder_point

    points = []

    print 'Making Plans...'

    for _ in range(ln_nm):
        points.append(get_location(ln_hght, 0, place_holder_point))
        points.append(get_location(ln_hght, ln_wdth/2, place_holder_point))
        points.append(get_location(0, ln_wdth/2, place_holder_point))
        points.append(get_location(0, ln_wdth, place_holder_point))

        place_holder_point = get_location(0, ln_wdth, place_holder_point)

    points = rotate_points(start_point, points, angle-180)

    print 'Adding Plans...'

    cmd_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0,
                            0, 0, 2))

    for i in range(len(points)):
        cmd_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                                points[i].lat, points[i].lon, 2))

    cmd_list.pop()
    cmd_list.append(cmd_list[-1])

    return cmd_list


print "Clearing any existing commands..."
CMDS.clear()

CMD_LIST = lawn_mower_pattern(2, 10, 20, 0)

CMD_LIST.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                        VEHICLE.home_location.lat, VEHICLE.home_location.lon, 2))
CMD_LIST.append(CMD_LIST[-1])

print "Uploading new commands..."
add_cmd_list(CMD_LIST)

arm_and_takeoff()

VEHICLE.commands.next = 0 # Reset mission set to first (0) waypoint

VEHICLE.mode = VehicleMode("AUTO")  # Set mode to AUTO to start mission

print "Mission Started!"

while VEHICLE.system_status == 'ACTIVE':
    if CMDS.next == CMDS.count:
        print "Exiting Standard Mission State..."
        break

    print 'Distance to waypoint {}: {}'.format(CMDS.next, dist_till_wp())
    time.sleep(1)

print "Setting LAND mode..."
VEHICLE.mode = VehicleMode("LAND")

print "Close vehicle object"
VEHICLE.close()
