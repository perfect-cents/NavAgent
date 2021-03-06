"""
    A NavAgent Script for the Copter.
"""


import argparse
import math
import time

from dronekit import Command, connect, LocationGlobal, LocationGlobalRelative, \
                     Vehicle, VehicleMode
from pymavlink import mavutil


def is_iterable(maybe_iter, unless=(basestring, dict)):
    """
        Checks if maybe_iter is is_iterable if not in unless.
    """

    try:
        iter(maybe_iter)
    except TypeError:
        return False
    return not isinstance(maybe_iter, unless)

def iterate(maybe_iter, unless=(basestring, dict)):
    """
        Returns maybe_iter as [maybe_iter] if the instance isn't iteratable and
        not in unless.
    """
    if is_iterable(maybe_iter, unless=unless):
        return maybe_iter
    return [maybe_iter]


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
DEV_CMD_LIST = []


def get_distance(a_location, b_location):
    """
        Input two global_frame or global_relative_frames.

        Returns the distance as a float.
    """

    d_lat = b_location.lat - a_location.lat
    d_long = b_location.lon - a_location.lon

    return math.sqrt((d_lat*d_lat) + (d_long*d_long)) * 1.113195e5

def get_bearing(a_location, b_location):
    """
        Input two global_frame or global_relative_frames.

        Returns the yaw/bearing in degrees starting clockwise from the north.
    """

    off_x = b_location.lon - a_location.lon
    off_y = b_location.lat - a_location.lat

    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00

    return bearing

def get_location(meters_north, meters_east,
                 original_location=VEHICLE.location.global_relative_frame):
    """
        Input the a global_frame or a global_relative_frame, and offsets in meters.

        Returns a new global_frame or global_relative_frame.
    """

    earth_radius = 6378137.0  # Radius of "spherical" earth

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

def get_location_polor(bearing, dist,
                       original_location=VEHICLE.location.global_relative_frame):
    """
        Input the a global_frame or a global_relative_frame, and polor offsets
        as angle, and meters.

        Returns a new global_frame or global_relative_frame.
    """

    earth_radius = 6378137.0  # Radius of "spherical" earth

    bearing = 360 - (bearing - 90)%360
    bearing = math.radians(bearing)

    meters_north = math.sin(bearing) * dist
    meters_east = math.cos(bearing) * dist

    lat_dist = meters_north / earth_radius
    lon_dist = meters_east / (earth_radius*math.cos(math.pi*original_location.lat/180))

    new_lat = original_location.lat + (lat_dist * 180/math.pi)
    new_lon = original_location.lon + (lon_dist * 180/math.pi)

    if isinstance(original_location, LocationGlobal):
        target_location = LocationGlobal(new_lat, new_lon, original_location.alt)
    elif isinstance(original_location, LocationGlobalRelative):
        target_location = LocationGlobalRelative(new_lat, new_lon, original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return target_location

def rotate_point(center, point, angle, clockwise=True):
    """
        Rotates a point around a center counter-clockwise for a specified angle
        in degrees.
    """

    if clockwise is True:
        angle *= -1

    return get_location_polor(get_bearing(center, point) + \
                              angle, get_distance(center, point), center)

def rotate_points(center, points, angle, clockwise=False):
    """
        Rotates an array of points around a center counter-clockwise for a
        specified angle in degrees.
    """

    rotated_points = []

    for point in iterate(points):
        rotated_points.append(rotate_point(center, point, angle, clockwise))

    return rotated_points


def dist_till_wp():
    """
        Returns the distance to the next command i.e. next waypoint in meters.
    """

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

def arm_and_takeoff(takeoff_alt=2):
    """
        Standard Takeoff Procedure.
    """

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


def add_cmds(cmd_list):
    """
        Clears and Adds a cmd_list to the CMDS.
    """

    CMDS.clear()

    for cmd in cmd_list:
        CMDS.add(cmd)

    CMDS.upload()


def rectangle(height, angle=0, corner_location=VEHICLE.home_location, width=None):
    """
        Makes a cmd_list for a rectangular pattern.
    """

    cmd_list = []

    if width is None:
        width = height

    points = [get_location(height, 0, corner_location),
              get_location(height, width, corner_location),
              get_location(0, width, corner_location),
              corner_location]

    points = rotate_points(points[-1], points, angle)

    for point in points:
        cmd_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                                point.lat, point.lon, 2))

    return cmd_list

def polygon(center, radius, sides=8, counter_clockwise=False, entry_angle=0 ,test=0):
    """
        Makes a cmd_list for a circle pattern.
    """

    cmd_list = []
    loc_list = []

    angle = 360.0/sides

    shift = 0
    if counter_clockwise:
        shift = 1

    for i in xrange(sides):
        loc_list.append(get_location_polor(angle*(i+shift) + entry_angle, radius, center))

    loc_list = rotate_points(center, loc_list, test, True)

    for point in loc_list:
        cmd_list.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                                point.lat, point.lon, 2))

    if counter_clockwise:
        return cmd_list[::-1]
    else:
        return cmd_list


def deviate(dog_loc):
    """
        Deviation Procedure
    """

    next_wp = LocationGlobalRelative(CMD_LIST[CMDS.next].y,
                                     CMD_LIST[CMDS.next].x,
                                     CMD_LIST[CMDS.next].z)
    plus_one_wp = LocationGlobalRelative(CMD_LIST[CMDS.next+1].y,
                                         CMD_LIST[CMDS.next+1].x,
                                         CMD_LIST[CMDS.next+1].z)

    bearing = get_bearing(VEHICLE.location.global_relative_frame, next_wp)
    dog_bearing = get_bearing(VEHICLE.location.global_relative_frame, dog_loc)
    next_to_plus_bearing = get_bearing(next_wp, plus_one_wp)

    dev_bearing = (bearing-dog_bearing) % 360
    turn_bearing = (bearing+next_to_plus_bearing) % 360

    next_to_dog_bearing = get_bearing(next_wp, dog_loc)

    next_dist = get_distance(VEHICLE.location.global_relative_frame, next_wp)

    left = True
    if dev_bearing < 180:
        left = False

    quad_bearing = turn_bearing
    if turn_bearing > 180:
        quad_bearing -= 180

    if 65 > turn_bearing > 295: # maybe change later
        if quad_bearing:
            pass
    else:
        if next_to_dog_bearing < quad_bearing:
            dev_cmd_list = polygon(dog_loc, 25, 8, left)
            #dev_cmd_list.append()
        elif quad_bearing <= next_to_dog_bearing < 180:
            dev_cmd_list = polygon(dog_loc, 25, 8, left)
            #dev_cmd_list.append()
        elif 180 <= next_to_dog_bearing < quad_bearing+180:
            dev_cmd_list = polygon(dog_loc, 25, 8, left)
            #dev_cmd_list.append()
        else:
            dev_cmd_list = polygon(dog_loc, 25, 8, left)
            #dev_cmd_list.append()



    return dev_cmd_list


print "Clearing any existing commands..."
CMDS.clear()

CMD_LIST.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0,
                        0, 0, 2))

CMD_LIST.extend(rectangle(50))

CMD_LIST.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                        VEHICLE.home_location.lat, VEHICLE.home_location.lon, 2))
CMD_LIST.append(CMD_LIST[-1])

add_cmds(CMD_LIST)

print "Uploading new commands..."
CMDS.upload()


DOG_POINT = get_location(100, 30)

arm_and_takeoff()

VEHICLE.mode = VehicleMode("AUTO")  # Set mode to AUTO to start mission
VEHICLE.commands.next = 0 # Reset mission set to first (0) waypoint

print "Mission Started!"

while VEHICLE.system_status == 'ACTIVE':

    '''
    if get_distance(VEHICLE.location.global_relative_frame, DOG_POINT) < 35:
        deviate(DOG_POINT)
    '''

    if CMDS.next == CMDS.count:
        print "Exiting Standard Mission State..."
        break

    print 'Distance to waypoint {}: {}'.format(CMDS.next, dist_till_wp())
    time.sleep(1)

#print 'Returning to Launch...'
#VEHICLE.mode = VehicleMode("RTL")

print "Setting LAND mode..."
VEHICLE.mode = VehicleMode("LAND")

print "Close vehicle object"
VEHICLE.close()
