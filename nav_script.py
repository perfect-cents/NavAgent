"""
    Another Round...
"""

###############
### IMPORTS ###
###############

import argparse
import math
import time

from dronekit import Command, connect, LocationGlobal, LocationGlobalRelative,\
                     Vehicle, VehicleMode
from pymavlink import mavutil

######################################################
### INITIALIZATION # OF # VEHICLE/GLOBAL VARIABLES ###
######################################################

PARSER = argparse.ArgumentParser(description="Argumnt parser for the navscript")
PARSER.add_argument("--connect")
PARSER_ARGS = PARSER.parse_args()
CONNECTION_STRING = PARSER_ARGS.connect

print "Connecting to vehicle on: {}".format(CONNECTION_STRING)
VEHICLE = connect(CONNECTION_STRING, wait_ready=True, vehicle_class=Vehicle)

print "Setting Home Location to: {}".format(VEHICLE.location.global_frame)
VEHICLE.home_location = VEHICLE.location.global_frame

CMDS = VEHICLE.commands
LOC = VEHICLE.location

CMD_LIST = []
DOG_ID_LIST = []

#####################################
### LOCATION # FUNCTIONS ###
#####################################

def get_distance(from_location, to_location):
    """ Calulates the distance between the 'from_location' and the
        'to_location' location objects.
    """

    lat_diff = from_location.lat - to_location.lat
    lon_diff = from_location.lon - to_location.lon

    return (lat_diff**2 + lon_diff**2)**.5 * 1.113195e5

def get_bearing(from_location, to_location):
    """ Calulates the bearing from the 'from_location' to the 'to_location' in
        degrees starting from the north rotating clockwise.
    """

    x_diff = from_location.lon - to_location.lon
    y_diff = from_location.lat - to_location.lat

    return math.atan2(-y_diff, x_diff) * 57.295780 - 90

def get_dist_till_wp():
    """ Calculates the distance until the CMD.next waypoint.
    """

    if CMDS.next == 0:
        return None

    cmd = CMDS[CMDS.next-1]  # Commands are zero indexed
    lat = cmd.x
    lon = cmd.y
    alt = cmd.z
    waypoint = LocationGlobalRelative(lat, lon, alt)

    return get_distance(LOC.global_relative_frame, waypoint)


def get_ortho_location(meters_north, meters_east, from_location=LOC.global_relative_frame):
    """ Returns a location object 'meters_north' meters to the north and
        'meters_east' meters to the east from the 'from_location' location
        object.

        The return will be of the same class as the 'from_location' object i.e.
        a LocationGlobal 'from_location' will return a LocationGlobal object.
    """

    earth_radius = 6378137.0

    lat_diff = meters_north / earth_radius
    lon_diff = meters_east / earth_radius * \
               math.cos(math.pi * from_location.lat/180) * 1.22674

    new_lat = from_location.lat + (lat_diff * 180/math.pi)
    new_lon = from_location.lon + (lon_diff * 180/math.pi)
    new_alt = 10#from_location.alt  # TODO : Might change later

    if isinstance(from_location, LocationGlobal):
        new_location = LocationGlobal(new_lat, new_lon, new_alt)
    elif isinstance(from_location, LocationGlobalRelative):
        new_location = LocationGlobalRelative(new_lat, new_lon, new_alt)
    else:
        raise Exception("Invalid Location object passed")

    return new_location

def get_polor_location(bearing, dist, from_location=LOC.global_relative_frame):
    """ Returns a location 'dist' meters at 'bearing' degrees from the
        'from_location' location object.

        The 'bearing' parameter starts from the north and rotates clockwise.

        The return will be of the same class as the from_location object i.e.
        a LocationGlobal object will return a LocationGlobal object.
    """

    bearing = math.radians(360 - (bearing - 90) % 360)

    meters_north = math.sin(bearing) * dist  # TODO : FIX SKEW!!!!!!!
    meters_east = math.cos(bearing) * dist

    return get_ortho_location(meters_north, meters_east, from_location)


def rotate_point(center, point, angle, clockwise=True):
    """ Returns a location object of a location object 'point' rotated around a
        'center' location object 'angle' degrees.

        The 'clockwise' bool specifies the direction of rotation.
    """

    angle *= 1 if clockwise is True else -1

    return get_polor_location(get_bearing(center, point) + angle,
                                          get_distance(center, point), center)

def rotate_points(center, points, angle, clockwise=True):
    """ Returns a list of location objects of the list 'points' rotated around
        a 'center' location object 'angle' degrees.

        The 'clockwise' bool specifies the direction of rotation.
    """

    rotated_points = []

    for point in points:
        rotated_points.append(rotate_point(center, point, angle, clockwise))

    return rotated_points

def translate_point(point, bearing, dist):
    """ Returns a location object of a location object 'point' translated at
        'bearing' deg clockwise from due north 'dist' meters from 'point'.
    """

    return get_polor_location(bearing, dist, point)

def translate_points(point, bearing, dist):
    """ Returns a list of location objects from a list location objects 'points'
        translated at 'bearing' deg clockwise from due north 'dist' meters from
        'point'.
    """

    translated_points = []

    bearing = math.radians(360 - (bearing - 90) % 360)

    meters_north = math.sin(bearing) * dist  # TODO : FIX SKEW ditto!!!!!!!
    meters_east = math.cos(bearing) * dist

    for point in points:
        translate_points.append(get_ortho_location(meters_north, meters_east, point))

    return (translated_points)


###########################
### COMMAND # FUNCTIONS ###
###########################

def add_cmds(cmd_list):
    """ Clears the stack and puts a 'cmd_list' onto the command stack.
    """

    CMDS.clear()

    for cmd in cmd_list:
        CMDS.add(cmd)

    CMDS.upload()


def cmd_takeoff(alt):
    """ Returns a takeoff command at 'alt' height.
    """
    return Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                   mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0,
                   0, 0, alt)

def cmd_point(location):
    """ Returns a waypoint command at the location object's location.
    """

    return Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                   mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                   location.lat, location.lon, location.alt)

def loc_to_cmd(location, altitude_diff=0):
    """ Returns a waypoint command at the location object's location.
    
        The 'altitude_diff' will increase or decrease the altitude of all 
        commands by the amount.
    """

    return Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                   mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                   location.lat, location.lon, location.alt + altitude_diff)  # TODO CHANGE!!

def locs_to_cmds(loc_list, altitude_diff=0):
    """ Returns a list of waypoint commands at the locations of the location
        object's.
    
        The 'altitude_diff' will increase or decrease the altitude of all 
        commands by the amount.
    """
    cmd_list = []

    for loc in loc_list:
        cmd_list.append(loc_to_cmd(loc, altitude_diff))

    return cmd_list


def cmd_rectangle(height, width, angle=0, from_location=VEHICLE.home_location):
    """ Makes a cmd_list in a rectangular pattern.
    """

    cmd_list = []

    loc_list = [get_ortho_location(height, 0, from_location),
              get_ortho_location(height, width, from_location),
              get_ortho_location(0, width, from_location),
              from_location]

    loc_list = rotate_points(from_location, loc_list, angle)

    return locs_to_cmds(loc_list)

def cmd_polygon(center, radius, sides=8, clockwise=True, entry_angle=0):
    """ Makes a cmd_list in a circle pattern.
    """

    cmd_list = []
    loc_list = []

    angle = 360.0/sides

    shift = 0 if clockwise else 1

    entry_point = get_polor_location(entry_angle, radius, center)

    for i in range(sides):
        loc_list.append(rotate_point(center, entry_point, angle*(i + shift)))

    cmd_list = locs_to_cmds(loc_list)

    return cmd_list if clockwise else cmd_list[::-1]

#########################
### OTHER # FUNCTIONS ###
#########################

def arm():
    """ A simple arming procedure.
    """

    while not VEHICLE.is_armable:
        print "waiting for vehicle to become armable..."
        time.sleep(1)

    print "attempting to arm motors..."
    VEHICLE.mode = VehicleMode("GUIDED")  # copter should arm in guided mode
    VEHICLE.armed = True

    while not VEHICLE.armed:
        print "waiting for arming..."
        time.sleep(2)
        
    print "finished arming."

def takeoff(altitude):
    """ A simple default takeoff procedure.
    """

    print "taking off from ({},{})".format(VEHICLE.home_location.lat,
                                           VEHICLE.home_location.lon)
    VEHICLE.simple_takeoff(altitude)

    while True:
        print "altitude: ", LOC.global_relative_frame.alt
        if LOC.global_relative_frame.alt > altitude * .90:
            print "reached target altitude."
            break
        time.sleep(1)

def main():
    """ Main sequence
    """

    print "Clearing any existing commands..."
    CMDS.clear()

    CMD_LIST.append(cmd_takeoff(2))
    CMD_LIST.extend(cmd_rectangle(30, 100))
    CMD_LIST.append(loc_to_cmd(VEHICLE.home_location, 10))  # TODO : proper alt?
    #CMD_LIST.append(CMD_LIST[-1])

    add_cmds(CMD_LIST)

    print "Uploading new commands..."
    CMDS.upload()

    arm()

    takeoff(2)

    VEHICLE.mode = VehicleMode("AUTO")
    CMDS.next = 0

    print "Mission Started!"
    while VEHICLE.system_status == "ACTIVE":
        if CMDS.next == CMDS.count:
            print "Exiting Standard Mission State..."
            break

        print "to waypoint {}: {}".format(CMDS.next, get_dist_till_wp())
        time.sleep(2)

    print "Setting LAND mode..."
    VEHICLE.mode = VehicleMode("LAND")

    print "Close vehicle object"
    VEHICLE.close()

if __name__ == '__main__':
    main()
