"""
    Lift Off...
"""


import argparse
import math
import time

from dronekit import Command, connect, LocationGlobal, LocationGlobalRelative,\
                     Vehicle, VehicleMode
from pymavlink import mavutil


PARSER = argparse.ArgumentParser()
PARSER.add_argument("--connect")
PARSER.add_argument("--alt")
PARSER.add_argument("--hang")

PARSER_ARGS = PARSER.parse_args()

CONNECTION_STRING = PARSER_ARGS.connect
ALT = int(PARSER_ARGS.alt) if PARSER_ARGS.alt is not None else 1
HANG = int(PARSER_ARGS.hang) if PARSER_ARGS.hang is not None else 10

print "Connecting to vehicle on: {}".format(CONNECTION_STRING)
VEHICLE = connect(CONNECTION_STRING, wait_ready=True, vehicle_class=Vehicle)

print "Setting Home Location to: {}".format(VEHICLE.location.global_frame)
VEHICLE.home_location = VEHICLE.location.global_frame


def arm():
    """ A simple arming procedure.
    """

    while not VEHICLE.is_armable:
        print " waiting for vehicle to become armable..."
        time.sleep(1)

    print "attempting to arm motors..."
    VEHICLE.mode = VehicleMode("GUIDED")  # copter should arm in guided mode
    VEHICLE.armed = True

    while not VEHICLE.armed:
        print "waiting for arming..."
        time.sleep(1)


def takeoff(altitude):
    """ A simple default takeoff procedure.
    """

    print "taking off from ({},{})".format(VEHICLE.home_location.lat,
                                           VEHICLE.home_location.lon)
    VEHICLE.simple_takeoff(altitude)

    while True:
        print "altitude: ", VEHICLE.location.global_relative_frame.alt
        if VEHICLE.location.global_relative_frame.alt > altitude * .90:
            print "Reached target altitude."
            break
        time.sleep(1)


arm()

takeoff(ALT)


for i in range(HANG):
    print "hang time:{:>3}:{:<2}".format((HANG - i) / 60, (HANG - i) % 60)
    time.sleep(1)


print "Setting LAND mode..."
VEHICLE.mode = VehicleMode("LAND")

print "Close vehicle object"
VEHICLE.close()
