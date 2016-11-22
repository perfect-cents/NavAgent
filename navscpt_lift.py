"""A simple liftoff NavAgent Script for copter.

"""


import argparse
import time

from dronekit import connect, Vehicle, VehicleMode


PARSER = argparse.ArgumentParser(description="External Argument Parser for NavAgent.")
PARSER.add_argument("--connect",
                    help=("vehicle connection target string."))

PARSER.add_argument("--alt",
                    help=("vehicle takeoff_alt."))

PARSER_ARGS = PARSER.parse_args()

CONNECTION_STRING = PARSER_ARGS.connect
TAKEOFF_ALT = int(PARSER_ARGS.alt)


print "Connecting to vehicle on: %s" % CONNECTION_STRING
VEHICLE = connect(CONNECTION_STRING, wait_ready=True, vehicle_class=Vehicle)

print "Setting Home Location..."
VEHICLE.home_location = VEHICLE.location.global_frame

CMDS = VEHICLE.commands
CMD_LIST = []

def arm_and_takeoff(takeoff_alt):
    print "basic pre-arm checks..."
    # don't let the user try to arm until autopilot is ready

    VEHICLE.armed = True

    while not VEHICLE.is_armable:
        print " waiting for vehicle to initialise..."
        time.sleep(5)

    print "arming motors..."
    # copter should arm in guided mode
    VEHICLE.mode = VehicleMode("GUIDED")


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
    CMDS.clear()

    for cmd in cmd_list:
        CMDS.add(cmd)

    CMDS.upload()

print "Clearing any existing commands..."
CMDS.clear()

arm_and_takeoff(TAKEOFF_ALT)

print "Mission Started!"

for i in range(5):
    print 'Waiting...'
    time.sleep(2)

#print 'Returning to Launch...'
#VEHICLE.mode = VehicleMode("RTL")

print "Setting LAND mode..."
VEHICLE.mode = VehicleMode("LAND")

print "Close vehicle object"
VEHICLE.close()
