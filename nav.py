####    Modules    ####
import math
from dronekit import Command, LocationGlobal, LocationGlobalRelative, VehicleMode
#from pymavlink import mavutil

def arm_and_takeoff(aTargetAltitude): # Arms drone and takes of to target alt.
    print "Basic pre-arm checks..."
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors..."
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print "Waiting for arming..."
        time.sleep(1)

    print "Taking off from (%s, %s)" % (vehicle.home_location.lat, vehicle.home_location.lon)
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    """
    Wait until the vehicle reaches a safe height before processing the goto
    (else the command after vehicle.simple_takeoff will execute immediately).
    """
    while True:
        print "Altitude: ", vehicle.location.global_relative_frame.alt
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude-1: # Trigger just below target alt.
            print "Reached target altitude"
            break
        time.sleep(1)


####    Location Methods    ####
def get_location_metres(original_location, metres_north, metres_east):
    """ Input the a global_frame or a global_relative_frame, and offsets in metres.
    Returns a new global_frame or global_relative_frame.
    """

    earth_radius = 6378137.0 #Radius of "spherical" earth

    lat_dist = metres_north/earth_radius
    lon_dist = metres_east/(earth_radius*math.cos(math.pi*original_location.lat/180))

    new_lat = original_location.lat + (lat_dist * 180/math.pi)
    new_lon = original_location.lon + (lon_dist * 180/math.pi)

    if isinstance(original_location) is LocationGlobal:
        target_location = LocationGlobal(new_lat, new_lon, original_location.alt)
    elif isinstance(original_location) is LocationGlobalRelative:
        target_location = LocationGlobalRelative(new_lat, new_lon, original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return target_location

def get_distance_metres(a_location, b_location):
    """ Input two global_frame or global_relative_frames.
    Returns the  as a float.
    """

    dlat = b_location.lat - a_location.lat
    dlong = b_location.lon - a_location.lon

    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

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

#### Closing Sequence    ####
def closing_seq(vehicle, sitl):
    #print 'Returning to Launch...'
    #vehicle.mode = VehicleMode("RTL")

    print "Setting LAND mode..."
    vehicle.mode = VehicleMode("LAND")

    print "Close vehicle object"
    vehicle.close()

    if sitl is not None:
        sitl.stop()
