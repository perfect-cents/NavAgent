"""A base template for NavAgent Scripts.

"""


import argparse

from custom_vehicle import CustomVehicle

from dronekit import connect
import nav

def main():
    """Main Doc String.

    """
    parser = argparse.ArgumentParser(description='External Argument Parser for NavAgent.')
    parser.add_argument('--connect',
                        help=("vehicle connection target string. "
                              "If not specified, SITL automatically started and used."))

    args = parser.parse_args()

    connection_string = args.connect
    connection_string = 'udp:127.0.0.1:14550'   # Essentialy a lazy thing to do

    '''

    sitl = None

    if not connection_string: # Start SITL if no connection string specified
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    '''

    print 'Connecting to vehicle on: %s' % connection_string
    vehicle = connect(connection_string, wait_ready=True, vehicle_class=CustomVehicle)



if __name__ == "__main__":
    #
    #
    main()






