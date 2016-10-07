""" A parser that makes a cmd_list out of a given json file

"""

import json
from dronekit import Command
from pymavlink import mavutil

def json_cmds(json_path):
    with open(json_path) as json_file:
        json_string = json_file.read()
        json_file.close()

    json_string = """
    {
        "0": {
            "lon": "32.679",
            "lat": "12.345",
            "alt": "10",
            "misson": "1",
            "data": "lots"
        },
        "1": {
            "lon": "32.679",
            "lat": "12.345",
            "alt": "10",
            "misson": "1",
            "data": "lots"
        },
        "2": {
            "lon": "32.679",
            "lat": "12.345",
            "alt": "10",
            "misson": "1",
            "data": "a little"
        },
        "3": {
            "lon": "32.679",
            "lat": "12.345",
            "alt": "10",
            "misson": "2",
            "data": "medium amount"
        }
    }"""

    json_dict = json.loads(json_string)

    cmd_list = []

    for key in range(len(json_dict)):
        cmd_list.append((Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                                 json_dict[str(key)]['lat'],
                                 json_dict[str(key)]['lon'],
                                 json_dict[str(key)]['alt'])),

                        json_dict[str(key)]['mission'],
                        json_dict[str(key)]['data'])

    return cmd_list

if __name__ == '__main__':
    main()
