""" Basic Demonstration of a Node Search.

"""

from string import ascii_lowercase
import itertools

POINT_LIST = [(8241.489, 2824.634, 27),
              (8245.800, 2814.630, 24),
              (8242.490, 2823.750, 27),
              (8242.549, 2825.760, 27),
              (8243.890, 2820.844, 40),
              (8245.100, 2816.967, 25),
              (8245.128, 2821.742, 28),
              (8246.349, 2817.047, 30),
              (8250.506, 2812.500, 25),
              (8250.740, 2811.267, 25),
              (8256.970, 2822.355, 39),
              (8256.980, 2822.298, 38),
              (8256.999, 2822.258, 36),
              (8256.999, 2822.458, 39),
              (8257.000, 2822.380, 40),
              (8257.005, 2822.300, 35),
              (8257.019, 2815.413, 11),
              (8257.191, 2816.683, 29),
              (8257.235, 2815.316, 20),
              (8257.394, 2815.148, 23)]

STARTING_POINT = (8250.256, 2819.426, 27)
ENDING_POINT = STARTING_POINT

class Node(object):
    """ A class that stores all of the Node's Data."""
    def __init__(self, lon, lat, alt):
        self._lon = lon
        self._lat = lat
        self._alt = alt

    @property
    def lon(self):
        return self._lon

    @property
    def lat(self):
        return self._lat

    @property
    def alt(self):
        return self._alt


def letter_gen():
    """ A char generator that uses ascii chars for a base 26 number sysetem."""
    size = 1
    while True:
        for char in itertools.product(ascii_lowercase, repeat=size):
            yield "".join(char)
        size += 1

NODES = {}

NODE_NAME_GEN = letter_gen()

for i in range(len(POINT_LIST)):
    NODES[NODE_NAME_GEN.next()] = Node(POINT_LIST[i][0], POINT_LIST[i][1], POINT_LIST[i][2])

NODES['start'] = Node(STARTING_POINT[0], STARTING_POINT[1], STARTING_POINT[2])


class Edge(object):
    def __init__(self, node1, node2):
        self._hist = []

        dist = ((node2.lon - node1.lon)**2 + (node2.lat - node1.lat)**2)**.5

        self._val = dist

    @property
    def val(self):
        return self._val

    @val.setter
    def val(self, val):
        raise Exception('INPUT_ERROR:',val,': Unable to set val.')

    @property
    def hist(self):
        return self._hist

    @hist.setter
    def hist(self, val):
        self._hist.append(val)

EDGES = {}


def edge_check(key1, key2):
    if key1 == key2:
        return False

    if key2 == 'start':
        EDGES[key1+'->'+'end'] = Edge(NODES[key1], NODES[key2])
        return False

    return True

for key1 in NODES:
    for key2 in NODES:
        if edge_check(key1, key2):
            EDGES[key1+'->'+key2] = Edge(NODES[key1], NODES[key2])


for k in EDGES:
    print k, ':', EDGES[k].val

OPTIMAL_ROUTE = []

def evaluate_route(route_list):
    vals = []

    for entry in route_list:
        vals.append(EDGES[entry].val)

    return sum(vals)

