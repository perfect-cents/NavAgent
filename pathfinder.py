import math

import itertools
from string import ascii_lowercase

class Square(object):
    def __init__(self, loc, type_='marked'):
        assert (isinstance(loc, tuple)), 'INVALID_LOC'
        self._loc = loc

        assert (type_ in ['mk', 'um', 'ng']), 'INVALID_TYPE'
        self._type_ = type_

        self._hist = []

    @property
    def type_(self):
        return self._type_

    @type_.setter
    def type_(self, value):
        raise Exception('TYPE_NOT_SETTABLE')

    @property
    def loc(self):
        return self._loc

    @loc.setter
    def loc(self, value):
        raise Exception('LOC_NOT_SETTABLE')

    @property
    def hist(self):
        return self._hist

    @hist.setter
    def hist(self, entry):
        if isinstance(entry, basestring) and len(entry) < 2:
            self._hist.append(entry)
        else:
            raise Exception('INVALID_ENTRY')


DUMMY_LOC = [[(0, 0, 10), (1, 0, 10), (2, 0, 10), (3, 0, 10)],
             [(0, 1, 10), (1, 1, 10), (2, 1, 10), (3, 1, 10)],
             [(0, 2, 10), (1, 2, 10), (2, 2, 10), (3, 2, 10)]]

DUMMY_TYPE = [['um', 'um', 'um', 'mk'],
              ['mk', 'um', 'mk', 'ng'],
              ['mk', 'mk', 'mk', 'mk']]

def letter_gen():
    size = 1
    while True:
        for chars in itertools.product(ascii_lowercase, repeat=size):
            yield "".join(chars)
        size +=1
'''
def to_meters():
    earth_radius

    lon = lon2 - lon1
    dlat = lat2 - lat1
    a = (sin(dlat/2))^2 + cos(lat1) * cos(lat2) * (sin(dlon/2))^2
    c = 2 * atan2( sqrt(a), sqrt(1-a)  )
    d = earth_radius * c
'''

class Grid(object):
    def __init__(self, dim):
        self._dim = dim

        self._grid = [[] for _ in range(self._dim[0])]
        self._grid_id = [[] for _ in range(self._dim[0])]
        self._marked_nodes = []

        self.initialize_grid()

        self._essential_connections = {}
        self._connections = {}

        self.initialize_connections()


    def initialize_grid(self):
        for row in range(self._dim[0]):
            for col in range(self._dim[1]):
                self._grid[row].append(Square(self.get_loc(row, col), self.get_type(row, col)))
                self._grid_id[row].append(str(row)+':'+str(col))
                if self._grid[row][col].type_ == 'mk':
                    self._marked_nodes.append(str(row)+':'+str(col))

            #for col in itertools.islice(letter_gen(), self._dim[1]):
            #    self._grid_id[row].append(str(row)+':'+col)

    def get_loc(self, row, col):
        return DUMMY_LOC[row][col]

    def get_type(self, row, col):
        return DUMMY_TYPE[row][col]

    def initialize_connections(self):
        all_connections = {}

        def all_connection_update(row, col, r_off, c_off):
                     all_connections.update({self._grid_id[row][col]+'->'+self._grid_id[row+r_off][col+c_off] : None})

        all_list = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]

        for row in range(self._dim[0]):
            for col in range(self._dim[1]):
                for dir_ in all_list:
                    if row+dir_[0] >= 0 and col+dir_[1] >= 0:
                        try:
                            all_connection_update(row, col, dir_[0], dir_[1])
                        except:
                            pass

        for key in all_connections:
            if key not in self._connections and self.nogo_check(key):
                new_val = self.make_value(key)
                self._connections.update({key : new_val})

        for key in self._connections:
            if key not in self._essential_connections and self.marked_check(key):
                self._essential_connections.update({key : self.connections[key]})


    def nogo_check(self, key):
        id_ = key.split('->')
        id_[0] = id_[0].split(':')
        id_[1] = id_[1].split(':')

        if self._grid[int(id_[0][0])][int(id_[0][1])].type_ == 'ng' or self._grid[int(id_[1][0])][int(id_[1][1])].type_ == 'ng':
            return False
        else:
            return True

    def marked_check(self, key):
        id_ = key.split('->')
        id_[0] = id_[0].split(':')
        id_[1] = id_[1].split(':')

        if self._grid[int(id_[0][0])][int(id_[0][1])].type_ == 'mk' and self._grid[int(id_[1][0])][int(id_[1][1])].type_ == 'mk':
            return True
        else:
            return False

    def make_value(self, key):
        id_ = key.split('->')
        id_[0] = id_[0].split(':')
        id_[1] = id_[1].split(':')

        src = self._grid[int(id_[0][0])][int(id_[0][1])]
        tar = self._grid[int(id_[1][0])][int(id_[1][1])]

        dist = ((tar.loc[0] - src.loc[0])**2 +
                (tar.loc[1] - src.loc[1])**2 +
                (tar.loc[2] - src.loc[2])**2)**.5

        angle = 0

        return src.type_+'->'+tar.type_


    @property
    def grid(self):
        return self._grid

    @property
    def marked_nodes(self):
        return self._marked_nodes

    @property
    def connections(self):
        return self._connections

    @property
    def econnections(self):
        return self._essential_connections

    def print_type(self):
        print('\n'.join([''.join(['{:10}'.format(item.type_) for item in row]) for row in self._grid]))

    def print_id(self):
        print('\n'.join([''.join(['{:10}'.format(item) for item in row]) for row in self._grid_id]))

class Connection():
    def __init__(self, dist, alt_diff):
        pass




def path_finder():
    pass

if __name__ == "__main__":
    DIM = (3, 4)
    g = Grid(DIM)

    for k, v in g.econnections.iteritems():
        print k, ':', v
    print
    for k, v in g.connections.iteritems():
        print k, ':', v
