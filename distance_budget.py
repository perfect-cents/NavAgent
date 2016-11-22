"""
    A simple distance budget idea.
"""



#import random
import sys
#import time

from collections import namedtuple

BAT_CHART_RAW = \
"""percent_charge voltage_12V voltage_24V specific_gravity
1.0 12.70   25.40   1.265
.95  12.64   25.25   1.257
.90  12.58   25.16   1.249
.85  12.52   25.04   1.241
.80  12.46   24.92   1.233
.75  12.40   24.80   1.225
.70  12.36   24.72   1.218
.65  12.32   24.64   1.211
.60  12.28   24.56   1.204
.55  12.24   24.48   1.197
.50  12.20   24.40   1.190
.45  12.16   24.32   1.183
.40  12.12   24.24   1.176
.35  12.08   24.16   1.169
.30  12.04   24.08   1.162
.25  12.00   24.00   1.155
.20  11.98   23.96   1.148
.15  11.96   23.92   1.141
.10  11.94   23.88   1.134
.5   11.92   23.84   1.127"""

def make_soc(raw_text, sort_by='specific_gravity'):
    battary_chart = []
    lines = raw_text.split('\n')

    SOC = namedtuple('SOC', lines.pop(0).split())

    for line in lines:
        battary_chart.append(SOC(*line.split()))

    return sorted(battary_chart, key=lambda x:getattr(x, sort_by))

BAT_12V = {11.98: 85, 12.52: 20, 12.28: 45, 12.04: 75, 12.32: 40, 12.58: 15,
           12.0 : 80, 12.36: 35, 12.12: 65, 12.64: 10, 12.4 : 30, 12.16: 60,
           11.96: 90, 12.2 : 55, 11.94: 95, 12.7 : 5 , 12.46: 25, 12.08: 70,
           12.24: 50, 11.92: 100}

def get_battary_val(voltage, battary=BAT_12V):
    """
        gets battary values for a specific voltage.
    """

    lower_voltages = []
    higher_voltages = []

    for k in battary:
        if k >= voltage:
            higher_voltages.append(k)
        else:
            lower_voltages.append(k)

    high_voltage = higher_voltages.sort()[0]
    low_voltage = lower_voltages.sort()[-1]

    percent_diff = (voltage - low_voltage)/(high_voltage - low_voltage)

    state_of_charge = percent_diff * (battary[high_voltage] - battary[low_voltage])\
                    + battary[low_voltage]

    return state_of_charge


class Vehicle(object):
    """
        Temp Vehicle Class.
    """

    def __init__(self):
        self.ground_speed = 15
        self.air_speed = self.ground_speed

    @property
    def ground_speed(self):
        return self.ground_speed

    @property
    def air_speed(self):
        return self.air_speed



    '''
    def __repr__(self):
        pass
    '''

    def __str__(self):
        pass


def main():
    """
        Main Function
    """

    args = sys.argv[1:]

    print args


if __name__ == '__main__':
    main()


    '''
        from bisect import bisect_left


        def take_closest(my_list, my_number, sorted_=False):
            """
                If two numbers are equally close it'll return the smallest number.
            """

            if not sorted_:
                my_list = sorted(my_list)

            pos = bisect_left(my_list, my_number)

            if pos == 0:
                return my_list[0]
            if pos == len(my_list):
                return my_list[-1]

            before = my_list[pos - 1]
            after = my_list[pos]

            if after - my_number < my_number - before:
                return after
            else:
                return before
    '''
