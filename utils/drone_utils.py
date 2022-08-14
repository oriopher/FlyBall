import numpy as np


def first_on_second_off(drone1, drone2):
    drone1.active, drone2.active = True, False
    print(drone1.ident, "active")


def reachability(distance, offset=0):
    # distance in cm, only one axis
    plot = np.array([[0, 0.85],
                     [2, 2],
                     [30, 2.76],
                     [50, 2.76],
                     [70, 2.93],
                     [90, 4.15]])

    for i in range(len(plot) - 1):
        if plot[i, 0] <= distance <= plot[i + 1, 0]:
            a = (plot[i + 1, 1] - plot[i, 1]) / (plot[i + 1, 0] - plot[i, 0])
            b = - a * plot[i, 0] + plot[i, 1]
            return a * distance + b + offset

    return plot[-1, 1]
