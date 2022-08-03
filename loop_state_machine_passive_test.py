from datetime import datetime
from prediction import NumericBallPredictor
import numpy as np
from common import reachability, FLOOR_HEIGHT, DRONE_DEFAULT_HEIGHT

MIN_SAFE_HEIGHT = FLOOR_HEIGHT + 30
X_DEST = 70
Y_DEST = 300


class State:
    def next(self, state=1):
        raise NotImplemented

    def to_transition(self, drone, other_drone, balloon, borders):
        raise NotImplemented

    def run(self, drone, other_drone, balloon, borders):
        raise NotImplemented

    def setup(self, drone, other_drone, balloon, borders):
        pass

    def cleanup(self, transition, drone, other_drone, balloon, borders):
        pass


class ON_GROUND(State):
    def __str__(self):
        return "On Ground"

    def next(self, state=1):
        return HOVERING()

    def to_transition(self, drone, other_drone, balloon, borders):
        return drone.tookoff

    def run(self, drone, other_drone, balloon, borders):
        return


class HOVERING(State):
    def __str__(self):
        return "Hovering"

    def next(self, state=1):
        return WAITING()

    def to_transition(self, drone, other_drone, balloon, borders):
        return drone.start

    def run(self, drone, other_drone, balloon, borders):
        return


class WAITING(State):
    def __str__(self):
        return "Waiting"

    def next(self, state=1):
        return STANDING_BY()

    def to_transition(self, drone, other_drone, balloon, borders):
        return drone.testing

    def run(self, drone, other_drone, balloon, borders):
        if borders.set_borders:
            drone.seek_middle()
        else:
            x_dest, y_dest = drone.x_0, drone.y_0
            drone.track_2d(x_dest, y_dest)


class STANDING_BY(State):
    def __str__(self):
        return "Standing By"

    def next(self, state=1):
        return WAITING()

    def to_transition(self, drone, other_drone, balloon, borders):
        return 1 - drone.testing

    def run(self, drone, other_drone, balloon, borders):
        x_dest, y_dest = X_DEST, Y_DEST
        drone.track_2d(x_dest, y_dest)

