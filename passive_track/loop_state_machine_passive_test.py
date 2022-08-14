from utils.consts import FLOOR_HEIGHT
from state_machine.loop_state_machine import State2Drones

MIN_SAFE_HEIGHT = FLOOR_HEIGHT + 30
X_DEST = 90
Y_DEST = 350
X_START = 40
Y_START = 230


class ON_GROUND(State2Drones):
    def __str__(self):
        return "On Ground"

    def next(self, state=1):
        return HOVERING()

    def to_transition(self, drone, other_drone, balloon, borders):
        return drone.tookoff

    def run(self, drone, other_drone, balloon, borders):
        return


class HOVERING(State2Drones):
    def __str__(self):
        return "Hovering"

    def next(self, state=1):
        return WAITING()

    def to_transition(self, drone, other_drone, balloon, borders):
        return drone.start

    def run(self, drone, other_drone, balloon, borders):
        return


class WAITING(State2Drones):
    def __str__(self):
        return "Waiting"

    def next(self, state=1):
        return STANDING_BY()

    def to_transition(self, drone, other_drone, balloon, borders):
        return drone.testing

    def run(self, drone, other_drone, balloon, borders):
        x_dest, y_dest = X_START, Y_START
        drone.track_2d(x_dest, y_dest)


class STANDING_BY(State2Drones):
    def __str__(self):
        return "Standing By"

    def next(self, state=1):
        return WAITING()

    def to_transition(self, drone, other_drone, balloon, borders):
        return 1 - drone.testing

    def run(self, drone, other_drone, balloon, borders):
        obstacle = other_drone.obstacle
        x_dest, y_dest = obstacle.get_preparation_dest()
        drone.track_2d(x_dest, y_dest, obstacle)
