from datetime import datetime
import numpy as np

from state_machine.loop_state_machine import State2Drones
from prediction.prediction import NumericBallPredictor
from utils.common import reachability, first_on_second_off
from utils.consts import DRONE_MIN_HEIGHT


# INVARIANT: at all time one drone is active and one is inactive.

class ON_GROUND(State2Drones):
    def __str__(self):
        return "On Ground"

    def setup(self, drone, other_drone, balloon, borders):
        drone.active = drone.ident < other_drone.ident  # arbitrarily choose who is active and who is passive

    def next(self, state=1):
        return HOVERING()

    def to_transition(self, drone, other_drone, balloon, borders):
        return drone.tookoff and borders.is_set

    def cleanup(self, transition, drone, other_drone, balloon, borders):
        drone.takeoff()
        drone.stop()

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
        drone.stop()
        return


class WAITING(State2Drones):
    def __str__(self):
        return "Waiting"

    def next(self, state=1):
        return STANDING_BY()

    def setup(self, drone, other_drone, balloon, borders):
        if drone.ident < other_drone.ident:
            first_on_second_off(drone, other_drone)

    def to_transition(self, drone, other_drone, balloon, borders):
        if drone.testing:
            drone.testing = 0
            return 1
        return 0

    def run(self, drone, other_drone, balloon, borders):
        drone.go_home(other_drone.obstacle)


class STANDING_BY(State2Drones):
    XY_VEL_BOUND = 30

    def __str__(self):
        return "Standing By"

    def next(self, state=1):
        return SEARCHING_PREDICTION() if state == 1 else PREPARE_AND_AVOID()

    # def to_transition(self, drone, other_drone, balloon, borders):
    #     if not borders.in_borders(balloon):
    #         return 0
    #     pred = NumericBallPredictor(balloon)
    #     pred_time, pred_coords = pred.get_optimal_hitting_point(z_bound=drone.z / 100,
    #                                                         xy_vel_bound=self.XY_VEL_BOUND / 100)                                                    

    #     x_rel = pred_coords[0] - drone.x
    #     y_rel = pred_coords[1] - drone.y
    #     x_rel_other = pred_coords[0] - other_drone.x
    #     y_rel_other = pred_coords[1] - other_drone.y

    #     if (x_rel**2 + y_rel**2) == (x_rel_other**2 + y_rel_other**2):
    #         return 1 if drone.active else 2
    #     elif (x_rel**2 + y_rel**2) < (x_rel_other**2 + y_rel_other**2):
    #         first_on_second_off(drone, other_drone)
    #         return 1
    #     else:
    #         first_on_second_off(other_drone, drone)
    #         return 2

    def to_transition(self, drone, other_drone, balloon, borders):
        if not borders.in_borders(balloon):
            return 0
        if drone.active:
            return 1
        return 2

    def run(self, drone, other_drone, balloon, borders):
        drone.go_home(other_drone.obstacle)


class SEARCHING_PREDICTION(State2Drones):
    Z_OFFSET = 50
    XY_VEL_BOUND = 30

    def __str__(self):
        return "Searching Prediction"

    def next(self, state=1):
        return SEARCHING() if state == 1 else WAITING()

    def setup(self, drone, other_drone, balloon, borders):
        drone.search_pred_start()

    def to_transition(self, drone, other_drone, balloon, borders):
        if np.sqrt(balloon.vx ** 2 + balloon.vy ** 2) <= self.XY_VEL_BOUND \
                and balloon.z >= drone.z:
            return 1
        if balloon.vz <= 0 and balloon.z >= drone.z:
            return 1
        if balloon.vz <= 0 and balloon.z <= drone.z:
            return 2
        if not (borders.in_borders(balloon) and borders.in_borders(drone)):
            return 2
        return 0

    def run(self, drone, other_drone, balloon, borders):
        pred = NumericBallPredictor(balloon)
        pred_time, pred_coords = pred.get_optimal_hitting_point(z_bound=drone.z / 100,
                                                                xy_vel_bound=self.XY_VEL_BOUND / 100)

        if np.any(pred_coords):
            x_dest, y_dest, z_dest = drone.new_pred(pred_coords)
        else:
            x_dest, y_dest, z_dest = drone.dest_coords

        x_to_target = abs(x_dest - drone.drone_search_pred_coords[0])
        y_to_target = abs(y_dest - drone.drone_search_pred_coords[1])
        time_to_hit_from_start = max(reachability(x_to_target, offset=0.6), reachability(y_to_target, offset=0.6))
        time_until_hit = time_to_hit_from_start + (drone.drone_search_pred_time - datetime.now()).total_seconds()
        pred_time, pred_coords = pred.get_optimal_hitting_point(z_bound=drone.z / 100,
                                                                xy_vel_bound=self.XY_VEL_BOUND / 100,
                                                                start_time=time_until_hit)
        if not np.any(pred_coords):  # if pred_coords != (0,0,0)
            z_dest = pred_coords[2] - self.Z_OFFSET
            if z_dest < DRONE_MIN_HEIGHT:
                z_dest = DRONE_MIN_HEIGHT

        drone.track_3d(x_dest, y_dest, z_dest)


class SEARCHING(State2Drones):
    Z_OFFSET = 50

    def __str__(self):
        return "Searching"

    def next(self, state=1):
        return HITTING() if state == 1 else WAITING()

    def to_transition(self, drone, other_drone, balloon, borders):
        UPPER_LIMIT = 110
        Z_LIMIT = 50

        pred = NumericBallPredictor(balloon)
        _, _, z_dest = pred.get_prediction(reachability(distance=0))

        z_rel = z_dest - drone.z

        if z_rel < Z_LIMIT and balloon.z >= drone.z and balloon.vz <= 0:
            return 1

        z_rel = balloon.z - drone.z

        if z_rel < UPPER_LIMIT and balloon.vz <= 0:
            return 1
        if balloon.vz <= 0 and balloon.z <= drone.z:
            return 2
        if not (borders.in_borders(balloon) and borders.in_borders(drone)):
            return 2
        return 0

    def run(self, drone, other_drone, balloon, borders):
        pred = NumericBallPredictor(balloon)
        x_dest, y_dest, z_dest = pred.get_prediction(reachability(distance=0))
        z_dest = drone.z
        drone.track_3d(x_dest, y_dest, z_dest)


class HITTING(State2Drones):
    def __str__(self):
        return "Hitting"

    def next(self, state=1):
        return DESCENDING()

    def setup(self, drone, other_drone, balloon, borders):
        drone.start_hit()

    def cleanup(self, transition, drone, other_drone, balloon, borders):
        first_on_second_off(other_drone, drone)

    def to_transition(self, drone, other_drone, balloon, borders):
        Z_LIMIT = 20

        z_rel = balloon.z - drone.z

        transition = z_rel < Z_LIMIT
        return transition

    def run(self, drone, other_drone, balloon, borders):
        time_since_hitting = (datetime.now() - drone.start_hit_timer).total_seconds()
        pred = NumericBallPredictor(balloon)
        x_dest, y_dest, z_dest = pred.get_prediction(reachability(0) - time_since_hitting)

        drone.track_hitting(x_dest, y_dest, z_dest)


class DESCENDING(State2Drones):
    def __str__(self):
        return "Descending"

    def next(self, state=1):
        return PREPARE_AND_AVOID()

    def to_transition(self, drone, other_drone, balloon, borders):
        Z_OFFSET = 30
        return drone.z <= other_drone.z + Z_OFFSET

    def run(self, drone, other_drone, balloon, borders):
        drone.track_descending(other_drone.obstacle)
        # drone.track_descending_2drones(other_drone)


class PREPARE_AND_AVOID(State2Drones):
    def __str__(self):
        return "Prepare and Avoid"

    def next(self, state=1):
        return SEARCHING_PREDICTION() if state == 1 else WAITING()

    def to_transition(self, drone, other_drone, balloon, borders):
        if drone.active and (
                balloon.vz > 0 or balloon.z <= other_drone.z):  # drone is active only after other drone hits
            return 1
        if isinstance(other_drone.state, WAITING):
            return 2
        return 0

    def run(self, drone, other_drone, balloon, borders):
        HEIGHT_PREPARATION_FACTOR = 0.5

        if not drone.active:
            x_dest, y_dest = other_drone.obstacle.get_preparation_dest()
            z_dest = DRONE_MIN_HEIGHT
        else:  # this occurs only when the other drone finished the hitting stage
            x_dest, y_dest, z_dest = drone.dest_coords

        drone.track_3d(x_dest, y_dest, z_dest, other_drone.obstacle)
