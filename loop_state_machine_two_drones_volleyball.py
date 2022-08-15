from datetime import datetime
import numpy as np

from loop_state_machine import State2Drones
from prediction import NumericBallPredictor
from utils.drone_utils import reachability, first_on_second_off
from utils.consts import DRONE_MIN_HEIGHT


# INVARIANT: at all time one drone is active and one is inactive.

class ON_GROUND(State2Drones):
    """
    A State representing the drone is on ground, and waiting for the user to command the drone to take off.
    Transitions to HOVERING.
    """
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
    """
    A State representing the drone is hovering right after takeoff,
    and waiting for the user to command the drone to start movement.
    Transitions to WAITING.
    """
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
    """
    A State representing the drone is waiting for the user to start the game,
    in its home (and move to it, drone 2 is set to be passive and avoid drone 1).
    Transitions to STANDING_BY.
    """
    def __str__(self):
        return "Waiting"

    def next(self, state=1):
        return STANDING_BY()

    def setup(self, drone, other_drone, balloon, borders):
        if drone.ident < other_drone.ident:
            first_on_second_off(drone, other_drone)

    def to_transition(self, drone, other_drone, balloon, borders):
        if drone.game_start:
            drone.game_start = 0
            return 1
        return 0

    def run(self, drone, other_drone, balloon, borders):
        drone.go_home(other_drone.obstacle)


class STANDING_BY(State2Drones):
    """
    A State representing the drone is standing by in its home and waiting for the balloon to enter the borders.
    Transitions to SEARCHING_PREDICTION if the drone is active, and to PREPARE_AND_AVOID if it's passive.
    """
    XY_VEL_BOUND = 30

    def __str__(self):
        return "Standing By"

    def next(self, state=1):
        return SEARCHING_PREDICTION() if state == 1 else PREPARE_AND_AVOID()

    def to_transition(self, drone, other_drone, balloon, borders):
        if not borders.in_borders(balloon):
            return 0
        if drone.active:
            return 1
        return 2

    def run(self, drone, other_drone, balloon, borders):
        drone.go_home(other_drone.obstacle)


class SEARCHING_PREDICTION(State2Drones):
    """
    A State representing the drone is searching and moving to the optimal hitting point prediction in the xy plain.
    Transitions to Searching state when the balloon is slow enough.
    or to WAITING if the balloon dropped bellow the drone, or exited the borders.
    """
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
    """
    A State representing the drone is tracking to the reachable prediction of the balloon in the xy plain.
    Transitions to HITTING state when the balloon is low enough.
    or to WAITING if the balloon dropped bellow the drone, or exited the borders.
    """
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
    """
    A State representing the drone is tracking to hit the balloon.
    Transitions to DESCENDING state right before hitting the balloon and switching the activity of the drones.
    """
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
    """
    A State representing the drone is decelerating its upward trajectory and
    descending to a default height while avoiding the other drone.
    Transitions to PREPARE_AND_AVOID when the drone is low enough.
    """
    def __str__(self):
        return "Descending"

    def next(self, state=1):
        return PREPARE_AND_AVOID()

    def to_transition(self, drone, other_drone, balloon, borders):
        Z_OFFSET = 30
        return drone.z <= other_drone.z + Z_OFFSET

    def run(self, drone, other_drone, balloon, borders):
        drone.track_descending(other_drone.obstacle)


class PREPARE_AND_AVOID(State2Drones):
    """
    A state representing the drone is passive and moving to the preparation location (while avoiding the other drone).
    Transitions to SEARCHING_PREDICTION if the the drone switched to being active
    and the balloon is lower than the other drone, while moving upward,
    or to WAITING if the other drone state transitioned to the WAITING state.
    """
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
        if not drone.active:
            x_dest, y_dest = other_drone.obstacle.get_preparation_dest()
            z_dest = DRONE_MIN_HEIGHT
        else:  # this occurs only when the other drone finished the hitting stage
            x_dest, y_dest, z_dest = drone.dest_coords

        drone.track_3d(x_dest, y_dest, z_dest, other_drone.obstacle)
