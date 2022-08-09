from datetime import datetime
from prediction import NumericBallPredictor
import numpy as np
from common import reachability, FLOOR_HEIGHT, DRONE_DEFAULT_HEIGHT

MIN_SAFE_HEIGHT = FLOOR_HEIGHT + 30


class State:
    def next(self, state=1):
        raise NotImplemented

    def to_transition(self, drone, balloon, borders):
        raise NotImplemented

    def run(self, drone, balloon, borders):
        raise NotImplemented

    def setup(self, drone, balloon, borders):
        pass

    def cleanup(self, transition, drone, balloon, borders):
        pass


class ON_GROUND(State):
    def __str__(self):
        return "On Ground"

    def next(self, state=1):
        return HOVERING()

    def to_transition(self, drone, balloon, borders):
        return drone.tookoff and borders.set_borders

    def cleanup(self, transition, drone, balloon, borders):
        drone.takeoff()
        drone.stop()

    def run(self, drone, balloon, borders):
        return


class HOVERING(State):
    def __str__(self):
        return "Hovering"

    def next(self, state=1):
        return WAITING()

    def to_transition(self, drone, balloon, borders):
        return drone.start

    def run(self, drone, balloon, borders):
        return


class WAITING(State):
    def __str__(self):
        return "Waiting"

    def next(self, state=1):
        return STANDING_BY()

    def to_transition(self, drone, balloon, borders):
        if drone.testing:
            drone.testing = 0
            return 1
        return 0

    def run(self, drone, balloon, borders):
        drone.go_home()

class STANDING_BY(State):
    def __str__(self):
        return "Standing By"

    def next(self, state=1):
        return SEARCHING_PREDICTION()

    def to_transition(self, drone, balloon, borders):
        return borders.in_borders(balloon)

    def run(self, drone, balloon, borders):
        drone.go_home()


class SEARCHING_PREDICTION(State):
    Z_OFFSET = 50
    XY_VEL_BOUND = 30

    def __str__(self):
        return "Searching Prediction"

    def next(self, state=1):
        return SEARCHING() if state == 1 else STANDING_BY()

    def setup(self, drone, balloon, borders):
        drone.search_pred_start()

    def to_transition(self, drone, balloon, borders):
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

    def run(self, drone, balloon, borders):
        pred = NumericBallPredictor(balloon)
        pred_time, pred_coords = pred.get_optimal_hitting_point(z_bound=drone.z / 100,
                                                                xy_vel_bound=self.XY_VEL_BOUND / 100)

        if np.any(pred_coords):
            x_dest, y_dest, z_dest = drone.new_pred(pred_coords)
        else:
            x_dest, y_dest, z_dest = drone.dest_coords

        x_to_target = abs(x_dest - drone.drone_search_pred_coords[0])
        y_to_target = abs(y_dest - drone.drone_search_pred_coords[1])
        time_to_hit_from_start = max(reachability(x_to_target), reachability(y_to_target))
        time_until_hit = time_to_hit_from_start + (drone.drone_search_pred_time - datetime.now()).total_seconds()
        pred_time, pred_coords = pred.get_optimal_hitting_point(z_bound=drone.z / 100,
                                                                xy_vel_bound=self.XY_VEL_BOUND / 100,
                                                                start_time=time_until_hit)
        if not np.any(pred_coords):  # if pred_coords != (0,0,0)
            z_dest = pred_coords[2] - self.Z_OFFSET
            if z_dest < MIN_SAFE_HEIGHT:
                z_dest = MIN_SAFE_HEIGHT

        drone.track_3d(x_dest, y_dest, z_dest)


class SEARCHING(State):
    Z_OFFSET = 50

    def __str__(self):
        return "Searching"

    def next(self, state=1):
        return HITTING() if state == 1 else STANDING_BY()

    def to_transition(self, drone, balloon, borders):
        UPPER_LIMIT = 110
        LOWER_LIMIT = 20
        XY_LIMIT = 30
        VEL_LIMIT = 30

        x_rel = balloon.x - drone.x
        y_rel = balloon.y - drone.y
        z_rel = balloon.z - drone.z

        if z_rel < UPPER_LIMIT and balloon.vz <= 0:
            return 1
        if abs(x_rel) < XY_LIMIT and abs(y_rel) < XY_LIMIT and LOWER_LIMIT < z_rel < UPPER_LIMIT \
                and abs(drone.vx) < VEL_LIMIT and abs(drone.vy) < VEL_LIMIT \
                and balloon.vz <= 0:
            return 1
        if balloon.vz <= 0 and balloon.z <= drone.z:
            return 2
        if not (borders.in_borders(balloon) and borders.in_borders(drone)):
            return 2
        return 0

    def run(self, drone, balloon, borders):
        pred = NumericBallPredictor(balloon)
        x_dest, y_dest, z_dest = pred.get_prediction(reachability(distance=0, offset=0))
        z_dest = drone.z
        drone.track_3d(x_dest, y_dest, z_dest)


class HITTING(State):
    def __str__(self):
        return "Hitting"

    def next(self, state=1):
        return DESCENDING()

    def setup(self, drone, balloon, borders):
        drone.start_hit()

    def to_transition(self, drone, balloon, borders):
        Z_LIMIT = 15
        # XY_LIMIT = 40

        x_rel = balloon.x - drone.x
        y_rel = balloon.y - drone.y
        z_rel = balloon.z - drone.z

        # transition = not (abs(x_rel) < XY_LIMIT and abs(y_rel) < XY_LIMIT) or (z_rel < Z_LIMIT)
        transition = z_rel < Z_LIMIT
        return transition

    def run(self, drone, balloon, borders):
        time_since_hitting = (datetime.now() - drone.start_hit_timer).total_seconds()
        pred = NumericBallPredictor(balloon)
        x_dest, y_dest, z_dest = pred.get_prediction(reachability(0, 0) - time_since_hitting)

        drone.track_hitting(x_dest, y_dest, z_dest)


class DESCENDING(State):
    def __str__(self):
        return "Descending"

    def setup(self, drone, balloon, borders):
        drone.active = False

    def next(self, state=1):
        return WAITING()

    def to_transition(self, drone, balloon, borders):
        Z_OFFSET = 15
        return drone.z < DRONE_DEFAULT_HEIGHT + Z_OFFSET

    def run(self, drone, balloon, borders):
        drone.track_descending()
