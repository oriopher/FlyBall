from datetime import datetime
from pickle import FALSE
from prediction import NumericBallPredictor
import numpy as np
from common import reachability, first_on_second_off, FLOOR_HEIGHT, DRONE_DEFAULT_HEIGHT

MIN_SAFE_HEIGHT = FLOOR_HEIGHT + 30

# INVARIANT: at all time one drone is active and one is inactive.
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

    def setup(self, drone, other_drone, balloon, borders):
        drone.active = drone.ident < other_drone.ident # arbitrarly choose who is active and who is passive

    def next(self, state=1):
        return HOVERING()

    def to_transition(self, drone, other_drone, balloon, borders):
        return drone.tookoff and borders.set_borders
    
    def cleanup(self, transition, drone, other_drone, balloon, borders):
        drone.takeoff()
        drone.stop()

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
        drone.stop()
        return


class WAITING(State):
    def __str__(self):
        return "Waiting"

    def next(self, state=1):
        return STANDING_BY()

    def to_transition(self, drone, other_drone, balloon, borders):
        if drone.testing:
            drone.testing = 0
            return 1
        return 0

    def run(self, drone, other_drone, balloon, borders):
        drone.seek_middle(other_drone.obstacle)

class STANDING_BY(State):
    XY_VEL_BOUND = 30
    def __str__(self):
        return "Standing By"

    def next(self, state=1):
        return SEARCHING_PREDICTION() if state == 1 else PREPARE_AND_AVOID()

    def to_transition(self, drone, other_drone, balloon, borders):
        if not borders.in_borders(balloon):
            return 0
        pred = NumericBallPredictor(balloon)
        pred_time, pred_coords = pred.get_optimal_hitting_point(z_bound=drone.z / 100,
                                                            xy_vel_bound=self.XY_VEL_BOUND / 100)                                                    
        
        x_rel = pred_coords[0] - drone.x
        y_rel = pred_coords[1] - drone.y
        x_rel_other = pred_coords[0] - other_drone.x
        y_rel_other = pred_coords[1] - other_drone.y

        if (x_rel**2 + y_rel**2) == (x_rel_other**2 + y_rel_other**2):
            return 1 if drone.active else 2
        elif (x_rel**2 + y_rel**2) < (x_rel_other**2 + y_rel_other**2):
            first_on_second_off(drone, other_drone)
            return 1
        else:
            first_on_second_off(other_drone, drone)
            return 2

    # def to_transition(self, drone, other_drone, balloon, borders):
    #     if not borders.in_borders(balloon):
    #         return 0
    #     if drone.active:
    #         return 1
    #     return 2

    def run(self, drone, other_drone, balloon, borders):
        if borders.set_borders:
            drone.seek_middle(other_drone.obstacle)
        else:
            x_dest, y_dest = drone.x_0, drone.y_0
            drone.track_2d(x_dest, y_dest)


class SEARCHING_PREDICTION(State):
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
        return HITTING() if state == 1 else WAITING()

    def to_transition(self, drone, other_drone, balloon, borders):
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

    def run(self, drone, other_drone, balloon, borders):
        pred = NumericBallPredictor(balloon)
        x_dest, y_dest, z_dest = pred.get_prediction(reachability(distance=0, offset=0))
        z_dest = drone.z
        drone.track_3d(x_dest, y_dest, z_dest)


class HITTING(State):
    def __str__(self):
        return "Hitting"

    def next(self, state=1):
        return DESCENDING()

    def setup(self, drone, other_drone, balloon, borders):
        drone.start_hit()
    
    def cleanup(self, transition, drone, other_drone, balloon, borders):
        first_on_second_off(other_drone, drone)

    def to_transition(self, drone, other_drone, balloon, borders):
        Z_LIMIT = 25
        # XY_LIMIT = 40

        x_rel = balloon.x - drone.x
        y_rel = balloon.y - drone.y
        z_rel = balloon.z - drone.z

        # transition = not (abs(x_rel) < XY_LIMIT and abs(y_rel) < XY_LIMIT) or (z_rel < Z_LIMIT)
        transition = z_rel < Z_LIMIT
        return transition

    def run(self, drone, other_drone, balloon, borders):
        time_since_hitting = (datetime.now() - drone.start_hit_timer).total_seconds()
        pred = NumericBallPredictor(balloon)
        x_dest, y_dest, z_dest = pred.get_prediction(reachability(0, 0) - time_since_hitting)

        drone.track_hitting(x_dest, y_dest, z_dest)


class DESCENDING(State):
    def __str__(self):
        return "Descending"

    def next(self, state=1):
        return PREPARE_AND_AVOID()

    def to_transition(self, drone, other_drone, balloon, borders):
        Z_OFFSET = 15
        return drone.z <= other_drone.z + Z_OFFSET

    def run(self, drone, other_drone, balloon, borders):
        drone.track_descending(other_drone.obstacle)


class PREPARE_AND_AVOID(State):
    def __str__(self):
        return "Prepare and Avoid"

    def next(self, state=1):
        return SEARCHING_PREDICTION() if state == 1 else WAITING()

    def to_transition(self, drone, other_drone, balloon, borders):
        if drone.active and (balloon.vz > 0 or balloon.z <= other_drone.z): # drone is active only after other drone hits
            return 1 
        if isinstance(other_drone.state, WAITING):
            return 2
        return 0

    def run(self, drone, other_drone, balloon, borders):
        HEIGHT_PREPARATION_FACTOR = 0.5
        
        if not drone.active: 
            x_dest, y_dest = other_drone.obstacle.get_preparation_dest()
            z_dest = HEIGHT_PREPARATION_FACTOR * other_drone.z
        else:   # this occurs only when the other drone finished the hitting stage
            x_dest, y_dest, z_dest = drone.dest_coords[0], drone.dest_coords[1], drone.dest_coords[2]

        drone.track_3d(x_dest, y_dest, z_dest, other_drone.obstacle)
