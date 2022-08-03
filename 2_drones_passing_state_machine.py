from datetime import datetime
from prediction import NumericBallPredictor
import numpy as np
from common import reachability,DRONE_DEFAULT_HEIGHT, MIN_SAFE_HEIGHT


class State:
    def next(self, state=1):
        raise NotImplemented

    def to_transition(self, *args, **kwargs):
        raise NotImplemented

    def run(self, *args, **kwargs):
        raise NotImplemented

    def setup(self, *args, **kwargs):
        pass

    def cleanup(self, state, *args, **kwargs):
        pass


class ON_GROUND(State):
    def __str__(self):
        return "On Ground"

    def next(self, state=1):
        return HOVERING()

    def to_transition(self, *args, **kwargs):
        return kwargs['drone'].tookoff

    def run(self, *args, **kwargs):
        return


class HOVERING(State):
    def __str__(self):
        return "Hovering"

    def next(self, state=1):
        print("Waiting")
        return WAITING()

    def to_transition(self, *args, **kwargs):
        return kwargs['drone'].start

    def run(self, **kwargs):
        return


class WAITING(State):
    def __str__(self):
        return "Waiting"

    def next(self, state=1):
        print("Stand By")
        return STANDING_BY()

    def to_transition(self, *args, **kwargs):
        drone = kwargs['drone']
        if drone.testing:
            drone.testing = 0
            return 1
        return 0

    def run(self, *args, **kwargs):
        borders = kwargs['borders']
        drone = kwargs['drone']

        if borders.set_borders:
            x_dest, y_dest = drone.middle
            drone.seek_middle()
        else:
            x_dest, y_dest = drone.x_0, drone.y_0
            drone.track_2d(x_dest, y_dest)
        z_dest = DRONE_DEFAULT_HEIGHT


class STANDING_BY(State):
    def __str__(self):
        return "Standing By"

    def next(self, state=1):
        if state == 1:
            print("Search Prediction")
        else:
            print("Prepare and Avoid")
        return SEARCHING_PREDICTION() if state == 1 else PREPARE_AND_AVOID()

    def to_transition(self, *args, **kwargs):
        borders = kwargs['borders']
        balloon = kwargs['balloon']
        drone = kwargs['drone']

        if borders.in_borders(balloon):
            return 1 if drone.active else 2
        else:
            return 0

    def run(self, *args, **kwargs):
        borders = kwargs['borders']
        drone = kwargs['drone']

        if borders.set_borders:
            x_dest, y_dest = drone.middle
            drone.seek_middle()
        else:
            x_dest, y_dest = drone.x_0, drone.y_0
            drone.track_2d(x_dest, y_dest)


class SEARCHING_PREDICTION(State):
    Z_OFFSET = 50
    XY_VEL_BOUND = 30

    def __str__(self):
        return "Searching Prediction"

    def next(self, state=1):
        if state == 1:
            print("Searching")
        else:
            print("Stand By")
        return SEARCHING() if state == 1 else STANDING_BY()

    def setup(self, *args, **kwargs):
        drone = kwargs['drone']
        drone.search_pred_start()

    def to_transition(self, *args, **kwargs):
        balloon = kwargs['balloon']
        drone = kwargs['drone']
        borders = kwargs['borders']

        if np.sqrt(balloon.vx ** 2 + balloon.vy ** 2) <= self.XY_VEL_BOUND \
                and balloon.z >= drone.z:
            return 1
        if balloon.vz <= 0 and balloon.z <= drone.z:
            return 2
        if not (borders.in_borders(balloon) and borders.in_borders(drone)):
            return 2
        return 0

    def run(self, *args, **kwargs):
        drone = kwargs['drone']
        balloon = kwargs['balloon']

        pred = NumericBallPredictor(balloon)
        pred_time, pred_coords = pred.get_optimal_hitting_point(z_bound=drone.z / 100,
                                                                xy_vel_bound=self.XY_VEL_BOUND / 100)

        if np.any(pred_coords):
            x_dest, y_dest, z_dest = drone.new_pred(pred_coords)
        else:
            x_dest, y_dest, z_dest = drone.dest_coords
        # x_dest = recognizable_object.get_phys_balloon(0)
        # y_dest = recognizable_object.get_phys_balloon(1)
        # z_dest = Z_HIT

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
        if state == 1:
            print("Hitting")
        else:
            print("Stand By")
        return HITTING() if state == 1 else STANDING_BY()

    def to_transition(self, *args, **kwargs):
        UPPER_LIMIT = 110
        LOWER_LIMIT = 20
        XY_LIMIT = 30
        VEL_LIMIT = 30

        drone = kwargs['drone']
        balloon = kwargs['balloon']
        borders = kwargs['borders']

        x_rel = balloon.x - drone.x
        y_rel = balloon.y - drone.y
        z_rel = balloon.z - drone.z

        if abs(x_rel) < XY_LIMIT and abs(y_rel) < XY_LIMIT and LOWER_LIMIT < z_rel < UPPER_LIMIT \
                and abs(drone.vx) < VEL_LIMIT and abs(drone.vy) < VEL_LIMIT \
                and balloon.vz <= 0:
            return 1
        if balloon.vz <= 0 and balloon.z <= drone.z:
            return 2
        if not (borders.in_borders(balloon) and borders.in_borders(drone)):
            return 2
        return 0

    def run(self, *args, **kwargs):
        drone = kwargs['drone']
        balloon = kwargs['balloon']
        x_dest, y_dest, z_dest = balloon.x, balloon.y, balloon.z
        drone.track_3d(x_dest, y_dest, z_dest)


class HITTING(State):
    def __str__(self):
        return "Hitting"

    def next(self, state=1):
        return DESCENDING()

    def setup(self, *args, **kwargs):
        drone = kwargs['drone']
        drone.start_hit()

    def to_transition(self, *args, **kwargs):
        Z_LIMIT = 15
        XY_LIMIT = 40

        drone = kwargs['drone']
        balloon = kwargs['balloon']
        x_rel = balloon.x - drone.x
        y_rel = balloon.y - drone.y
        z_rel = balloon.z - drone.z

        transition = not (abs(x_rel) < XY_LIMIT and abs(y_rel) < XY_LIMIT) or (z_rel < Z_LIMIT)
        return transition

    def run(self, *args, **kwargs):
        drone = kwargs['drone']
        balloon = kwargs['balloon']

        time_since_hitting = (datetime.now() - drone.start_hit_timer).total_seconds()
        pred = NumericBallPredictor(balloon)
        x_dest, y_dest, z_dest = pred.get_prediction(reachability(0, 0) - time_since_hitting)

        drone.track_hitting(x_dest, y_dest, z_dest)


class DESCENDING(State):
    def __str__(self):
        return "Descending"

    def setup(self, *args, **kwargs):
        drone, other_drone = kwargs['drone'], kwargs['other_drone']
        drone.active = False
        other_drone.active = True

    def next(self, state=1):
        return PREPARE_AND_AVOID()

    def to_transition(self, *args, **kwargs):
        Z_OFFSET = 15
        drone = kwargs['drone']
        return drone.z < DRONE_DEFAULT_HEIGHT + Z_OFFSET

    def run(self, *args, **kwargs):
        drone = kwargs['drone']
        left_right, for_back = 0, 0
        up_down = -100
        drone.wait_rc_control()
        drone.send_rc_control(left_right, for_back, up_down, 0)

class PREPARE_AND_AVOID(State):
    def __str__(self):
        return "Hovering"

    def next(self, state=1):
        print("Waiting")
        return WAITING()

    def to_transition(self, *args, **kwargs):
        return kwargs['drone'].start

    def run(self, **kwargs):
        return

