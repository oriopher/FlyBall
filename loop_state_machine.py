from datetime import datetime
from tracking import track_3d, track_hitting, velocity_control_function, seek_middle, track_2d
from prediction import NumericBallPredictor
import numpy as np
from common import reachability, FLOOR_HEIGHT, DRONE_DEFAULT_HEIGHT

MIN_SAFE_HEIGHT = FLOOR_HEIGHT + 30


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
    def next(self, state=1):
        return HOVERING()

    def to_transition(self, *args, **kwargs):
        return kwargs['loop_status'].tookoff

    def run(self, *args, **kwargs):
        return


class HOVERING(State):
    def next(self, state=1):
        print("Waiting")
        return WAITING()

    def to_transition(self, *args, **kwargs):
        return kwargs['loop_status'].start

    def run(self, **kwargs):
        return


class WAITING(State):
    def next(self, state=1):
        print("Stand By")
        return STANDING_BY()

    def to_transition(self, *args, **kwargs):
        loop_status = kwargs['loop_status']
        if loop_status.testing:
            loop_status.testing = 0
            return 1
        return 0

    def run(self, *args, **kwargs):
        borders = kwargs['borders']
        loop_status = kwargs['loop_status']

        if borders.set_borders:
            seek_middle(kwargs['image_3d'], kwargs['tello'], borders)
            x_dest = borders.x_middle
            y_dest = borders.y_middle
            z_dest = DRONE_DEFAULT_HEIGHT
        else:
            x_dest = loop_status.x_0
            y_dest = loop_status.y_0
            z_dest = DRONE_DEFAULT_HEIGHT
            track_2d(kwargs['image_3d'], kwargs['tello'], x_dest, y_dest)

        loop_status.set_dest_coords((x_dest, y_dest, z_dest))


class STANDING_BY(State):
    def next(self, state=1):
        print("Search Prediction")
        return SEARCHING_PREDICTION()

    def to_transition(self, *args, **kwargs):
        borders = kwargs['borders']
        image_3d = kwargs['image_3d']

        return borders.balloon_in_borders(image_3d)

    def run(self, *args, **kwargs):
        borders = kwargs['borders']
        loop_status = kwargs['loop_status']

        if borders.set_borders:
            seek_middle(kwargs['image_3d'], kwargs['tello'], borders)
            x_dest = borders.x_middle
            y_dest = borders.y_middle
            z_dest = DRONE_DEFAULT_HEIGHT
        else:
            x_dest = loop_status.x_0
            y_dest = loop_status.y_0
            z_dest = DRONE_DEFAULT_HEIGHT
            track_2d(kwargs['image_3d'], kwargs['tello'], x_dest, y_dest)

        loop_status.set_dest_coords((x_dest, y_dest, z_dest))


class SEARCHING_PREDICTION(State):
    Z_OFFSET = 50
    XY_VEL_BOUND = 30

    def next(self, state=1):
        if state == 1:
            print("Searching")
        else:
            print("Stand By")
        return SEARCHING() if state == 1 else STANDING_BY()

    def setup(self, *args, **kwargs):
        image_3d = kwargs['image_3d']
        loop_status = kwargs['loop_status']
        loop_status.drone_search_pred_time = datetime.now()
        loop_status.drone_search_pred_coords = (image_3d.phys_x_drone, image_3d.phys_y_drone, image_3d.phys_z_drone)

    def to_transition(self, *args, **kwargs):
        image_3d = kwargs['image_3d']
        borders = kwargs['borders']
        loop_status = kwargs['loop_status']

        # if np.sqrt(image_3d.velocity_x_balloon ** 2 + image_3d.velocity_y_balloon ** 2) <= self.XY_VEL_BOUND \
        #         and image_3d.velocity_z_balloon <= 0 and image_3d.phys_z_balloon >= image_3d.phys_z_drone:
        #     loop_status.drone_search_pred_time = 0
        #     loop_status.drone_search_pred_coords = (0,0,0)
        #     return 1
        if np.sqrt(image_3d.velocity_x_balloon ** 2 + image_3d.velocity_y_balloon ** 2) <= self.XY_VEL_BOUND \
                and image_3d.phys_z_balloon >= image_3d.phys_z_drone:
            return 1
        if image_3d.velocity_z_balloon <= 0 and image_3d.phys_z_balloon <= image_3d.phys_z_drone:
            return 2
        if not (borders.balloon_in_borders(image_3d) and borders.drone_in_borders(image_3d)):
            return 2
        return 0

    def run(self, *args, **kwargs):
        image_3d = kwargs['image_3d']

        pred = NumericBallPredictor(image_3d)
        pred_time, pred_coords = pred.get_optimal_hitting_point(z_bound=image_3d.phys_z_drone / 100,
                                                                xy_vel_bound=self.XY_VEL_BOUND / 100)
        x_dest, y_dest, z_dest = pred_coords
        loop_status = kwargs['loop_status']
        if (x_dest, y_dest, z_dest) == (0, 0, 0):
            x_dest, y_dest, z_dest = loop_status.dest_coords
        # x_dest = image_3d.get_phys_balloon(0)
        # y_dest = image_3d.get_phys_balloon(1)
        # z_dest = Z_HIT

        x_to_target = abs(x_dest - loop_status.drone_search_pred_coords[0])
        y_to_target = abs(y_dest - loop_status.drone_search_pred_coords[1])
        time_to_hit_from_start = max(reachability(x_to_target), reachability(y_to_target))
        time_until_hit = time_to_hit_from_start + (loop_status.drone_search_pred_time - datetime.now()).total_seconds()
        pred_time, pred_coords = pred.get_optimal_hitting_point(z_bound=image_3d.phys_z_drone / 100,
                                                                xy_vel_bound=self.XY_VEL_BOUND / 100,
                                                                start_time=time_until_hit)
        if not np.any(pred_coords):  # if pred_coords != (0,0,0)
            z_dest = pred_coords[2] - self.Z_OFFSET
            if z_dest < MIN_SAFE_HEIGHT:
                z_dest = MIN_SAFE_HEIGHT

        loop_status.set_dest_coords((x_dest, y_dest, z_dest))
        track_3d(image_3d, kwargs['tello'], x_dest, y_dest, z_dest)


class SEARCHING(State):
    Z_OFFSET = 50

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

        image_3d = kwargs['image_3d']
        borders = kwargs['borders']
        x_rel = int(image_3d.get_phys_balloon(0) - image_3d.get_phys_drone(0))
        y_rel = int(image_3d.get_phys_balloon(1) - image_3d.get_phys_drone(1))
        z_rel = int(image_3d.get_phys_balloon(2) - image_3d.get_phys_drone(2))

        if abs(x_rel) < XY_LIMIT and abs(y_rel) < XY_LIMIT and LOWER_LIMIT < z_rel < UPPER_LIMIT \
                and abs(image_3d.velocity_x_drone) < VEL_LIMIT and abs(image_3d.velocity_y_drone) < VEL_LIMIT \
                and image_3d.velocity_z_balloon <= 0:
            return 1
        if image_3d.velocity_z_balloon <= 0 and image_3d.phys_z_balloon <= image_3d.phys_z_drone:
            return 2
        if not (borders.balloon_in_borders(image_3d) and borders.drone_in_borders(image_3d)):
            return 2
        return 0

    def run(self, *args, **kwargs):
        image_3d = kwargs['image_3d']
        loop_status = kwargs['loop_status']
        x_dest = image_3d.get_phys_balloon(0)
        y_dest = image_3d.get_phys_balloon(1)
        z_dest = image_3d.get_phys_drone(2)
        loop_status.set_dest_coords((x_dest, y_dest, z_dest))
        track_3d(image_3d, kwargs['tello'], x_dest, y_dest, z_dest)


class HITTING(State):
    def next(self, state=1):
        return DESCENDING()

    def setup(self, *args, **kwargs):
        loop_status = kwargs['loop_status']
        loop_status.start_hit_timer = None
        loop_status.start_hit_time = datetime.now()

    def to_transition(self, *args, **kwargs):
        Z_LIMIT = 15
        XY_LIMIT = 40

        image_3d = kwargs['image_3d']
        x_rel = int(image_3d.get_phys_balloon(0) - image_3d.get_phys_drone(0))
        y_rel = int(image_3d.get_phys_balloon(1) - image_3d.get_phys_drone(1))
        z_rel = int(image_3d.get_phys_balloon(2) - image_3d.get_phys_drone(2))

        transition = not (abs(x_rel) < XY_LIMIT and abs(y_rel) < XY_LIMIT) or (z_rel < Z_LIMIT)
        return transition

    def run(self, *args, **kwargs):
        image_3d = kwargs['image_3d']
        tello = kwargs['tello']
        loop_status = kwargs['loop_status']

        time_since_hitting = (datetime.now() - loop_status.start_hit_timer).total_seconds()
        pred = NumericBallPredictor(image_3d)
        x_dest, y_dest, z_dest = pred.get_prediction(reachability(0, 0) - time_since_hitting)

        loop_status.set_dest_coords((x_dest, y_dest, z_dest))

        track_hitting(image_3d, tello, x_dest, y_dest, z_dest)


class DESCENDING(State):
    def next(self, state=1):
        return WAITING()

    def to_transition(self, *args, **kwargs):
        Z_OFFSET = 15
        image_3d = kwargs['image_3d']
        return image_3d.phys_z_drone < DRONE_DEFAULT_HEIGHT + Z_OFFSET

    def run(self, *args, **kwargs):
        tello = kwargs['tello']
        left_right, for_back = 0, 0
        up_down = -100
        while not tello.send_rc_control:
            continue
        tello.send_rc_control(left_right, for_back, up_down, 0)