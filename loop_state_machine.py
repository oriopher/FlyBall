from velocity_pot import track_3d, lin_velocity_with_two_params, seek_middle, track_2d
from prediction import NumericBallPredictor
import numpy as np

FLOOR_HEIGHT = -100
DRONE_DEFAULT_HEIGHT = FLOOR_HEIGHT + 40


class State:
    def next(self, state=1):
        raise NotImplemented

    def to_transition(self, *args, **kwargs):
        raise NotImplemented

    def run(self, *args, **kwargs):
        raise NotImplemented


class ON_GROUND(State):
    def next(self, state=1):
        return HOVERING()

    def to_transition(self, *args, **kwargs):
        return kwargs['loop_status'].tookoff

    def run(self, *args, **kwargs):
        return


class HOVERING(State):
    def next(self, state=1):
        return STANDING_BY()

    def to_transition(self, *args, **kwargs):
        return kwargs['loop_status'].start

    def run(self, **kwargs):
        return


class STANDING_BY(State):
    def next(self, state=1):
        return SEARCHING_PREDICTION()

    def to_transition(self, *args, **kwargs):
        loop_status = kwargs['loop_status']
        return loop_status.test_state == 2
        return kwargs['loop_status'].hit

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
            # image_3d = kwargs['image_3d']
            # x_dest = 20
            # y_dest = 160
            # z_dest = -10
            # track_3d(image_3d,  kwargs['tello'], x_dest, y_dest, z_dest)

        loop_status.set_dest_coords((x_dest, y_dest, z_dest))


class SEARCHING_PREDICTION(State):
    Z_OFFSET = 50
    XY_VEL_BOUND = 5

    def next(self, state=1):
        return SEARCHING() if state == 1 else STANDING_BY()

    def to_transition(self, *args, **kwargs):
        z_bound = DRONE_DEFAULT_HEIGHT + self.Z_OFFSET
        image_3d = kwargs['image_3d']

        if np.sqrt(image_3d.velocity_x_balloon ** 2 + image_3d.velocity_y_balloon ** 2) <= self.XY_VEL_BOUND \
                and image_3d.velocity_z_balloon <= 0 and image_3d.phys_z_balloon >= z_bound:
            return 1
        if image_3d.velocity_z_balloon <= 0 and image_3d.phys_z_balloon <= z_bound:
            return 2
        return 0

    def run(self, *args, **kwargs):
        Z_HIT = DRONE_DEFAULT_HEIGHT + self.Z_OFFSET
        image_3d = kwargs['image_3d']
        pred = NumericBallPredictor(image_3d)
        pred_time, pred_coords = pred.get_optimal_hitting_point(z_bound=Z_HIT, xy_vel_bound=self.XY_VEL_BOUND)
        x_dest, y_dest, z_dest = pred_coords
        loop_status = kwargs['loop_status']
        if (x_dest, y_dest, z_dest) == (0, 0, 0):
            loop_status.stop_hit()
            return
        # x_dest = image_3d.get_phys_balloon(0)
        # y_dest = image_3d.get_phys_balloon(1)
        # z_dest = Z_HIT
        loop_status.set_dest_coords((x_dest, y_dest, z_dest - Z_OFFSET))
        track_3d(image_3d, kwargs['tello'], x_dest, y_dest, z_dest - Z_OFFSET)


class SEARCHING(State):
    def next(self, state=1):
        return HITTING() if state == 1 else STANDING_BY()

    def to_transition(self, *args, **kwargs):
        UPPER_LIMIT = 120
        LOWER_LIMIT = 20
        XY_LIMIT = 50
        VEL_LIMIT = 40

        image_3d = kwargs['image_3d']
        x_rel = int(image_3d.get_phys_balloon(0) - image_3d.get_phys_drone(0))
        y_rel = int(image_3d.get_phys_balloon(1) - image_3d.get_phys_drone(1))
        z_rel = int(image_3d.get_phys_balloon(2) - image_3d.get_phys_drone(2))

        if abs(x_rel) < XY_LIMIT and abs(y_rel) < XY_LIMIT and LOWER_LIMIT < z_rel < UPPER_LIMIT \
               and abs(image_3d.velocity_x_drone) < VEL_LIMIT and abs(image_3d.velocity_y_drone) < VEL_LIMIT \
               and image_3d.velocity_z_balloon < 0:
            return 1
        if image_3d.velocity_z_balloon <= 0 and image_3d.phys_z_balloon <= Z_BOUND:
            return 2
        return 0

    def run(self, *args, **kwargs):
        Z_OFFSET = 50
        image_3d = kwargs['image_3d']
        loop_status = kwargs['loop_status']
        track_3d(image_3d, kwargs['tello'], image_3d.phys_x_balloon,
                 image_3d.phys_y_balloon, loop_status.hit_coords[2] - Z_OFFSET)


class HITTING(State):
    def next(self, state=1):
        return DESCENDING()

    def to_transition(self, *args, **kwargs):
        Z_LIMIT = 15
        XY_LIMIT = 40

        image_3d = kwargs['image_3d']
        loop_status = kwargs['loop_status']
        x_rel = int(image_3d.get_phys_balloon(0) - image_3d.get_phys_drone(0))
        y_rel = int(image_3d.get_phys_balloon(1) - image_3d.get_phys_drone(1))
        z_rel = int(image_3d.get_phys_balloon(2) - image_3d.get_phys_drone(2))

        transition = not (abs(x_rel) < XY_LIMIT and abs(y_rel) < XY_LIMIT) or (z_rel < Z_LIMIT)
        if transition:
            loop_status.hit_mode_off()
        return transition

    def run(self, *args, **kwargs):
        image_3d = kwargs['image_3d']
        tello = kwargs['tello']
        x_rel = int(image_3d.get_phys_balloon(0) - image_3d.get_phys_drone(0))
        y_rel = int(image_3d.get_phys_balloon(1) - image_3d.get_phys_drone(1))

        loop_status = kwargs['loop_status']
        x_dest = image_3d.get_phys_balloon(0)
        y_dest = image_3d.get_phys_balloon(1)
        z_dest = image_3d.get_phys_balloon(2)
        loop_status.set_dest_coords((x_dest, y_dest, z_dest))

        left_right = lin_velocity_with_two_params(x_rel, image_3d.velocity_x_balloon, 'x')
        for_back = lin_velocity_with_two_params(y_rel, image_3d.velocity_y_balloon, 'y')
        up_down = 100
        while not tello.send_rc_control:
            continue
        tello.send_rc_control(left_right, for_back, up_down, 0)


class DESCENDING(State):
    def next(self, state=1):
        return STANDING_BY()

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
