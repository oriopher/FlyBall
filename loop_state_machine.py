from velocity_pot import track_3d, lin_velocity_with_two_params, seek_middle, track_2d

FLOOR_HEIGHT = -70
DRONE_DEFAULT_HEIGHT = FLOOR_HEIGHT + 80


class State:
    @property
    def next(self):
        raise NotImplemented

    def to_transition(self, *args, **kwargs):
        raise NotImplemented

    def run(self, *args, **kwargs):
        raise NotImplemented


class ON_GROUND(State):
    @property
    def next(self):
        return HOVERING()

    def to_transition(self, *args, **kwargs):
        return kwargs['loop_status'].tookoff

    def run(self, *args, **kwargs):
        return


class HOVERING(State):
    @property
    def next(self):
        return STANDING_BY()

    def to_transition(self, *args, **kwargs):
        return kwargs['loop_status'].start

    def run(self, **kwargs):
        return


class STANDING_BY(State):
    @property
    def next(self):
        return SEARCHING()

    def to_transition(self, *args, **kwargs):
        return kwargs['loop_status'].hit

    def run(self, *args, **kwargs):
        borders = kwargs['borders']
        if borders.set_borders:
            seek_middle(kwargs['image_3d'],  kwargs['tello'], borders)
        else:
            loop_status = kwargs['loop_status']
            track_2d(kwargs['image_3d'],  kwargs['tello'], loop_status.x_0, loop_status.y_0)


class SEARCHING(State):
    @property
    def next(self):
        return HITTING()

    def to_transition(self, *args, **kwargs):
        UPPER_LIMIT = 50
        LOWER_LIMIT = 30
        XY_LIMIT = 10
        VEL_LIMIT = 5

        image_3d = kwargs['image_3d']
        loop_status = kwargs['loop_status']
        x_rel = int(image_3d.phys_x_balloon - image_3d.phys_x_drone)
        y_rel = int(image_3d.phys_y_balloon - image_3d.phys_y_drone)
        z_rel = int(loop_status.hit_coords[2] - image_3d.phys_z_drone)

        return abs(x_rel) < XY_LIMIT and abs(y_rel) < XY_LIMIT and LOWER_LIMIT < z_rel < UPPER_LIMIT \
               and abs(image_3d.velocity_x_drone) < VEL_LIMIT and abs(image_3d.velocity_y_drone) < VEL_LIMIT

    def run(self, *args, **kwargs):
        Z_OFFSET = 40
        image_3d = kwargs['image_3d']
        loop_status = kwargs['loop_status']
        track_3d(image_3d, kwargs['tello'], image_3d.phys_x_balloon,
                 image_3d.phys_y_balloon, loop_status.hit_coords[2] - Z_OFFSET)


class HITTING(State):
    @property
    def next(self):
        return DESCENDING()

    def to_transition(self, *args, **kwargs):
        Z_LIMIT = 15
        XY_LIMIT = 40

        image_3d = kwargs['image_3d']
        loop_status = kwargs['loop_status']
        x_rel = int(image_3d.phys_x_balloon - image_3d.phys_x_drone)
        y_rel = int(image_3d.phys_y_balloon - image_3d.phys_y_drone)
        z_rel = int(loop_status.hit_coords[2] - image_3d.phys_z_drone)

        transition = not (abs(x_rel) < XY_LIMIT and abs(y_rel) < XY_LIMIT) or (z_rel < Z_LIMIT)
        if transition:
            loop_status.hit_mode_off()
        return transition

    def run(self, *args, **kwargs):
        image_3d = kwargs['image_3d']
        tello = kwargs['tello']
        x_rel = int(image_3d.phys_x_balloon - image_3d.phys_x_drone)
        y_rel = int(image_3d.phys_y_balloon - image_3d.phys_y_drone)

        left_right = lin_velocity_with_two_params(x_rel, image_3d.velocity_x_balloon, 'x')
        for_back = lin_velocity_with_two_params(y_rel, image_3d.velocity_y_balloon, 'y')
        up_down = 100
        while not tello.send_rc_control:
            continue
        tello.send_rc_control(left_right, for_back, up_down, 0)


class DESCENDING(State):
    @property
    def next(self):
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

