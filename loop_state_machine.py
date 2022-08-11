class State:
    """
    A base class for states for the drones state machine in the main loop (a drone is per drone).
    """
    def next(self, state=1):
        """
        :param state: a number representing the next state received from the transition function.
        :return: the next state of the drone.
        """
        raise NotImplemented

    def to_transition(self, drone, other_drone, balloon, borders):
        """
        The states transition function.
        :param drone: the current drone's Drone object.
        :param other_drone: the other drone's Drone object.
        :param balloon: the balloons RecognizableObject.
        :param borders: the Borders of the game.
        :return: 0 or False if the drone should stay in the current state.
        True or a number if the drone should transition (the number helps the next function decide to which state).
        """
        raise NotImplemented

    def run(self, drone, other_drone, balloon, borders):
        """
        The main logic of the state.
        :param drone: the current drone's Drone object.
        :param other_drone: the other drone's Drone object.
        :param balloon: the balloons RecognizableObject.
        :param borders: the Borders of the game.
        """
        raise NotImplemented

    def setup(self, drone, other_drone, balloon, borders):
        """
        Initializations at the begging of the state (not always implemented).
        :param drone: the current drone's Drone object.
        :param other_drone: the other drone's Drone object.
        :param balloon: the balloons RecognizableObject.
        :param borders: the Borders of the game.
        """
        pass

    def cleanup(self, transition, drone, other_drone, balloon, borders):
        """
        Cleanups at the end of the state (not always implemented).
        :param transition: the result of the transition function indicating the next state.
        :param drone: the current drone's Drone object.
        :param other_drone: the other drone's Drone object.
        :param balloon: the balloons RecognizableObject.
        :param borders: the Borders of the game.
        """
        pass
