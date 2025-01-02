class Action:

    def __init__(self, states, durations):
        """
        Represents a sequence of target states with durations for each one of them.

        Parameters:
            states (list[Configuration]): List of target configurations.
            durations (list[float]): List of durations for each configuration.
        """
        assert len(states) == len(durations)

        self.states = states
        self.durations = durations
        self.current_index = 0
        self.started = False
        self.ended = False

    def current_target(self):
        """
        Get the current target state and its duration.

        Returns:
            tuple: (target_state, duration)
        """
        if not self.ended:
            return self.states[self.current_index], self.durations[self.current_index]
        return None, None

    def advance(self):
        """
        Move to the next target state in the sequence.
        """
        if self.current_index < len(self.states) - 1:
            self.current_index += 1
        else:
            self.ended = True
