class Action:
    def __init__(self, configurations, durations):
        """
        Represents a sequence of target configurations with durations for a single action.

        Parameters:
            configurations (list of np.ndarray): List of target configurations.
            durations (list of float): List of durations for each configuration.
        """
        assert len(configurations) == len(durations), "Configurations and durations must have the same length."
        self.configurations = configurations
        self.durations = durations
        self.current_index = 0
        self.started = False
        self.ended = False

    def current_target(self):
        """
        Get the current target configuration and its duration.

        Returns:
            tuple: (target_configuration, duration)
        """
        if not self.ended:
            return self.configurations[self.current_index], self.durations[self.current_index]
        return None, None

    def advance(self):
        """
        Move to the next target configuration in the sequence.
        """
        if self.current_index < len(self.configurations) - 1:
            self.current_index += 1
        else:
            self.ended = True
