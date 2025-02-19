class Individual:
    """
    Represents an individual in the population.
    """

    def __init__(self, genome):
        self.genome = genome
        self.fitness = None

    def __eq__(self, other):
        if not isinstance(other, Individual):
            raise ValueError(f'Could not compare instance of {type(self)} with {type(other)}.')
        return self.genome == other.genome