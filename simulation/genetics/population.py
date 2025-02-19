from simulation.genetics.individual import Individual
from simulation.genetics.genome import Genome

import random


class Population:
    """
    Represents the population of individuals.
    """

    def __init__(self, size, gene_bounds):
        """
        Parameters:
            size (int): Size of the population.
            gene_bounds (list[(float, float)]): Gene bounds.
        """
        self.individuals = [Individual(Genome(gene_bounds=gene_bounds)) for _ in range(size)]

    def select(self, tournament_size=3):
        """
        Selects an individual using tournament selection  i.e. Take n individuals from the population
        at random and select the one with higher fitness score among them.
        """
        tournament = random.sample(self.individuals, tournament_size)
        return max(tournament, key=lambda ind: ind.fitness)

    def get_best_individual(self):
        """
        Returns the individual with the highest fitness.
        """
        return max(self.individuals, key=lambda ind: ind.fitness)

    def __iter__(self):
        """
        Returns an iterator over the individuals in the population.
        """
        return iter(self.individuals)
