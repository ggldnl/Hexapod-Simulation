import random


class Genome:
    """
    Represents the parameters of a gait pattern.
    """

    def __init__(self, genes=None, gene_bounds=None):
        """
        Parameters:
            genes (list[int]): List of gene values (e.g., [step_length, phase_offset, speed, etc.]).
            gene_bounds (list[tuple[float, float]]): List of (min, max) tuples for each gene.
        """
        self.genes = genes if genes else []
        self.gene_bounds = gene_bounds if gene_bounds else []

        if len(self.genes) == 0:
            self.randomize()

    def __eq__(self, other):
        if not isinstance(other, Genome):
            raise ValueError(f'Could not compare instance of {type(self)} with {type(other)}.')
        if len(self.genes) != len(other.genes):
            return False
        return all([self.genes[i] == other.genes[i] for i in range(len(self.genes))])

    def randomize(self):
        """
        Randomly initializes genes within bounds.
        """
        self.genes = [random.uniform(low, high) for low, high in self.gene_bounds]

    def mutate(self, mutation_rate=0.1):
        """
        Mutates the genome with the given mutation rate.

        Parameters:
            mutation_rate (float): Mutation rate.
        """
        for i in range(len(self.genes)):
            if random.random() < mutation_rate:
                low, high = self.gene_bounds[i]
                self.genes[i] = random.uniform(low, high)

    def crossover(self, other):
        """
        Performs crossover with another genome.

        Parameters:
            other (Genome): Other genome to cross with self.
        """
        crossover_point = random.randint(1, len(self.genes) - 1)
        child1_genes = self.genes[:crossover_point] + other.genes[crossover_point:]
        child2_genes = other.genes[:crossover_point] + self.genes[crossover_point:]
        return Genome(child1_genes, self.gene_bounds), Genome(child2_genes, self.gene_bounds)
