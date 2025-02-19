import sys

sys.path.append('../Hexapod-Controller/controller')
print(f'Controller successfully imported.')

import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet_data

from pathlib import Path
import numpy as np
import argparse
import time

from simulation.sinusoidal_signals.generator import OpenLoopGaitGenerator
from simulation.genetics.genome import Genome
from simulation.genetics.individual import Individual
from simulation.genetics.population import Population


def map_signals(signals, min_angles=None, max_angles=None):
    min_angles = min_angles if min_angles is not None else np.full((6, 3), -np.pi/2)
    max_angles = max_angles if max_angles is not None else np.full((6, 3), np.pi / 2)
    return ((signals + 1) / 2) * (max_angles - min_angles) + min_angles

def disable_collisions(simulation_client, robot_A, robot_B):
    num_joints_A = simulation_client.getNumJoints(robot_A)
    num_joints_B = simulation_client.getNumJoints(robot_B)
    for link_index_A in range(-1, num_joints_A):
        for link_index_B in range(-1, num_joints_B):
            simulation_client.setCollisionFilterPair(robot_A, robot_B, link_index_A, link_index_B, enableCollision=False)

def genome_to_gait(genome):
    shape = (6, 3, 4)  # 4 parameters for each of the 3 joints of each of the 6 legs
    assert len(genome.genes) == shape[0] * shape[1] * shape[2]

    # Reshape the data
    gait_params = [
        [
            genome.genes[i * shape[1] * shape[2] + j * shape[2]: i * shape[1] * shape[2] + (j + 1) * shape[2]]
            for j in range(shape[1])
        ]
        for i in range(shape[0])
    ]
    return gait_params

def evaluate_fitness(data, target_position):

    if not isinstance(target_position, np.ndarray):
        target_position = np.array(target_position)

    positions = np.array([pos for pos, _ in data])  # Extract position vectors
    orientations = np.array([orient for _, orient in data])  # Extract orientation vectors

    # Compute total distance traveled on x-y plane
    final_position = positions[-1]
    initial_position = positions[0]
    positional_accuracy = np.linalg.norm(final_position[:2] - initial_position[:2])  # Only x and y for distance

    # Calculate height stability (variance from target_height)
    height_stability = np.std(positions[:, 2])

    # Penalize excess height above the target
    height_excess = positions[:, 2] - target_position[2]
    jump_penalty = np.sum(height_excess[height_excess > 0]) / len(positions)  # Average excess height

    # Calculate orientation stability
    orientation_stability = np.linalg.norm(np.std(orientations, axis=0))  # Standard deviation of orientation vectors

    """
    # Normalization
    max_height_stability = target_position[2]
    max_positional_accuracy = np.linalg.norm(target_position)
    max_orientation_stability = np.pi

    normalized_height = height_stability / max_height_stability
    normalized_position = positional_accuracy / max_positional_accuracy
    normalized_orientation = orientation_stability / max_orientation_stability
    """

    # Combined to create the fitness
    fitness = (
            positional_accuracy     # Encourage movement
            - height_stability      # Penalize height instability, we should have constant height during gait
            - orientation_stability # Penalize shaky movements
            -  jump_penalty         # Penalize excessive height occurring during jumps
    )

    return np.nanmax([fitness, 0])


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Learning open-loop gait patterns with genetic algorithms.")
    parser.add_argument("-u", "--URDF", type=str, default='Hexapod-Hardware/hexapod.urdf', help="Path to the robot's URDF")
    parser.add_argument('-n', '--name', type=str, default='hexapod', help="Name of the robot in the config")
    parser.add_argument('-d', '--dt', type=float, default=0.02, help="Time delta for update (default=0.02=50Hz)")
    parser.add_argument('-v', '--video_path', type=str, default=None, help="If provided, the script will save an mp4 of the simulation on the path")
    parser.add_argument('-p', '--population_size', type=int, default=10, help="Population size")
    parser.add_argument('-t', '--tournament_size', type=int, default=3, help="")  # TODO fill help
    parser.add_argument('-m', '--mutation_rate', type=float, default=0.1, help="")  # TODO fill help
    parser.add_argument('-g', '--generations', type=int, default=50, help="Number of generations to run the algorithm for")
    parser.add_argument('-r', '--duration', type=float, default=20, help="Duration of the simulation for each generation in seconds")

    args = parser.parse_args()

    # --------------------------------- Genetics --------------------------------- #

    population_size = args.population_size
    tournament_size = args.tournament_size
    mutation_rate = args.mutation_rate
    generations = args.generations
    duration = args.duration
    dt = args.dt

    target_position = [10, 0, 0.6]

    # --------------------------------- PyBullet --------------------------------- #

    # Connect to physics server
    # physics = p.connect(p.GUI)

    simulation_client = bc.BulletClient(connection_mode=p.GUI)
    simulation_client.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    simulation_client.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    simulation_client.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    simulation_client.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)  # Disable shadows

    # Set initial camera view
    default_camera_distance = 0.5
    default_camera_roll = 0
    default_camera_yaw = 0
    default_camera_pitch = -45
    default_camera_target = [0, 0, 0]

    simulation_client.resetDebugVisualizerCamera(
        cameraDistance=default_camera_distance,
        cameraYaw=default_camera_yaw,
        cameraPitch=default_camera_pitch,
        cameraTargetPosition=default_camera_target
    )

    # Load additional data and set gravity
    simulation_client.setAdditionalSearchPath(pybullet_data.getDataPath())
    simulation_client.setGravity(0, 0, -9.81)

    # Load plane and robot URDF files
    planeId = simulation_client.loadURDF("plane.urdf")

    repo_dir = Path(__file__).parent.parent
    urdf_file = Path(repo_dir, args.URDF)
    initial_position = [0, 0, 0]
    initial_orientation = p.getQuaternionFromEuler([0, 0, 0])

    # Add one robot for each individual
    robots = {}
    joint_mapping = None
    for _ in range(population_size):
        robotID = simulation_client.loadURDF(str(urdf_file), initial_position, initial_orientation, flags=p.URDF_USE_INERTIA_FROM_FILE)
        robots[robotID] = []

        if joint_mapping is None:

            # List the revolute joints
            revolute_joints = []
            num_joints = simulation_client.getNumJoints(robotID)
            for joint_index in range(num_joints):
                joint_info = simulation_client.getJointInfo(robotID, joint_index)
                joint_type = joint_info[2]  # Joint type is the third element in joint_info tuple
                if joint_type == simulation_client.JOINT_REVOLUTE:
                    revolute_joints.append(joint_index)
                    print(f"Joint {joint_index} is revolute: {joint_info[1].decode('utf-8')}")

            # We know the joints are well formatted (always coxa, femur and tibia for each leg, for leg from 1 to 6)
            joint_mapping = {
                i: {
                    'coxa_joint': revolute_joints[0 + i * 3],
                    'femur_joint': revolute_joints[1 + i * 3],
                    'tibia_joint': revolute_joints[2 + i * 3]
                } for i in range(6)
            }

    # Disable collisions once for good
    adj = [[int(i > j) for i in range(population_size)] for j in range(population_size)]
    robotIDs = list(robots.keys())
    couples = [(robotIDs[i], robotIDs[j]) for i in range(population_size) for j in range(population_size) if adj[i][j]]
    for i, j in couples:
        disable_collisions(simulation_client, i, j)

    # ------------------------------ Simulation loop ----------------------------- #

    if args.video_path:

        video_path = Path(args.video_path)
        folder_path = video_path.parent
        if not folder_path.exists():
            folder_path.mkdir(exist_ok=True, parents=True)

        simulation_client.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, args.video_path)

    try:

        # Create initial population
        # gene bounds are limits for (amplitude, phase, duty cycle, offset) for each joint of each leg
        gene_bounds = [(0, 1), (0, 1), (0, 1), (-1, 1)] * (3 * 6)
        genome_template = Genome(gene_bounds=gene_bounds)
        population = Population(size=population_size, gene_bounds=gene_bounds)

        for generation in range(generations):

            # Prepare gait signals given population genomes
            generators = {
                robotID: OpenLoopGaitGenerator(params=genome_to_gait(individual.genome))
                for robotID, individual in zip(robotIDs, population)
            }

            # Make the simulation run for the given duration and collect some gait data
            t = 0.0
            while t <= duration:

                # Step the simulations
                simulation_client.stepSimulation()

                for robotID in robotIDs:

                    generator = generators[robotID]

                    # Get the joint angles
                    signals = generator.step(t)
                    joint_angles = map_signals(signals)

                    for i in range(6):
                        joints = joint_mapping[i]
                        angles = joint_angles[i]

                        # Move the joints to the target position
                        p.setJointMotorControl2(robotID, joints['coxa_joint'], p.POSITION_CONTROL, targetPosition=angles[0])
                        p.setJointMotorControl2(robotID, joints['femur_joint'], p.POSITION_CONTROL, targetPosition=angles[1])
                        p.setJointMotorControl2(robotID, joints['tibia_joint'], p.POSITION_CONTROL, targetPosition=angles[2])

                # For each time step, store position and orientation of each robot
                for robotID in robotIDs:
                    position, orientation = p.getBasePositionAndOrientation(robotID)
                    orientation_euler = p.getEulerFromQuaternion(orientation)
                    robots[robotID].append((position, orientation_euler))

                time.sleep(dt)
                t += dt

            # Evaluate fitness function
            for individual, robotID in zip(population, robotIDs):
                robot_data = robots[robotID]
                individual.fitness = evaluate_fitness(robot_data, target_position)

            # Print current average fitness score
            fitness_scores = []
            for individual in population:
                fitness_scores.append(individual.fitness)
            average_fitness = sum(fitness_scores) / len(fitness_scores)
            print(f'Average fitness score for generation {generation}: {average_fitness}')

            # If this is the last iteration, break before resetting the fitness
            if generation == generations - 1:
                break

            # Evolution loop
            new_population = []
            while len(new_population) < population_size:

                # Selection
                parent1 = population.select(tournament_size)
                parent2 = population.select(tournament_size)

                # Crossover
                child1_genome, child2_genome = parent1.genome.crossover(parent2.genome)

                # Mutation
                child1_genome.mutate(mutation_rate)
                child2_genome.mutate(mutation_rate)

                # Create new individuals
                child1 = Individual(child1_genome)
                child2 = Individual(child2_genome)

                # Add to new population
                new_population.extend([child1, child2])

            # Replace old population with new population
            population.individuals = new_population[:population_size]

            # Reset the robot's base position and orientation
            for robotID in robotIDs:

                # Reset position and orientation
                p.resetBasePositionAndOrientation(robotID, initial_position, initial_orientation)

                # Reset base velocities
                p.resetBaseVelocity(robotID, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])

                # Reset joint position and velocity
                num_joints = p.getNumJoints(robotID)
                for joint_index in range(num_joints):
                    p.resetJointState(robotID, joint_index, targetValue=0, targetVelocity=0)

            # Reset the robot's position and orientation history
            for robotID in robotIDs:
                robots[robotID] = []

        # After all the generations, select the best individual
        best_individual = population.get_best_individual()
        print('Best individual:')
        print(best_individual.genome.genes)

        # TODO write the result to a file

    finally:

        print('Disconnected.')

        # Stop the recording
        if args.video_path:
            p.stopStateLogging(p.STATE_LOGGING_VIDEO_MP4)

        # Disconnect from the simulations when done
        # p.disconnect()
