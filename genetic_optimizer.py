import math
import random
import EasyGA

from neo_controller_genetic import GeneticNeoController
from kesslergame import Scenario, KesslerGame, GraphicsType, TrainerEnvironment

def fitness(chromosome):
     
    try:
        my_test_scenario = Scenario(name='Test Scenario',
        num_asteroids=5,
        ship_states=[
        {'position': (400, 400), 'angle': 90, 'lives': 3, 'team': 1},
        ],
        map_size=(1000, 800),
        time_limit=60,
        ammo_limit_multiplier=0,
        stop_if_no_ammo=False)
        game_settings = {'perf_tracker': True,
        'graphics_type': GraphicsType.Tkinter,
        'realtime_multiplier': 1,
        'graphics_obj': None}
        #game = KesslerGame(settings=game_settings)
        game = TrainerEnvironment(settings=game_settings)
        
        neo_controller = GeneticNeoController(chromosome)
        score, _ = game.run(scenario=my_test_scenario, controllers=[neo_controller])
        # Return the negative number of asteroids hit to maximize it
    
        # print(score.stop_reason)
        print(score.teams[0].asteroids_hit)
        return score.teams[0].asteroids_hit
    except:
        return 5

  

def generate_chromosome():
    chromosome = []

    # bullet time
    a = 0
    b = random.uniform(a, 0.05)
    c = random.uniform(b, 0.1)
    chromosome.append([a, b, c])

    a = random.uniform(0, 0.05)
    b = random.uniform(a, 0.05)
    c = random.uniform(b, 0.1)
    chromosome.append([a, b, c])

    a = 0
    b = 0.1
    chromosome.append(a)
    chromosome.append(b)

    # theta delta
    angle_small_threshold = math.pi/50
    angle_large_threshold = math.pi

    chromosome.append(-1*angle_large_threshold)
    chromosome.append(-1*angle_small_threshold)

    a = random.uniform(-1 * angle_large_threshold, 0)
    b = random.uniform(a, 0)
    c = random.uniform(b, 0)
    chromosome.append([a, b, c])

    a = random.uniform(-1 * angle_small_threshold, angle_small_threshold)
    b = random.uniform(a, angle_small_threshold)
    c = random.uniform(b, angle_small_threshold)
    chromosome.append([a, b, c])

    a = random.uniform(0, angle_large_threshold)
    b = random.uniform(a, angle_large_threshold)
    c = random.uniform(b, angle_large_threshold)
    chromosome.append([a, b, c])

    chromosome.append(angle_small_threshold) 
    chromosome.append(angle_large_threshold) 
    
    # ship turn
    a = -180
    b = -180
    c = -45
    chromosome.append([a, b, c])

    a = random.uniform(-90, -46)
    b = random.uniform(a, 0)
    c = 0
    chromosome.append([a, b, c])

    a = random.uniform(-45, -1)
    b = random.uniform(a, 45)
    c = 45
    chromosome.append([a, b, c])

    a = random.uniform(0, 44)
    b = random.uniform(a, 90)
    c = 90
    chromosome.append([a, b, c])

    a = random.uniform(45, 89)
    b = random.uniform(a, 180)
    c = 180
    chromosome.append([a, b, c])

    # ship fire (not sure if these should be randomized or not)
    chromosome.append([-1, -1, 0.1])
    chromosome.append([-0.1, 1, 1])

    # ship thrust
    thrust_max_point = 300
    thrust_mid_point = 50

    a = -thrust_max_point
    b = -thrust_max_point
    c = -thrust_mid_point
    chromosome.append([a, b, c])

    a = random.uniform(-thrust_max_point, -thrust_mid_point-1)
    b = random.uniform(a, 0)
    c = 0
    chromosome.append([a, b, c])

    a = random.uniform(-thrust_mid_point, -1)
    b = random.uniform(a, thrust_mid_point)
    c = thrust_mid_point
    chromosome.append([a, b, c])

    a = random.uniform(0, thrust_mid_point)
    b = random.uniform(a, thrust_max_point)
    c = thrust_max_point
    chromosome.append([a, b, c])

    a = random.uniform(thrust_mid_point, thrust_max_point-1)
    b = random.uniform(a, thrust_max_point)
    c = thrust_max_point
    chromosome.append([a, b, c])

    # current ship thrust
    a = -thrust_max_point
    b = -thrust_max_point
    c = -thrust_mid_point
    chromosome.append([a, b, c])

    a = random.uniform(-thrust_max_point, -thrust_mid_point)
    b = random.uniform(a, 0)
    c = 0
    chromosome.append([a, b, c])

    a = random.uniform(-thrust_mid_point, -1)
    b = random.uniform(a, thrust_mid_point)
    c = thrust_mid_point
    chromosome.append([a, b, c])

    a = random.uniform(0, thrust_mid_point-1)
    b = random.uniform(a, thrust_max_point)
    c = thrust_max_point
    chromosome.append([a, b, c])

    a = random.uniform(thrust_mid_point, thrust_max_point-1)
    b = random.uniform(a, thrust_max_point)
    c = thrust_max_point
    chromosome.append([a, b, c])
    
    # ship speed
    ship_speed_max_point = 240
    ship_speed_mid_point = 30
    a = -ship_speed_max_point
    b = -ship_speed_max_point
    c = -ship_speed_mid_point
    chromosome.append([a, b, c])

    a = random.uniform(-ship_speed_max_point, -ship_speed_mid_point-1)
    b = random.uniform(a, 0)
    c = 0
    chromosome.append([a, b, c])

    a = random.uniform(-ship_speed_mid_point, -1)
    b = random.uniform(a, ship_speed_mid_point)
    c = ship_speed_mid_point
    chromosome.append([a, b, c])

    a = random.uniform(0, ship_speed_mid_point-1)
    b = random.uniform(a, ship_speed_max_point)
    c = ship_speed_max_point
    chromosome.append([a, b, c])

    a = random.uniform(ship_speed_mid_point, ship_speed_max_point-1)
    b = random.uniform(a, ship_speed_max_point)
    c = ship_speed_max_point
    chromosome.append([a, b, c])

    ##
    # not sure about turn_error and turn output, so they are not chromosomeized yet
    ##

    # distance
    distance_max_threshold = 500
    distance_large_threshold = 200
    distance_mid_threshold = 100
    distance_small_threshold = 50

    a = 0
    b = 0
    c = distance_small_threshold
    chromosome.append([a, b, c])

    a = random.uniform(0, distance_small_threshold-1)
    b = random.uniform(a, distance_mid_threshold)
    c = distance_mid_threshold
    chromosome.append([a, b, c])

    a = random.uniform(distance_small_threshold, distance_mid_threshold-1)
    b = random.uniform(a, distance_large_threshold)
    c = distance_large_threshold
    chromosome.append([a, b, c])

    a = random.uniform(distance_mid_threshold, distance_large_threshold-1)
    b = random.uniform(a, distance_max_threshold)
    c = distance_max_threshold
    chromosome.append([a, b, c])

    a = random.uniform(distance_large_threshold, distance_max_threshold-1)
    b = random.uniform(a, distance_max_threshold)
    c = distance_max_threshold
    chromosome.append([a, b, c])

    return chromosome

def generate_guided_chromosome():
    chromosome = []

    # bullet time
    chromosome.append([0, 0, 0.05])
    chromosome.append([0,0.05,0.1])
    a = 0
    b = 0.1
    chromosome.append(a)
    chromosome.append(b)

    # theta delta
    angle_small_threshold = math.pi/50
    angle_large_threshold = math.pi

    chromosome.append(-1*angle_large_threshold)
    chromosome.append(-1*angle_small_threshold) 

    chromosome.append([-1*angle_large_threshold, -1*angle_small_threshold, 0])

    chromosome.append([-1*angle_small_threshold, 0, angle_small_threshold])

    chromosome.append([0, angle_small_threshold, angle_large_threshold])

    chromosome.append(angle_small_threshold) 
    chromosome.append(angle_large_threshold)
    
    # ship turn
    chromosome.append([-180,-180,-45])

    chromosome.append([-90,-45,0])

    chromosome.append([-45,0,45])
    
    chromosome.append([0,45,90])

    chromosome.append([45,180,180])

    # ship fire 
    chromosome.append([-1, -1, 0.1])
    chromosome.append([-0.1, 1, 1])

    # ship thrust
    thrust_max_point = 480
    thrust_mid_point = 50

    chromosome.append([-thrust_max_point, -thrust_max_point, -thrust_mid_point])
    
    chromosome.append([-thrust_max_point, -thrust_mid_point, 0])

    chromosome.append([-thrust_mid_point, 0, thrust_mid_point])

    chromosome.append([0, thrust_mid_point, thrust_max_point])

    chromosome.append([thrust_mid_point, thrust_max_point, thrust_max_point])

    # current ship thrust
    chromosome.append([-thrust_max_point, -thrust_max_point, -thrust_mid_point])

    chromosome.append([-thrust_max_point, -thrust_mid_point, 0])

    chromosome.append([-thrust_mid_point, 0, thrust_mid_point])
        
    chromosome.append([0, thrust_mid_point, thrust_max_point])

    chromosome.append([thrust_mid_point, thrust_max_point, thrust_max_point])
    
    # ship speed
    ship_speed_max_point = 240
    ship_speed_mid_point = 30

    chromosome.append([-ship_speed_max_point, -ship_speed_max_point, -ship_speed_mid_point])

    chromosome.append([-ship_speed_max_point, -ship_speed_mid_point, -0])

    chromosome.append([-ship_speed_mid_point, 0, ship_speed_mid_point])

    chromosome.append([0, ship_speed_mid_point, ship_speed_max_point])

    chromosome.append([ship_speed_mid_point, ship_speed_max_point, ship_speed_max_point])

    ##
    # not sure about turn_error and turn output, so they are not chromosomeized yet
    ##

    # distance
    distance_max_threshold = 700
    distance_large_threshold = 200
    distance_mid_threshold = 100
    distance_small_threshold = 50

    chromosome.append([0, 0, distance_small_threshold])

    chromosome.append([0, distance_small_threshold, distance_mid_threshold])

    chromosome.append([distance_small_threshold, distance_mid_threshold, distance_large_threshold])

    chromosome.append([distance_mid_threshold, distance_large_threshold, distance_max_threshold])

    chromosome.append([distance_large_threshold, distance_max_threshold, distance_max_threshold])

    return chromosome

ga = EasyGA.GA()
ga.chromosome_impl = lambda: generate_chromosome()
ga.chromosome_length = 38 
ga.population_size = 10
ga.target_fitness_type = 'max'
ga.generation_goal = 2
ga.fitness_function_impl = fitness
ga.evolve()
ga.print_best_chromosome()
best_chromosome = ga.population[0]

