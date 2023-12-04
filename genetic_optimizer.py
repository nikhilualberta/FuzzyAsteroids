import EasyGA
from neo_controller_genetic import NeoController
from kesslergame import Scenario, KesslerGame, GraphicsType

def fitness(chromosome):
    # define the scenario
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
    game = KesslerGame(settings=game_settings)
    
    neo_controller = NeoController(chromosome)
    score, _ = game.run(scenario=my_test_scenario, controllers=[neo_controller])
    # Return the negative number of asteroids hit to maximize it
    return score.teams[0].asteroids_hit


def generate_chromosome():
    chromosome = []
    
    return chromosome

ga = EasyGA.GA()
ga.chromosome_impl = lambda: generate_chromosome()
ga.chromosome_length = 1 # change this once controller finalized
ga.population_size = 100
ga.target_fitness_type = 'max'
ga.generation_goal = 15
ga.fitness_function_impl = fitness

ga.evolve()
ga.print_best_chromosome()
best_chromosome = ga.population[0]