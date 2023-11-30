import EasyGA

def fitness():

    return

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
best_chromosome = ga.population[0] # pass this into our controller probably like below
# class DefensiveCamperController(KesslerController):
        
    # def __init__(self, chromosome):
        #chromosome = [[1,2,3],...]