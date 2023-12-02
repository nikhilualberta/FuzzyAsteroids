# Copyright © 2022 Thales. All Rights Reserved.
# NOTICE: This file is subject to the license agreement defined in file 'LICENSE', which is part of
# this source code package.

import time
from kesslergame import Scenario, KesslerGame, GraphicsType
from test_controller import TestController
from scott_dick_controller import ScottDickController
from defensive_camper_controller import DefensiveCamperController
from simulation_controller import SimulationController
from neo_controller import NeoController
from threat_controller import ThreatController
from graphics_both import GraphicsBoth
import random

random.seed(6)

def generate_asteroids(num_asteroids, position_range_x, position_range_y, speed_range, angle_range, size_range):
    asteroids = []
    for _ in range(num_asteroids):
        position = (random.randint(*position_range_x), random.randint(*position_range_y))
        speed = random.randint(*speed_range)
        angle = random.randint(*angle_range)
        size = random.randint(*size_range)
        asteroids.append({'position': position, 'speed': speed, 'angle': angle, 'size': size})

    return asteroids

width, height = (1600, 1000)

my_test_scenario = Scenario(name='Test Scenario',
                            #num_asteroids=5,
                            ship_states=[
                            {'position': (width//2, height//2), 'angle': 90, 'lives': 30, 'team': 1}],
                            #{'position': (600, 400), 'angle': 90, 'lives': 3, 'team': 2},
                            #],
                            #asteroid_states = [{'position': (1180, 720), 'speed': 800, 'angle': 180, 'size': 2}],
                            #asteroid_states = [{'position': (2560-800, 1440-100), 'speed': 600, 'angle': -172, 'size': 4}, {'position': (800, 100), 'speed': 600, 'angle': 8, 'size': 3}],
                            #asteroid_states = [{'position': (0, 720), 'speed': 60, 'angle': 0, 'size': 4}, {'position': (2500, 720), 'speed': -50, 'angle': 0, 'size': 4}],
                            #asteroid_states = [{'position': (2560, 1440), 'speed': 2400, 'angle': -174, 'size': 1}],
                            #asteroid_states = [{'position': (3223, 697), 'speed': 301, 'angle': -65, 'size': 3}, {'position': (316, 1335), 'speed': 287, 'angle': 161, 'size': 3}, {'position': (3268, 270), 'speed': 176, 'angle': -129, 'size': 2}, {'position': (922, 1171), 'speed': 94, 'angle': -178, 'size': 2}, {'position': (2296, 608), 'speed': 593, 'angle': -73, 'size': 3}, {'position': (3126, 1459), 'speed': 288, 'angle': -81, 'size': 2}, {'position': (2982, 1227), 'speed': 305, 'angle': 116, 'size': 3}, {'position': (35, 948), 'speed': 467, 'angle': -160, 'size': 3}, {'position': (1576, 1114), 'speed': 741, 'angle': 174, 'size': 3}, {'position': (1133, 621), 'speed': 497, 'angle': -4, 'size': 3}, {'position': (1506, 650), 'speed': 188, 'angle': 85, 'size': 2}, {'position': (1131, 290), 'speed': 113, 'angle': 6, 'size': 3}, {'position': (2624, 1086), 'speed': 359, 'angle': -150, 'size': 1}, {'position': (3304, 1217), 'speed': 667, 'angle': 160, 'size': 2}, {'position': (1639, 1004), 'speed': 390, 'angle': -127, 'size': 2}, {'position': (655, 335), 'speed': 356, 'angle': 160, 'size': 3}, {'position': (3385, 1571), 'speed': 64, 'angle': 164, 'size': 2}, {'position': (3186, 1498), 'speed': 747, 'angle': -134, 'size': 1}, {'position': (3364, 1657), 'speed': 735, 'angle': -40, 'size': 2}, {'position': (2554, 90), 'speed': 233, 'angle': 102, 'size': 3}, {'position': (382, 653), 'speed': 547, 'angle': -26, 'size': 3}, {'position': (1098, 759), 'speed': 745, 'angle': -17, 'size': 3}, {'position': (734, 1010), 'speed': 270, 'angle': -87, 'size': 1}, {'position': (962, 1456), 'speed': 398, 'angle': -94, 'size': 2}, {'position': (2061, 1520), 'speed': 282, 'angle': 149, 'size': 3}, {'position': (7, 7), 'speed': 420, 'angle': -52, 'size': 3}, {'position': (2849, 1510), 'speed': 30, 'angle': -123, 'size': 2}, {'position': (2363, 1208), 'speed': 262, 'angle': 3, 'size': 3}, {'position': (1475, 466), 'speed': 57, 'angle': 47, 'size': 2}, {'position': (1830, 872), 'speed': 361, 'angle': -34, 'size': 3}, {'position': (1077, 577), 'speed': 605, 'angle': -46, 'size': 3}, {'position': (884, 921), 'speed': 71, 'angle': 116, 'size': 2}, {'position': (2693, 110), 'speed': 254, 'angle': -14, 'size': 2}, {'position': (2694, 625), 'speed': 274, 'angle': -122, 'size': 2}, {'position': (1063, 932), 'speed': 666, 'angle': -116, 'size': 2}, {'position': (595, 1485), 'speed': 260, 'angle': -138, 'size': 3}, {'position': (111, 1662), 'speed': 505, 'angle': 169, 'size': 1}, {'position': (507, 841), 'speed': 340, 'angle': -6, 'size': 2}, {'position': (1284, 1157), 'speed': 423, 'angle': 160, 'size': 2}, {'position': (556, 1249), 'speed': 539, 'angle': 107, 'size': 3}, {'position': (3212, 126), 'speed': 221, 'angle': -141, 'size': 3}, {'position': (3110, 1469), 'speed': 295, 'angle': -74, 'size': 2}, {'position': (56, 576), 'speed': 428, 'angle': 178, 'size': 1}, {'position': (2796, 25), 'speed': 369, 'angle': -106, 'size': 1}, {'position': (1229, 1224), 'speed': 16, 'angle': 76, 'size': 3}, {'position': (2210, 962), 'speed': 471, 'angle': -112, 'size': 1}, {'position': (2347, 673), 'speed': 579, 'angle': 53, 'size': 1}, {'position': (3246, 624), 'speed': 335, 'angle': 33, 'size': 2}, {'position': (1788, 114), 'speed': 642, 'angle': -76, 'size': 1}, {'position': (2615, 687), 'speed': 593, 'angle': -32, 'size': 1}],
                            asteroid_states = generate_asteroids(
                                num_asteroids=8,
                                position_range_x=(0, width),
                                position_range_y=(0, height),
                                speed_range=(1, 600),
                                angle_range=(-180, 180),
                                size_range=(1, 1)
                            ),
                            map_size=(width, height),
                            time_limit=3600,
                            ammo_limit_multiplier=0,
                            stop_if_no_ammo=False)

game_settings = {'perf_tracker': True,
                'graphics_type': GraphicsType.Tkinter,
                'realtime_multiplier': 1,
                'graphics_obj': None}

game = KesslerGame(settings=game_settings) # Use this to visualize the game scenario
# game = TrainerEnvironment(settings=game_settings) # Use this for max-speed, no-graphics simulation
pre = time.perf_counter()
score, perf_data = game.run(scenario=my_test_scenario, controllers = [NeoController()])#, DefensiveCamperController()])
print('Scenario eval time: '+str(time.perf_counter()-pre))
print(score.stop_reason)
print('Asteroids hit: ' + str([team.asteroids_hit for team in score.teams]))
print('Deaths: ' + str([team.deaths for team in score.teams]))
print('Accuracy: ' + str([team.accuracy for team in score.teams]))
print('Mean eval time: ' + str([team.mean_eval_time for team in score.teams]))
print('Evaluated frames: ' + str([controller.eval_frames for controller in score.final_controllers]))
