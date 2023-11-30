# ECE 449 Intelligent Systems Engineering
# Fall 2023
# Dr. Scott Dick

# Demonstration of a fuzzy tree-based controller for Kessler Game.
# Please see the Kessler Game Development Guide by Dr. Scott Dick for a
#   detailed discussion of this source code.

from kesslergame import KesslerController # In Eclipse, the name of the library is kesslergame, not src.kesslergame
from typing import Dict, Tuple
from cmath import sqrt
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import math
import numpy as np
import matplotlib as plt

map_size_x = 2560
map_size_y = 1440

#TODO
# We might need a new antecedent for asteroid distance, and based on that apply thrust

class DefensiveCamperController(KesslerController):
        
    def __init__(self):
        self.eval_frames = 0 #What is this?

        # self.targeting_control is the targeting rulebase, which is static in this controller.      
        # Declare variables
        bullet_time = ctrl.Antecedent(np.arange(0,1.0,0.002), 'bullet_time')
        theta_delta = ctrl.Antecedent(np.arange(-1*math.pi,math.pi,0.1), 'theta_delta') # Radians due to Python
        asteroid_distance = ctrl.Antecedent(np.arange(0,228,1), 'asteroid_distance')
        ship_speed = ctrl.Antecedent(np.arange(-240,240,1), 'ship_speed')
        current_ship_thrust = ctrl.Antecedent(np.arange(-450, 450, 1), 'current_ship_thrust')
        ship_pos_x = ctrl.Antecedent(np.arange(0,800,1), 'ship_pos_x')
        ship_pos_y = ctrl.Antecedent(np.arange(0,800,1), 'ship_pos_y')

        ship_turn = ctrl.Consequent(np.arange(-180,180,1), 'ship_turn') # Degrees due to Kessler
        ship_fire = ctrl.Consequent(np.arange(-1,1,0.1), 'ship_fire')
        ship_thrust = ctrl.Consequent(np.arange(-450, 450, 1), 'ship_thrust') 

        #Declare fuzzy sets for bullet_time (how long it takes for the bullet to reach the intercept point)
        bullet_time['S'] = fuzz.trimf(bullet_time.universe,[0,0,0.05])
        bullet_time['M'] = fuzz.trimf(bullet_time.universe, [0,0.05,0.1])
        bullet_time['L'] = fuzz.smf(bullet_time.universe,0.0,0.1)
        
        #Declare fuzzy sets for theta_delta (degrees of turn needed to reach the calculated firing angle)
        theta_delta['NL'] = fuzz.zmf(theta_delta.universe, -1*math.pi/3,-1*math.pi/6)
        theta_delta['NS'] = fuzz.trimf(theta_delta.universe, [-1*math.pi/3,-1*math.pi/6,0])
        theta_delta['Z'] = fuzz.trimf(theta_delta.universe, [-1*math.pi/6,0,math.pi/6])
        theta_delta['PS'] = fuzz.trimf(theta_delta.universe, [0,math.pi/6,math.pi/3])
        theta_delta['PL'] = fuzz.smf(theta_delta.universe,math.pi/6,math.pi/3)
        
        #Declare fuzzy sets for the ship_turn consequent; this will be returned as turn_rate.
        ship_turn['NL'] = fuzz.trimf(ship_turn.universe, [-180,-180,-30])
        ship_turn['NS'] = fuzz.trimf(ship_turn.universe, [-90,-30,0])
        ship_turn['Z'] = fuzz.trimf(ship_turn.universe, [-30,0,30])
        ship_turn['PS'] = fuzz.trimf(ship_turn.universe, [0,30,90])
        ship_turn['PL'] = fuzz.trimf(ship_turn.universe, [30,180,180])
        
        #Declare singleton fuzzy sets for the ship_fire consequent; -1 -> don't fire, +1 -> fire; this will be thresholded
        #   and returned as the boolean 'fire'
        ship_fire['N'] = fuzz.trimf(ship_fire.universe, [-1,-1,0.0])
        ship_fire['Y'] = fuzz.trimf(ship_fire.universe, [0.0,1,1]) 
        
        ship_thrust['NL'] = fuzz.trimf(ship_thrust.universe, [-450, -450, -337.5])
        ship_thrust['NS'] = fuzz.trimf(ship_thrust.universe, [-337.5, -225, -112.5])
        ship_thrust['Z'] = fuzz.trimf(ship_thrust.universe, [-112.5, 0, 112.5])
        ship_thrust['PS'] = fuzz.trimf(ship_thrust.universe, [112.5, 225, 337.5])
        ship_thrust['PL'] = fuzz.trimf(ship_thrust.universe, [337.5, 450, 450])
        
        current_ship_thrust['NL'] = fuzz.trimf(ship_thrust.universe, [-450, -450, -337.5])
        current_ship_thrust['NS'] = fuzz.trimf(ship_thrust.universe, [-337.5, -225, -112.5])
        current_ship_thrust['Z'] = fuzz.trimf(ship_thrust.universe, [-112.5, 0, 112.5])
        current_ship_thrust['PS'] = fuzz.trimf(ship_thrust.universe, [112.5, 225, 337.5])
        current_ship_thrust['PL'] = fuzz.trimf(ship_thrust.universe, [337.5, 450, 450])
        
        ship_speed['NL'] = fuzz.trimf(ship_speed.universe, [-240, -160, -100])
        ship_speed['NS'] = fuzz.trimf(ship_speed.universe, [-100,-60, -10])
        ship_speed['Z'] = fuzz.trimf(ship_speed.universe, [-10, 5, 10])
        ship_speed['PS'] = fuzz.trimf(ship_speed.universe, [10, 60, 100])
        ship_speed['PL'] = fuzz.trimf(ship_speed.universe, [100, 160, 240])
        
        buffer = 70
        min_x = 0
        min_y = 0
        max_x = map_size_x
        max_y = map_size_y
        
        ship_pos_x['close_to_left'] = fuzz.trimf(ship_pos_x.universe, [min_x, min_x + buffer, min_x + buffer])
        ship_pos_x['close_to_right'] = fuzz.trimf(ship_pos_y.universe, [max_x - buffer, max_x - buffer, max_x])

        # Define membership functions for y
        ship_pos_y['close_to_top'] = fuzz.trimf( ship_pos_y.universe, [max_y - buffer * 2, max_y - buffer, max_y])
        ship_pos_y['close_to_bottom'] = fuzz.trimf( ship_pos_y.universe, [min_y, min_y + buffer, min_y + buffer * 2])
        
        position_control_rules = [
            ctrl.Rule(ship_pos_x['close_to_left'] & theta_delta['NL'], (ship_fire['N'], ship_thrust['PL'])),
            ctrl.Rule(ship_pos_x['close_to_left'] & theta_delta['NS'], (ship_fire['N'], ship_thrust['PL'])),
            ctrl.Rule(ship_pos_x['close_to_left'] & theta_delta['Z'], (ship_fire['N'], ship_thrust['PL'])),
            ctrl.Rule(ship_pos_x['close_to_left'] & theta_delta['PL'], (ship_fire['N'], ship_thrust['PL'])),
            ctrl.Rule(ship_pos_x['close_to_left'] & theta_delta['PS'], (ship_fire['N'], ship_thrust['PL'])),
            
        ]

        # Declare each fuzzy rule
        defensive_camper_rules = [
            ctrl.Rule(bullet_time['L'] & theta_delta['NL'], (ship_turn['NL'], ship_fire['N'], ship_thrust['PL'])),
            ctrl.Rule(bullet_time['L'] & theta_delta['NS'], (ship_turn['NS'], ship_fire['Y'], ship_thrust['PL'])),
            ctrl.Rule(bullet_time['L'] & theta_delta['Z'], (ship_turn['Z'], ship_fire['Y'], ship_thrust['PL'])),
            ctrl.Rule(bullet_time['L'] & theta_delta['PS'], (ship_turn['PS'], ship_fire['Y'], ship_thrust['PL'])),
            ctrl.Rule(bullet_time['L'] & theta_delta['PL'], (ship_turn['PL'], ship_fire['N'], ship_thrust['PL'])),

            ctrl.Rule(bullet_time['M'] & theta_delta['NL'], (ship_turn['NL'], ship_fire['N'], ship_thrust['PS'])),
            ctrl.Rule(bullet_time['M'] & theta_delta['NS'], (ship_turn['NS'], ship_fire['Y'], ship_thrust['PS'])),
            ctrl.Rule(bullet_time['M'] & theta_delta['Z'], (ship_turn['Z'], ship_fire['Y'], ship_thrust['PS'])),
            ctrl.Rule(bullet_time['M'] & theta_delta['PS'], (ship_turn['PS'], ship_fire['Y'], ship_thrust['PS'])),
            ctrl.Rule(bullet_time['M'] & theta_delta['PL'], (ship_turn['PL'], ship_fire['N'], ship_thrust['PS'])),

            ctrl.Rule(bullet_time['S'] & theta_delta['NL'], (ship_turn['NL'], ship_fire['N'], ship_thrust['NS'])),
            ctrl.Rule(bullet_time['S'] & theta_delta['NS'], (ship_turn['NS'], ship_fire['Y'], ship_thrust['NS'])),
            ctrl.Rule(bullet_time['S'] & theta_delta['Z'], (ship_turn['Z'], ship_fire['Y'], ship_thrust['NS'])),
            ctrl.Rule(bullet_time['S'] & theta_delta['PS'], (ship_turn['PS'], ship_fire['Y'], ship_thrust['NS'])),
            ctrl.Rule(bullet_time['S'] & theta_delta['PL'], (ship_turn['PL'], ship_fire['N'], ship_thrust['NS']))  
        ]
        
        thrust_control_rules = [
            # If we are moving slowly enough just continue with current thrust
            ctrl.Rule(current_ship_thrust['PL'] & (ship_speed['NS'] | ship_speed['Z'] | ship_speed['PS']), (ship_thrust['PL'])), # We are moving very slowly so let them continue
            ctrl.Rule(current_ship_thrust['PS'] & (ship_speed['NS'] | ship_speed['Z'] | ship_speed['PS']), (ship_thrust['PS'])), # We are moving very slowly so let them continue
            ctrl.Rule(current_ship_thrust['Z'] & (ship_speed['NS'] | ship_speed['Z'] | ship_speed['PS']), (ship_thrust['Z'])),
            ctrl.Rule(current_ship_thrust['NS'] & (ship_speed['NS'] | ship_speed['Z'] | ship_speed['PS']), (ship_thrust['NS'])), # We are moving very slowly so let them continue
            ctrl.Rule(current_ship_thrust['NS'] & (ship_speed['NS'] | ship_speed['Z'] | ship_speed['PS']), (ship_thrust['NL'])), # We are moving very slowly so let them continue
            
            
            ctrl.Rule(current_ship_thrust['NS'] & ship_speed['NL'], (ship_thrust['PS'])), #We are already moving very fast, slow down
            ctrl.Rule(current_ship_thrust['PS'] & ship_speed['PL'], (ship_thrust['NS'])), #We are already moving very fast, slow down
           
            
            ctrl.Rule(current_ship_thrust['NL'] & (ship_speed['NL'] | ship_speed['NS']), (ship_thrust['PS'])), #slow down
            ctrl.Rule(current_ship_thrust['NL'] & ship_speed['Z'], (ship_thrust['NL'])), # We are moving very slowly so let them continue
            ctrl.Rule(current_ship_thrust['NL'] & ship_speed['PL'], (ship_thrust['NL'])), #slow down
            ctrl.Rule(current_ship_thrust['NL'] & ship_speed['PS'], (ship_thrust['NL'])), #slow down

            ctrl.Rule(current_ship_thrust['PL'] & (ship_speed['NL'] | ship_speed['NS']), (ship_thrust['PL'])), #slow down
            ctrl.Rule(current_ship_thrust['PL'] & ship_speed['PL'], (ship_thrust['NL'])), #slow down
            ctrl.Rule(current_ship_thrust['PL'] & ship_speed['PS'], (ship_thrust['NL'])), #slow down
            
            ctrl.Rule(current_ship_thrust['PL'] & (ship_speed['PL'] | ship_speed['PS'] | ship_speed['Z'] | ship_speed['NL'] | ship_speed['PL'] | ship_speed['NS']), (ship_thrust['PL'])), #slow down
            ctrl.Rule(current_ship_thrust['PS'] & (ship_speed['PL'] | ship_speed['PS'] | ship_speed['Z'] | ship_speed['NL'] | ship_speed['PL'] | ship_speed['NS']), (ship_thrust['PS'])), #slow down
            ctrl.Rule(current_ship_thrust['Z'] & (ship_speed['PL'] | ship_speed['PS'] | ship_speed['Z'] | ship_speed['NL'] | ship_speed['PL'] | ship_speed['NS']), (ship_thrust['Z'])), #slow down
            ctrl.Rule(current_ship_thrust['NS'] & (ship_speed['PL'] | ship_speed['PS'] | ship_speed['Z'] | ship_speed['NL'] | ship_speed['PL'] | ship_speed['NS']), (ship_thrust['NS'])), #slow down
            ctrl.Rule(current_ship_thrust['NL'] & (ship_speed['PL'] | ship_speed['PS'] | ship_speed['Z'] | ship_speed['NL'] | ship_speed['PL'] | ship_speed['NS']), (ship_thrust['NL'])) #slow down
        ]
        

        #DEBUG
        #bullet_time.view()
        #theta_delta.view()
        #ship_turn.view()
        #ship_fire.view()
        # ship_thrust.view()
        # Declare the fuzzy controller, add the rules 
        # This is an instance variable, and thus available for other methods in the same object. See notes.                         
        # self.targeting_control = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15])
        
        self.position_conrol = ctrl.ControlSystem()  
        for i in position_control_rules:
            self.position_conrol.addrule(i)
         
        self.targeting_control = ctrl.ControlSystem()
        for i in defensive_camper_rules:
            self.targeting_control.addrule(i)
            
        self.thrust_control = ctrl.ControlSystem()
        for i in thrust_control_rules:
            self.thrust_control.addrule(i)
        

    def actions(self, ship_state: Dict, game_state: Dict) -> Tuple[float, float, bool]:
        #print('Self:')
        #print(self)
        #print('Game state:')
        #print(game_state)
        #print('Ship state:')
        #print(ship_state)
        """
        Method processed each time step by this controller.
        """
        # These were the constant actions in the basic demo, just spinning and shooting.
        #thrust = 0 <- How do the values scale with asteroid velocity vector?
        #turn_rate = 90 <- How do the values scale with asteroid velocity vector?
        
        # Answers: Asteroid position and velocity are split into their x,y components in a 2-element ?array each.
        # So are the ship position and velocity, and bullet position and velocity. 
        # Units appear to be meters relative to origin (where?), m/sec, m/sec^2 for thrust.
        # Everything happens in a time increment: delta_time, which appears to be 1/30 sec; this is hardcoded in many places.
        # So, position is updated by multiplying velocity by delta_time, and adding that to position.
        # Ship velocity is updated by multiplying thrust by delta time.
        # Ship position for this time increment is updated after the the thrust was applied.
        
        # Goal: demonstrate processing of game state, fuzzy controller, intercept computation 
        # Intercept-point calculation derived from the Law of Cosines, see notes for details and citation.

        # Function to compute the wrapped distance between two points in one dimension
        def wrapped_distance(coord1, coord2, max_coord):
            direct_dist = abs(coord1 - coord2)
            wrap_dist = min(coord1, coord2) + max_coord - max(coord1, coord2)
            return min(direct_dist, wrap_dist)

        # Function to duplicate asteroid positions for wraparound
        def duplicate_asteroids_for_wraparound(asteroid, max_x, max_y):
            # Original position
            duplicates = [asteroid]

            # Original X and Y coordinates
            orig_x, orig_y = asteroid["position"]

            # Generate positions for the duplicates
            for dx in [-max_x, 0, max_x]:
                for dy in [-max_y, 0, max_y]:
                    if dx == 0 and dy == 0:
                        continue  # Skip the original asteroid position
                    new_pos = (orig_x + dx, orig_y + dy)
                    duplicate = asteroid.copy()
                    duplicate["position"] = new_pos
                    duplicates.append(duplicate)

            return duplicates

        # Field size is hardcoded in map_size_x and map_size_y

        # Find the closest asteroid (disregards asteroid velocity)
        ship_pos_x = ship_state["position"][0]     # See src/kesslergame/ship.py in the KesslerGame Github
        ship_pos_y = ship_state["position"][1]       
        closest_asteroid = None
        
        closest_asteroid = None
        closest_asteroid_wraparound = None
        for a in game_state["asteroids"]:
            # Duplicate asteroid positions for wraparound
            duplicated_asteroids = duplicate_asteroids_for_wraparound(a, map_size_x, map_size_y)
            # Find distance without wraparound
            curr_dist = (a["position"][0] - ship_pos_x)**2 + (a["position"][1] - ship_pos_y)**2

            if closest_asteroid is None:
                    closest_asteroid = dict(aster=a, dist=curr_dist)
            else:
                if closest_asteroid["dist"] > curr_dist:
                    closest_asteroid["aster"] = a
                    closest_asteroid["dist"] = curr_dist

            for dup_asteroid in duplicated_asteroids:
                # Calculate wrapped distance for both x and y coordinates
                #wrapped_dist_x = wrapped_distance(ship_pos_x, dup_asteroid["position"][0], max_x)
                #wrapped_dist_y = wrapped_distance(ship_pos_y, dup_asteroid["position"][1], max_y)

                # Calculate Euclidean distance considering wraparound
                curr_dist = (dup_asteroid["position"][0] - ship_pos_x)**2 + (dup_asteroid["position"][1] - ship_pos_y)**2

                if closest_asteroid_wraparound is None:
                    closest_asteroid_wraparound = dict(aster=dup_asteroid, dist=curr_dist)
                else:
                    if closest_asteroid_wraparound["dist"] > curr_dist:
                        closest_asteroid_wraparound["aster"] = dup_asteroid
                        closest_asteroid_wraparound["dist"] = curr_dist

        #closest_asteroid["dist"] = math.sqrt(closest_asteroid["dist"])
        closest_asteroid_wraparound["dist"] = math.sqrt(closest_asteroid_wraparound["dist"])
        # For now, don't use the two asteroids separately and only use the wraparound one
        closest_asteroid = closest_asteroid_wraparound
        # closest_asteroid now contains the nearest asteroid considering wraparound

        # closest_asteroid is now the nearest asteroid object. 
        # Calculate intercept time given ship & asteroid position, asteroid velocity vector, bullet speed (not direction).
        # Based on Law of Cosines calculation, see notes.
        
        # Side D of the triangle is given by closest_asteroid.dist. Need to get the asteroid-ship direction
        #    and the angle of the asteroid's current movement.
        # REMEMBER TRIG FUNCTIONS ARE ALL IN RADAINS!!!
        
        
        asteroid_ship_x = ship_pos_x - closest_asteroid["aster"]["position"][0]
        asteroid_ship_y = ship_pos_y - closest_asteroid["aster"]["position"][1]
        
        asteroid_ship_theta = math.atan2(asteroid_ship_y,asteroid_ship_x)
        
        asteroid_direction = math.atan2(closest_asteroid["aster"]["velocity"][1], closest_asteroid["aster"]["velocity"][0]) # Velocity is a 2-element array [vx,vy].
        my_theta2 = asteroid_ship_theta - asteroid_direction
        cos_my_theta2 = math.cos(my_theta2)
        # Need the speeds of the asteroid and bullet. speed * time is distance to the intercept point
        asteroid_vel = math.sqrt(closest_asteroid["aster"]["velocity"][0]**2 + closest_asteroid["aster"]["velocity"][1]**2)
        bullet_speed = 800 # Hard-coded bullet speed from bullet.py
        
        # Determinant of the quadratic formula b^2-4ac
        targ_det = (-2 * closest_asteroid["dist"] * asteroid_vel * cos_my_theta2)**2 - (4*(asteroid_vel**2 - bullet_speed**2) * closest_asteroid["dist"])
        
        # Combine the Law of Cosines with the quadratic formula for solve for intercept time. Remember, there are two values produced.
        intrcpt1 = ((2 * closest_asteroid["dist"] * asteroid_vel * cos_my_theta2) + math.sqrt(targ_det)) / (2 * (asteroid_vel**2 -bullet_speed**2))
        intrcpt2 = ((2 * closest_asteroid["dist"] * asteroid_vel * cos_my_theta2) - math.sqrt(targ_det)) / (2 * (asteroid_vel**2-bullet_speed**2))
        
        # Take the smaller intercept time, as long as it is positive; if not, take the larger one.
        if intrcpt1 > intrcpt2:
            if intrcpt2 >= 0:
                bullet_t = intrcpt2
            else:
                bullet_t = intrcpt1
        else:
            if intrcpt1 >= 0:
                bullet_t = intrcpt1
            else:
                bullet_t = intrcpt2
                
        # Calculate the intercept point. The work backwards to find the ship's firing angle my_theta1.
        intrcpt_x = closest_asteroid["aster"]["position"][0] + closest_asteroid["aster"]["velocity"][0] * bullet_t
        intrcpt_y = closest_asteroid["aster"]["position"][1] + closest_asteroid["aster"]["velocity"][1] * bullet_t
        
        my_theta1 = math.atan2((intrcpt_y - ship_pos_y),(intrcpt_x - ship_pos_x))
        
        # Lastly, find the difference betwwen firing angle and the ship's current orientation. BUT THE SHIP HEADING IS IN DEGREES.
        shooting_theta = my_theta1 - ((math.pi/180)*ship_state["heading"])
        
        # Wrap all angles to (-pi, pi)
        shooting_theta = (shooting_theta + math.pi) % (2 * math.pi) - math.pi
        
        
        # position Controller to stay near center
    
        
        # Pass the inputs to the rulebase and fire it
        shooting = ctrl.ControlSystemSimulation(self.targeting_control,flush_after_run=1)
        
        shooting.input['bullet_time'] = bullet_t
        shooting.input['theta_delta'] = shooting_theta
        
        shooting.compute()
        
        # Get the defuzzified outputs
        turn_rate = shooting.output['ship_turn']
        thrust = shooting.output['ship_thrust']
        if shooting.output['ship_fire'] >= 0 and not ship_state['is_respawning']:
            fire = True
        else:
            fire = False
               
        # And return your three outputs to the game simulation. Controller algorithm complete.
        
        # position = ctrl.ControlSystemSimulation(self.position_conrol,flush_after_run=1)
        # if(ship_pos_x >= 730 or ship_pos_x <= 70):
        #     position.input['ship_pos_x'] = ship_pos_x
        #     # position.input['ship_pos_y'] = ship_pos_y
        #     position.input['theta_delta'] = shooting_theta
        
        #     position.compute()
        #     # turn_rate = position.output['ship_turn']
        #     thrust = position.output['ship_thrust']
        
        # this controller will look at out current speed and thurst and adjust so we dont uncontrollably runaway
        thurst_controller = ctrl.ControlSystemSimulation(self.thrust_control,flush_after_run=1)
        
        thurst_controller.input['ship_speed'] = ship_state['speed']
        thurst_controller.input['current_ship_thrust'] = thrust     
        thurst_controller.compute()   
        
        thrust = thurst_controller.output['ship_thrust']
        #if ship_state['is_respawning']:
        #    print('IS RESPAWNING')
        self.eval_frames +=1
        #DEBUG
        #print(thrust, bullet_t, shooting_theta, turn_rate, fire)
        #print("thrust is " + str(thrust) + "\n" + "turn rate is " + str(turn_rate) + "\n" + "fire is " + str(fire) + "\n")
        return thrust, turn_rate, fire

    @property
    def name(self) -> str:
        return "Defensive Camper Controller"