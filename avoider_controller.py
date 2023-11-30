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




class AvoiderController(KesslerController):
    
    
        
    def __init__(self):
        self.eval_frames = 0 #What is this?

        # self.targeting_control is the targeting rulebase, which is static in this controller.      
        # Declare variables
        bullet_time = ctrl.Antecedent(np.arange(0,1.0,0.002), 'bullet_time')
        # ship_time = ctrl.Antecedent(np.arange(-2, 2.001, 0.001), 'ship_time')
        ship_speed = ctrl.Antecedent(np.arange(0.0, 240.001, 0.001), 'ship_speed') 
        theta_delta = ctrl.Antecedent(np.arange(-1*math.pi,math.pi,0.1), 'theta_delta') # Radians due to Python
        asteroid_distance = ctrl.Antecedent(np.arange(0,228,1), 'asteroid_distance')

        ship_turn = ctrl.Consequent(np.arange(-180,180,1), 'ship_turn') # Degrees due to Kessler
        ship_fire = ctrl.Consequent(np.arange(-1,1,0.1), 'ship_fire')
        ship_thrust = ctrl.Consequent(np.arange(-450, 450, 1), 'ship_thrust') 
        
        #Declare fuzzy sets for bullet_time (how long it takes for the bullet to reach the intercept point)
        bullet_time['S'] = fuzz.trimf(bullet_time.universe,[0,0,0.05])
        bullet_time['M'] = fuzz.trimf(bullet_time.universe, [0,0.05,0.1])
        bullet_time['L'] = fuzz.smf(bullet_time.universe,0.0,0.1)
        
        #Declare fuzzy sets for theta_delta (degrees of turn needed to reach the calculated firing angle)
        theta_delta['NL'] = fuzz.zmf(theta_delta.universe, -2*math.pi/3, -math.pi/3)
        theta_delta['NS'] = fuzz.trimf(theta_delta.universe, [-2*math.pi/3, -math.pi/3, 0])
        theta_delta['Z'] = fuzz.trimf(theta_delta.universe, [-math.pi/3, 0, math.pi/3])
        theta_delta['PS'] = fuzz.trimf(theta_delta.universe, [0, math.pi/3, 2*math.pi/3])
        theta_delta['PL'] = fuzz.smf(theta_delta.universe, math.pi/3, 2*math.pi/3)
        
        #Declare fuzzy sets for the ship_turn consequent; this will be returned as turn_rate.
        ship_turn['NL'] = fuzz.trimf(ship_turn.universe, [-180,-180,-30])
        ship_turn['NS'] = fuzz.trimf(ship_turn.universe, [-90,-30,0])
        ship_turn['Z'] = fuzz.trimf(ship_turn.universe, [-30,0,30])
        ship_turn['PS'] = fuzz.trimf(ship_turn.universe, [0,30,90])
        ship_turn['PL'] = fuzz.trimf(ship_turn.universe, [30,180,180])
        
        #Declare singleton fuzzy sets for the ship_fire consequent; -1 -> don't fire, +1 -> fire; this will be  thresholded
        #   and returned as the boolean 'fire'
        ship_fire['N'] = fuzz.trimf(ship_fire.universe, [-1,-1,0.0])
        ship_fire['Y'] = fuzz.trimf(ship_fire.universe, [0.0,1,1])
        
        #declare ship thrust
        ship_thrust['NL'] = fuzz.trimf(ship_thrust.universe, [-450, -450, -337.5])
        ship_thrust['NS'] = fuzz.trimf(ship_thrust.universe, [-337.5, -225, -112.5])
        ship_thrust['Z'] = fuzz.trimf(ship_thrust.universe, [-112.5, 0, 112.5])
        ship_thrust['PS'] = fuzz.trimf(ship_thrust.universe, [112.5, 225, 337.5])
        ship_thrust['PL'] = fuzz.trimf(ship_thrust.universe, [337.5, 450, 450])
        
        # Declare fuzzy sets for ship_time (how long it takes for the ship and asteroid to intercept)
        # ship_time['NL'] = fuzz.trimf(ship_time.universe, [-5, -3.33, -1.67])
        # ship_time['NM'] = fuzz.trimf(ship_time.universe, [-3.33, -1.67, 0])
        # ship_time['NS'] = fuzz.trimf(ship_time.universe, [-1.67, 0, 1.67])
        # ship_time['PS'] = fuzz.trimf(ship_time.universe, [0, 1.67, 3.33])
        # ship_time['PL'] = fuzz.trimf(ship_time.universe, [1.67, 3.33, 5])
        
        asteroid_distance['VC'] = fuzz.trimf(asteroid_distance.universe, [0, 35, 70])  # very close
        asteroid_distance['C'] = fuzz.trimf(asteroid_distance.universe, [70, 90, 114])  # close
        asteroid_distance['M'] = fuzz.trimf(asteroid_distance.universe, [114, 130, 152])  # medium
        asteroid_distance['F'] = fuzz.trimf(asteroid_distance.universe, [152, 170, 190])  # far
        asteroid_distance['VF'] = fuzz.trimf(asteroid_distance.universe, [190, 200, 228])  # very far
        
        ship_speed['Very Slow'] = fuzz.trimf(ship_speed.universe, [-5, 40, 80])
        ship_speed['Slow'] = fuzz.trimf(ship_speed.universe, [40, 80, 120])
        ship_speed['Moderate'] = fuzz.trimf(ship_speed.universe, [80, 120, 160])
        ship_speed['Fast'] = fuzz.trimf(ship_speed.universe, [120, 160, 200])
        ship_speed['Very Fast'] = fuzz.trimf(ship_speed.universe, [160, 200, 240])

        threat_rules = [
            ctrl.Rule(asteroid_distance['VC'] & theta_delta['Z'], 
                            (ship_turn['Z'], ship_fire['Y'], ship_thrust['NL'])),
            
            ctrl.Rule(asteroid_distance['VC'] & theta_delta['NS'], 
                            (ship_turn['NS'], ship_fire['Y'], ship_thrust['NL'])),
            
            ctrl.Rule(asteroid_distance['VC'] & theta_delta['PS'], 
                            (ship_turn['PS'], ship_fire['Y'], ship_thrust['NL'])),
            
            ctrl.Rule(asteroid_distance['M'] & theta_delta['PS'], 
                            (ship_turn['PS'], ship_fire['Y'], ship_thrust['NL'])),
            
            ctrl.Rule((asteroid_distance['M'] | asteroid_distance['VC']) & theta_delta['PL'], 
                            (ship_turn['PL'], ship_fire['N'], ship_thrust['PS'])),
             
            ctrl.Rule((asteroid_distance['M'] | asteroid_distance['VC']) & theta_delta['NL'], 
                            (ship_turn['NL'], ship_fire['N'], ship_thrust['PS'])),
            
            ctrl.Rule(asteroid_distance['M'] & theta_delta['NS'], 
                            (ship_turn['NS'], ship_fire['Y'], ship_thrust['NL'])),
            
           ctrl.Rule((asteroid_distance['M'] | asteroid_distance['F'] |  asteroid_distance['VF']) & theta_delta['PS'], 
                            (ship_turn['PS'], ship_fire['N'], ship_thrust['PS'])),
            
            ctrl.Rule((asteroid_distance['M'] | asteroid_distance['F'] |  asteroid_distance['VF']) & theta_delta['NS'], 
                            (ship_turn['NS'], ship_fire['N'], ship_thrust['PS'])),
            
            ctrl.Rule((asteroid_distance['M'] | asteroid_distance['F'] |  asteroid_distance['VF']) & theta_delta['PL'], 
                            (ship_turn['PL'], ship_fire['N'], ship_thrust['PS'])),
            
            ctrl.Rule((asteroid_distance['M'] | asteroid_distance['F'] |  asteroid_distance['VF']) & theta_delta['NL'], 
                            (ship_turn['NL'], ship_fire['N'], ship_thrust['PS'])),
            
            ctrl.Rule((asteroid_distance['M'] | asteroid_distance['F'] |  asteroid_distance['VF']) & theta_delta['Z'], 
                            (ship_turn['Z'], ship_fire['N'], ship_thrust['Z'])),
            
            ctrl.Rule((asteroid_distance['VC'] | asteroid_distance['C'] | asteroid_distance['M'] | asteroid_distance['F'] |  asteroid_distance['VF'])
                            & (theta_delta['NL'] | theta_delta['NS'] | theta_delta['Z'] | theta_delta['PS'] | theta_delta['PL']), 
                        (ship_turn['Z'], ship_fire['Y'], ship_thrust['PS']))
        ]
        
        
        
        
        
        
        
        
        self.avoider_control = ctrl.ControlSystem()
        
        for rule in threat_rules:
            self.avoider_control.addrule(rule)
            


        

    def actions(self, ship_state: Dict, game_state: Dict) -> Tuple[float, float, bool]:
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
        

        # My demonstration controller does not move the ship, only rotates it to shoot the nearest asteroid.
        # Goal: demonstrate processing of game state, fuzzy controller, intercept computation 
        # Intercept-point calculation derived from the Law of Cosines, see notes for details and citation.

        # Find the closest asteroid (disregards asteroid velocity)
        ship_pos_x = ship_state["position"][0]     # See src/kesslergame/ship.py in the KesslerGame Github
        ship_pos_y = ship_state["position"][1]
        
        
        # Ships velocity componenets
        ship_velocity_x = ship_state["velocity"][0] 
        ship_velocity_y = ship_state["velocity"][1]
              
        closest_asteroid = None
        
        for a in game_state["asteroids"]:
            #Loop through all asteroids, find minimum Eudlidean distance
            curr_dist = math.sqrt((ship_pos_x - a["position"][0])**2 + (ship_pos_y - a["position"][1])**2)
            if closest_asteroid is None :
                # Does not yet exist, so initialize first asteroid as the minimum. Ugh, how to do?
                closest_asteroid = dict(aster = a, dist = curr_dist)
                
            else:    
                # closest_asteroid exists, and is thus initialized. 
                if closest_asteroid["dist"] > curr_dist:
                    # New minimum found
                    closest_asteroid["aster"] = a
                    closest_asteroid["dist"] = curr_dist

        # closest_asteroid is now the nearest asteroid object. 
        # Calculate intercept time given ship & asteroid position, asteroid velocity vector, bullet speed (not direction).
        # Based on Law of Cosines calculation, see notes.
        
        # Side D of the triangle is given by closest_asteroid.dist. Need to get the asteroid-ship direction
        #    and the angle of the asteroid's current movement.
        # REMEMBER TRIG FUNCTIONS ARE ALL IN RADAINS!!!
        
        # Relative motion between ship and asteroid
        relative_velocity_x = closest_asteroid["aster"]["velocity"][0] - ship_velocity_x
        relative_velocity_y = closest_asteroid["aster"]["velocity"][1] - ship_velocity_y
        
        # Calculate the components of the relative position vector
        relative_position_x = ship_pos_x - closest_asteroid["aster"]["position"][0]
        relative_position_y = ship_pos_y - closest_asteroid["aster"]["position"][1]
        
        ship_t = (relative_position_x * relative_velocity_x + relative_position_y * relative_velocity_y) / (relative_velocity_x**2 + relative_velocity_y**2)
        if ship_t > 2:
            ship_t = 2
        
        if ship_t < -2:
            ship_t = -2
        # Ensure the intercept time is positive
        
        asteroid_ship_x = ship_pos_x - closest_asteroid["aster"]["position"][0]
        asteroid_ship_y = ship_pos_y - closest_asteroid["aster"]["position"][1]
        
        asteroid_ship_theta = math.atan2(asteroid_ship_y,asteroid_ship_x)
        
        asteroid_direction = math.atan2(closest_asteroid["aster"]["velocity"][1], closest_asteroid["aster"]["velocity"][0]) # Velocity is a 2-element array [vx,vy].
        my_theta2 = asteroid_ship_theta - asteroid_direction
        cos_my_theta2 = math.cos(my_theta2)
        # Need the speeds of the asteroid and bullet. speed * time is distance to the intercept point
        asteroid_vel = math.sqrt(closest_asteroid["aster"]["velocity"][0]**2 + closest_asteroid["aster"]["velocity"][1]**2)
        bullet_speed = 800 + ship_state['speed']# Hard-coded bullet speed from bullet.py
        
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
        
        # Pass the inputs to the rulebase and fire it

        threat = ctrl.ControlSystemSimulation(self.avoider_control, flush_after_run=1)
        
        
        
        # threat.input['bullet_time'] = bullet_t
        threat.input['theta_delta'] = shooting_theta
        # threat.input['bullet_time'] = bullet_t
        threat.input['asteroid_distance'] = closest_asteroid['dist'] - closest_asteroid['aster']['radius']
        # threat.input['ship_time'] = ship_t
        # threat.input['ship_speed'] = ship_state['speed']
        threat.compute()
        
            
        # Get the defuzzified outputs
        turn_rate = threat.output['ship_turn']
        
        if threat.output['ship_fire'] >= 0:
            fire = True
        else:
            fire = False
               
        # And return your three outputs to the game simulation. Controller algorithm complete.
        thrust = threat.output['ship_thrust']
        
        self.eval_frames +=1
        
        #DEBUG
        print(thrust, bullet_t, shooting_theta, turn_rate, fire, ship_t)
        
        return thrust, turn_rate, fire

    @property
    def name(self) -> str:
        return "Threat Controller"