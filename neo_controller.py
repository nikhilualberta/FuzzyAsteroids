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
from collections import deque
import copy

#TODO
# We might need a new antecedent for asteroid distance, and based on that apply thrust

time_delta = 1/30

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

def find_closest_asteroid(game_state, ship_state, shot_at_asteroids, time_to_simulate = 4.0):
    #print("Shot at asteroids:")
    #print(shot_at_asteroids)
    game_state['map_size']
    asteroids = game_state['asteroids']
    closest_asteroid = None
    ship_x = ship_state['position'][0]
    ship_y = ship_state['position'][1]
    min_x = 0
    min_y = 0
    max_x = game_state['map_size'][0]
    max_y = game_state['map_size'][1]

    def check_coordinate_bounds(x, y):
        if min_x <= x <= max_x and min_y <= y <= max_y:
            return True
        else:
            return False

    def check_intercept_bounds(candidate_asteroid, additional_future_timesteps = 0):
        _, _, intercept_x, intercept_y = calculate_interception(ship_x, ship_y, candidate_asteroid["position"][0] + (2 + additional_future_timesteps) * time_delta * candidate_asteroid["velocity"][0], candidate_asteroid["position"][1] + (2 + additional_future_timesteps) * time_delta * candidate_asteroid["velocity"][1], candidate_asteroid["velocity"][0], candidate_asteroid["velocity"][1], ship_state["heading"])
        return check_coordinate_bounds(intercept_x, intercept_y)
        

    def check_intercept_feasibility(candidate_asteroid):
        if not check_intercept_bounds(candidate_asteroid):
            # If we shoot at this instant and it'll still be out of bounds, don't bother
            return False
        # Intercept is within bounds, but only guaranteed if we shoot at this instant!
        # Check whether we have enough time to turn our camera to aim at it, and still intercept it within bounds.
        # If we can't turn in one second far enough to shoot it before it goes off screen, don't even bother dude
        turn_rate_range = 180.0 # How many degrees we can turn in 1 second
        timesteps_from_now = 0.0 # We allow fractional timesteps
        iterations = 10
        max_bullet_time = 200
        for it in range(iterations):
            if timesteps_from_now > max_bullet_time:
                # Fuggetaboutit, we ain't hunting down this asteroid until it loops around at least
                break
            print(f"Iteration {it}, current total timesteps from now: {timesteps_from_now}")
            _, shooting_theta, _, _ = calculate_interception(ship_x, ship_y, candidate_asteroid["position"][0] + (2 + timesteps_from_now) * time_delta * candidate_asteroid["velocity"][0], candidate_asteroid["position"][1] + (2 + timesteps_from_now) * time_delta * candidate_asteroid["velocity"][1], candidate_asteroid["velocity"][0], candidate_asteroid["velocity"][1], ship_state["heading"])
            
            shooting_theta_deg = shooting_theta * 180.0 / math.pi
            shooting_theta_deg = abs(shooting_theta_deg)
            print(shooting_theta_deg / (turn_rate_range * time_delta))
            number_of_timesteps_itll_take_to_turn = shooting_theta_deg / (turn_rate_range * time_delta)
            
            timesteps_from_now += number_of_timesteps_itll_take_to_turn
        
        if check_intercept_bounds(candidate_asteroid, math.ceil(timesteps_from_now)):
            # If at that future time, we can shoot and still intercept the bullet...
            # Iterate once more to see whether we can hit it after we turn
            return True
        else:
            return False
    
    def check_collision(a_x, a_y, a_r, b_x, b_y, b_r):
        #print(a_x, a_y, a_r, b_x, b_y, b_r)
        collision_fudge_factor = 1.03
        if (a_x - b_x)**2 + (a_y - b_y)**2 <= collision_fudge_factor*(a_r + b_r)**2:
            return True
        else:
            return False

    asteroids = game_state['asteroids']
    ship_radius = ship_state['radius']
    simulated_asteroids = copy.deepcopy(asteroids)
    closest_asteroid = None
    speedup_factor = 1
    breakout_flag = False
    for i in range(math.ceil(time_to_simulate / time_delta / speedup_factor)):
        if breakout_flag:
            break
        for ind, a in enumerate(simulated_asteroids):
            a['position'] = [pos + speedup_factor*v*time_delta for pos, v in zip(a['position'], a['velocity'])]
            if check_collision(a['position'][0], a['position'][1], a['radius'], ship_x, ship_y, ship_radius):
                closest_asteroid = asteroids[ind]
                #print(f"Closest asteroid to hit me is at {i*time_delta*speedup_factor} seconds from now")
                #print(closest_asteroid)
                breakout_flag = True
                break
    if closest_asteroid is not None:
        print(f"Asteroid getting close, <={time_to_simulate} seconds away. Defend!")
        return closest_asteroid

    # Target physically closest asteroid
    '''
    closest_asteroid = None
    closest_asteroid_square_dist = 3840**2
    for a in asteroids:
        curr_dist_square = (a["position"][0] - ship_x)**2 + (a["position"][1] - ship_y)**2
        if curr_dist_square < closest_asteroid_square_dist:
            if (a["velocity"][0], a["velocity"][1], a["radius"]) not in shot_at_asteroids:
                closest_asteroid = a
                closest_asteroid_square_dist = curr_dist_square
            elif (a["velocity"][0], a["velocity"][1], a["radius"]) in shot_at_asteroids:
                print('THIS ASTEROID ALREADY IN LIST!!!')
    '''
    #print(closest_asteroid)

    # Find closest angular distance so aiming is faster
    closest_asteroid = None
    closest_asteroid_angular_dist = 10000000
    ship_heading = ship_state['heading']
    print(f"Num asteroids: {len(asteroids)}, num shot at already: {len(shot_at_asteroids)}")
    for a_no_wraparound in asteroids:
        duplicated_asteroids = duplicate_asteroids_for_wraparound(a_no_wraparound, max_x, max_y)
        for a in duplicated_asteroids:
            theta = math.atan2(a['position'][1] - ship_y, a['position'][0] - ship_x)
            
            # Lastly, find the difference between firing angle and the ship's current orientation. BUT THE SHIP HEADING IS IN DEGREES.
            shooting_theta = theta - ((math.pi/180)*ship_heading)

            # Wrap all angles to (-pi, pi)
            curr_angular_dist = abs((shooting_theta + math.pi) % (2 * math.pi) - math.pi)

            if curr_angular_dist < closest_asteroid_angular_dist and check_intercept_feasibility(a):
                if (a["velocity"][0], a["velocity"][1], a["radius"]) not in shot_at_asteroids:
                    closest_asteroid = a
                    closest_asteroid_angular_dist = curr_angular_dist
                elif (a["velocity"][0], a["velocity"][1], a["radius"]) in shot_at_asteroids:
                    #print('THIS ASTEROID ALREADY IN LIST!!!')
                    pass
    if closest_asteroid is None:
        closest_asteroid = asteroids[0]
    return closest_asteroid


def calculate_interception(ship_pos_x, ship_pos_y, asteroid_pos_x, asteroid_pos_y, asteroid_vel_x, asteroid_vel_y, ship_heading):
    # Calculate intercept time given ship & asteroid position, asteroid velocity vector, bullet speed (not direction).
    # Based on Law of Cosines calculation, see notes.
    
    # Side D of the triangle is given by closest_asteroid.dist. Need to get the asteroid-ship direction
    #    and the angle of the asteroid's current movement.
    # REMEMBER TRIG FUNCTIONS ARE ALL IN RADAINS!!!
    
    asteroid_ship_x = ship_pos_x - asteroid_pos_x
    asteroid_ship_y = ship_pos_y - asteroid_pos_y
    
    asteroid_ship_theta = math.atan2(asteroid_ship_y, asteroid_ship_x)
    
    asteroid_direction = math.atan2(asteroid_vel_y, asteroid_vel_x) # Velocity is a 2-element array [vx,vy].
    my_theta2 = asteroid_ship_theta - asteroid_direction
    cos_my_theta2 = math.cos(my_theta2)
    # Need the speeds of the asteroid and bullet. speed * time is distance to the intercept point
    asteroid_vel = math.sqrt(asteroid_vel_x**2 + asteroid_vel_y**2)
    bullet_speed = 800 # Hard-coded bullet speed from bullet.py
    
    # Discriminant of the quadratic formula b^2-4ac
    asteroid_dist = math.sqrt((ship_pos_x - asteroid_pos_x)**2 + (ship_pos_y - asteroid_pos_y)**2)
    targ_det = (-2 * asteroid_dist * asteroid_vel * cos_my_theta2)**2 - (4*(asteroid_vel**2 - bullet_speed**2) * asteroid_dist**2)
    if targ_det < 0:
        # There is no intercept. Return a fake intercept
        return 100000, 100000, -100, -100
    # Combine the Law of Cosines with the quadratic formula for solve for intercept time. Remember, there are two values produced.
    intrcpt1 = ((2 * asteroid_dist * asteroid_vel * cos_my_theta2) + math.sqrt(targ_det)) / (2 * (asteroid_vel**2 - bullet_speed**2))
    intrcpt2 = ((2 * asteroid_dist * asteroid_vel * cos_my_theta2) - math.sqrt(targ_det)) / (2 * (asteroid_vel**2 - bullet_speed**2))
    
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
    intercept_x = asteroid_pos_x + asteroid_vel_x * bullet_t
    intercept_y = asteroid_pos_y + asteroid_vel_y * bullet_t
    
    my_theta1 = math.atan2(intercept_y - ship_pos_y, intercept_x - ship_pos_x)
    
    # Lastly, find the difference betwwen firing angle and the ship's current orientation. BUT THE SHIP HEADING IS IN DEGREES.
    shooting_theta = my_theta1 - ((math.pi/180)*ship_heading)

    # Wrap all angles to (-pi, pi)
    shooting_theta = (shooting_theta + math.pi) % (2 * math.pi) - math.pi

    return bullet_t, shooting_theta, intercept_x, intercept_y


class NeoController(KesslerController):
        
    def __init__(self):
        self.init_done = False

        self.pid_integral = 0
        self.pid_previous_error = 0
        self.previously_targetted_asteroid = None

        self.shot_at_asteroids = {} # Dict of tuples, with the values corresponding to the timesteps we need to wait until they can be shot at again
        self.fire_on_frames = set()
        self.last_time_fired = -1

    def finish_init(self, game_state):
        self.eval_frames = 0 #What is this?

        # self.targeting_control is the targeting rulebase, which is static in this controller.      
        # Declare variables
        bullet_time = ctrl.Antecedent(np.arange(0,1.0,0.002), 'bullet_time')
        theta_delta = ctrl.Antecedent(np.arange(-1*math.pi,math.pi,0.1), 'theta_delta') # Radians due to Python
        theta_delta_integral = ctrl.Antecedent(np.arange(-1*math.pi,math.pi,0.1), 'theta_delta_integral') # Radians due to Python
        theta_delta_derivative = ctrl.Antecedent(np.arange(-1*math.pi,math.pi,0.1), 'theta_delta_derivative') # Radians due to Python
        theta_pid_input = ctrl.Antecedent(np.arange(-1*math.pi,math.pi,0.1), 'theta_pid_input') # Radians due to Python
        asteroid_distance = ctrl.Antecedent(np.arange(0,228,1), 'asteroid_distance')
        ship_speed = ctrl.Antecedent(np.arange(-240,240,1), 'ship_speed')
        current_ship_thrust = ctrl.Antecedent(np.arange(-480, 480, 1), 'current_ship_thrust')
        ship_pos_x = ctrl.Antecedent(np.arange(0,800,1), 'ship_pos_x')
        ship_pos_y = ctrl.Antecedent(np.arange(0,800,1), 'ship_pos_y')

        ship_turn = ctrl.Consequent(np.arange(-180,180,1), 'ship_turn') # Degrees due to Kessler
        ship_fire = ctrl.Consequent(np.arange(-1,1,0.1), 'ship_fire')
        ship_thrust = ctrl.Consequent(np.arange(-480, 480, 1), 'ship_thrust') 

        # Declare fuzzy sets for bullet_time (how long it takes for the bullet to reach the intercept point)
        bullet_time['S'] = fuzz.trimf(bullet_time.universe,[0,0,0.05])
        bullet_time['M'] = fuzz.trimf(bullet_time.universe, [0,0.05,0.1])
        bullet_time['L'] = fuzz.smf(bullet_time.universe,0.0,0.1)

        # Declare fuzzy sets for theta_delta (degrees of turn needed to reach the calculated firing angle)
        angle_small_threshold = math.pi/40
        angle_large_threshold = math.pi
        theta_delta['NL'] = fuzz.zmf(theta_delta.universe, -1*angle_large_threshold, -1*angle_small_threshold)
        theta_delta['NS'] = fuzz.trimf(theta_delta.universe, [-1*angle_large_threshold, -1*angle_small_threshold, 0])
        theta_delta['Z'] = fuzz.trimf(theta_delta.universe, [-1*angle_small_threshold, 0, angle_small_threshold])
        theta_delta['PS'] = fuzz.trimf(theta_delta.universe, [0, angle_small_threshold, angle_large_threshold])
        theta_delta['PL'] = fuzz.smf(theta_delta.universe, angle_small_threshold, angle_large_threshold)

        pid_scale_factor = 3.0
        theta_pid_input['NL'] = fuzz.zmf(theta_pid_input.universe, -1*pid_scale_factor*math.pi/3, -1*pid_scale_factor*math.pi/6)
        theta_pid_input['NS'] = fuzz.trimf(theta_pid_input.universe, [-1*pid_scale_factor*math.pi/3, -1*pid_scale_factor*math.pi/6, 0])
        theta_pid_input['Z'] = fuzz.trimf(theta_pid_input.universe, [-1*pid_scale_factor*math.pi/6, 0, pid_scale_factor*math.pi/6])
        theta_pid_input['PS'] = fuzz.trimf(theta_pid_input.universe, [0, pid_scale_factor*math.pi/6, pid_scale_factor*math.pi/3])
        theta_pid_input['PL'] = fuzz.smf(theta_pid_input.universe, pid_scale_factor*math.pi/6, pid_scale_factor*math.pi/3)

        # Declare fuzzy sets for the ship_turn consequent; this will be returned as turn_rate.
        ship_turn['NL'] = fuzz.trimf(ship_turn.universe, [-180,-180,-45])
        ship_turn['NS'] = fuzz.trimf(ship_turn.universe, [-90,-45,0])
        ship_turn['Z'] = fuzz.trimf(ship_turn.universe, [-45,0,45])
        ship_turn['PS'] = fuzz.trimf(ship_turn.universe, [0,45,90])
        ship_turn['PL'] = fuzz.trimf(ship_turn.universe, [45,180,180])
        
        #Declare singleton fuzzy sets for the ship_fire consequent; -1 -> don't fire, +1 -> fire; this will be thresholded
        #   and returned as the boolean 'fire'
        ship_fire['N'] = fuzz.trimf(ship_fire.universe, [-1,-1,0.0])
        ship_fire['Y'] = fuzz.trimf(ship_fire.universe, [0.0,1,1]) 
        
        ship_thrust['NL'] = fuzz.trimf(ship_thrust.universe, [-480, -480, -337.5])
        ship_thrust['NS'] = fuzz.trimf(ship_thrust.universe, [-337.5, -225, -112.5])
        ship_thrust['Z'] = fuzz.trimf(ship_thrust.universe, [-112.5, 0, 112.5])
        ship_thrust['PS'] = fuzz.trimf(ship_thrust.universe, [112.5, 225, 337.5])
        ship_thrust['PL'] = fuzz.trimf(ship_thrust.universe, [337.5, 480, 480])
        
        current_ship_thrust['NL'] = fuzz.trimf(ship_thrust.universe, [-480, -480, -337.5])
        current_ship_thrust['NS'] = fuzz.trimf(ship_thrust.universe, [-337.5, -225, -112.5])
        current_ship_thrust['Z'] = fuzz.trimf(ship_thrust.universe, [-112.5, 0, 112.5])
        current_ship_thrust['PS'] = fuzz.trimf(ship_thrust.universe, [112.5, 225, 337.5])
        current_ship_thrust['PL'] = fuzz.trimf(ship_thrust.universe, [337.5, 480, 480])
        
        ship_speed['NL'] = fuzz.trimf(ship_speed.universe, [-240, -160, -100])
        ship_speed['NS'] = fuzz.trimf(ship_speed.universe, [-100,-60, -10])
        ship_speed['Z'] = fuzz.trimf(ship_speed.universe, [-10, 5, 10])
        ship_speed['PS'] = fuzz.trimf(ship_speed.universe, [10, 60, 100])
        ship_speed['PL'] = fuzz.trimf(ship_speed.universe, [100, 160, 240])
        
        buffer = 70
        min_x = 0
        min_y = 0
        max_x = game_state['map_size'][0]
        max_y = game_state['map_size'][1]
        
        ship_pos_x['close_to_left'] = fuzz.trimf(ship_pos_x.universe, [min_x, min_x + buffer, min_x + buffer])
        ship_pos_x['close_to_right'] = fuzz.trimf(ship_pos_y.universe, [max_x - buffer, max_x - buffer, max_x])

        # Define membership functions for y
        ship_pos_y['close_to_top'] = fuzz.trimf( ship_pos_y.universe, [max_y - buffer * 2, max_y - buffer, max_y])
        ship_pos_y['close_to_bottom'] = fuzz.trimf( ship_pos_y.universe, [min_y, min_y + buffer, min_y + buffer * 2])
        
        position_control_rules = [
            # CURRENTLY UNUSED
            ctrl.Rule(ship_pos_x['close_to_left'] & theta_delta['NL'], (ship_fire['N'], ship_thrust['PL'])),
            ctrl.Rule(ship_pos_x['close_to_left'] & theta_delta['NS'], (ship_fire['N'], ship_thrust['PL'])),
            ctrl.Rule(ship_pos_x['close_to_left'] & theta_delta['Z'], (ship_fire['N'], ship_thrust['PL'])),
            ctrl.Rule(ship_pos_x['close_to_left'] & theta_delta['PL'], (ship_fire['N'], ship_thrust['PL'])),
            ctrl.Rule(ship_pos_x['close_to_left'] & theta_delta['PS'], (ship_fire['N'], ship_thrust['PL'])),
        ]

        # Declare each fuzzy rule
        defensive_camper_rules = [
            # PID controller to determine turn rate
            # ship_turn = P + I + D = theta_delta + theta_delta_integral + theta_delta_derivative
            ctrl.Rule(theta_pid_input['NL'], ship_turn['NL']),
            ctrl.Rule(theta_pid_input['NS'], ship_turn['NS']),
            ctrl.Rule(theta_pid_input['Z'], ship_turn['Z']),
            ctrl.Rule(theta_pid_input['PS'], ship_turn['PS']),
            ctrl.Rule(theta_pid_input['PL'], ship_turn['PL']),

            # Only fire if we're pretty much aimed at the asteroid
            ctrl.Rule(theta_delta['NL'] | theta_delta['PL'], ship_fire['N']),
            ctrl.Rule(theta_delta['Z'], ship_fire['Y']),

            # If we're far from the asteroid, thrust toward it. If we're super close, stop going closer and even back up a bit
            ctrl.Rule(bullet_time['L'], ship_thrust['PL']),
            ctrl.Rule(bullet_time['M'], ship_thrust['PS']),
            ctrl.Rule(bullet_time['S'], ship_thrust['NS']),
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
            
            ctrl.Rule(current_ship_thrust['PL'] & (ship_speed['PL'] | ship_speed['PS'] | ship_speed['Z'] | ship_speed['NL'] | ship_speed['NS']), (ship_thrust['PL'])), #slow down
            ctrl.Rule(current_ship_thrust['PS'] & (ship_speed['PL'] | ship_speed['PS'] | ship_speed['Z'] | ship_speed['NL'] | ship_speed['NS']), (ship_thrust['PS'])), #slow down
            ctrl.Rule(current_ship_thrust['Z'] & (ship_speed['PL'] | ship_speed['PS'] | ship_speed['Z'] | ship_speed['NL'] | ship_speed['NS']), (ship_thrust['Z'])), #slow down
            ctrl.Rule(current_ship_thrust['NS'] & (ship_speed['PL'] | ship_speed['PS'] | ship_speed['Z'] | ship_speed['NL'] | ship_speed['NS']), (ship_thrust['NS'])), #slow down
            ctrl.Rule(current_ship_thrust['NL'] & (ship_speed['PL'] | ship_speed['PS'] | ship_speed['Z'] | ship_speed['NL'] | ship_speed['NS']), (ship_thrust['NL'])) #slow down
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
        
        self.position_control = ctrl.ControlSystem()
        for i in position_control_rules:
            self.position_control.addrule(i)
         
        self.targeting_control = ctrl.ControlSystem()
        for i in defensive_camper_rules:
            self.targeting_control.addrule(i)
            
        self.thrust_control = ctrl.ControlSystem()
        for i in thrust_control_rules:
            self.thrust_control.addrule(i)
    

    def actions(self, ship_state: Dict, game_state: Dict) -> Tuple[float, float, bool]:
        if not self.init_done:
            self.finish_init(game_state)
            self.init_done = True
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
                    #if dx == 0 and dy == 0:
                    #    continue  # Skip the original asteroid position
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
        
        map_size_x = game_state['map_size'][0]
        map_size_y = game_state['map_size'][1]
        #print(map_size_x, map_size_y)
        
        #if self.eval_frames % 30 == 0 or self.previously_targetted_asteroid is None:
        closest_asteroid = find_closest_asteroid(game_state, ship_state, self.shot_at_asteroids)
        #else:
        #    closest_asteroid = self.previously_targetted_asteroid
        #if self.previously_targetted_asteroid is None or closest_asteroid['aster']['velocity'] != self.previously_targetted_asteroid['aster']['velocity']:
        #    # We're targetting a new asteroid. Reset the PID terms
        #    #print("Targetting new asteroid!")
        #    #print(closest_asteroid)
        #    self.pid_integral = 0
        #    self.pid_previous_error = 0
        #    self.previously_targetted_asteroid = closest_asteroid
        # closest_asteroid now contains the nearest asteroid considering wraparound

        # closest_asteroid is now the nearest asteroid object. 
        # Calculate intercept time given ship & asteroid position, asteroid velocity vector, bullet speed (not direction).
        # Based on Law of Cosines calculation, see notes.
        
        # Side D of the triangle is given by closest_asteroid.dist. Need to get the asteroid-ship direction
        #    and the angle of the asteroid's current movement.
        # REMEMBER TRIG FUNCTIONS ARE ALL IN RADIANS!!!
        #print("We're targetting:")
        #print(closest_asteroid)
        bullet_t, shooting_theta, _, _ = calculate_interception(ship_pos_x, ship_pos_y, closest_asteroid["position"][0] + 2 * time_delta * closest_asteroid["velocity"][0], closest_asteroid["position"][1] + 2 * time_delta * closest_asteroid["velocity"][1], closest_asteroid["velocity"][0], closest_asteroid["velocity"][1], ship_state["heading"])
        #print(f"Shooting theta: {shooting_theta}, bullet t: {bullet_t}")
        # position Controller to stay near center
        
        
        # Pass the inputs to the rulebase and fire it
        shooting = ctrl.ControlSystemSimulation(self.targeting_control,flush_after_run=1)
        
        # PID stuff
        # If it overshoots a lot and oscillates, either the integral gain (Ki) needs to be increased or all gains (Kp,Ki,Kd) should be reduced
        # Too much overshoot? Increase Kd, decrease Kp.
        # Response too damped? Increase Kp.
        # Ramps up quickly to a value below target value and then slows down as it approaches target value? Try increasing the Ki constant.
        Kp = 3.0
        Ki = .3
        Kd = 1.2
        #Ki_abs_cap = 0.5
        leaky_factor = 0.96

        def log_decay_num(x):
            # This one has the right idea of making super large values smaller, but it doesn't give enough weight to super small stuff
            eps = 0.00001
            if x < 0:
                sign = -1.0
            elif x >= 0:
                sign = 1.0
            if abs(x) < eps:
                return 0
            return sign * math.log1p(abs(x))

        def sqrt_decay_num(x):
            # This one enhances small values and weighs them more, so I like this one
            eps = 0.00001
            if x < 0:
                sign = -1.0
            elif x >= 0:
                sign = 1.0
            if abs(x) < eps:
                return 0
            #print(f"{x}, {abs(x)}")
            return sign * math.sqrt(abs(x))

        shooting.input['bullet_time'] = bullet_t
        P = shooting_theta
        shooting.input['theta_delta'] = shooting_theta
        self.pid_integral = self.pid_integral * leaky_factor + sqrt_decay_num(shooting_theta)
        #if self.pid_integral > Ki_abs_cap:
        #    self.pid_integral = Ki_abs_cap
        #elif self.pid_integral < -Ki_abs_cap:
        #    self.pid_integral = -Ki_abs_cap
        I = self.pid_integral
        D = shooting_theta - self.pid_previous_error
        self.pid_previous_error = shooting_theta
        #shooting.input['theta_pid_input'] = Kp*P + Ki*I + Kd*D
        #print(f"P: {Kp*P} I: {Ki*I} D: {Kd*D}, input: {Kp*P + Ki*I + Kd*D}, theta_delta: {shooting_theta}")
        #shooting.compute()
        
        # Get the defuzzified outputs
        #turn_rate = shooting.output['ship_turn']
        turn_rate_range = 180.0
        eps = 0.00001

        shooting_theta_deg = shooting_theta * 180.0 / math.pi
        #print(f"shooting theta deg {shooting_theta_deg}")
        if shooting_theta_deg < 0:
            sign = -1
        else:
            sign = +1
        shooting_theta_deg = abs(shooting_theta_deg)
        if shooting_theta_deg > turn_rate_range * time_delta:
            turn_rate = sign * turn_rate_range
            #print(f'Turn rate is max at: {turn_rate}')
        else:
            self.fire_on_frames.add(self.eval_frames + 1)
            turn_rate = sign * shooting_theta_deg / time_delta
            #print(f'SNAP! Turn rate: {turn_rate}')

        if self.eval_frames in self.fire_on_frames and not ship_state['is_respawning']:
            if self.eval_frames - self.last_time_fired >= 5:
                # Our firing cooldown has ran out, and we can fire. We can only shoot once every 5 frames.
                self.last_time_fired = self.eval_frames
                fire = True
                self.fire_on_frames.remove(self.eval_frames)
                if (closest_asteroid["velocity"][0], closest_asteroid["velocity"][1], closest_asteroid["radius"]) not in self.shot_at_asteroids:
                    self.shot_at_asteroids[(closest_asteroid["velocity"][0], closest_asteroid["velocity"][1], closest_asteroid["radius"])] = math.ceil(bullet_t / time_delta)
            else:
                fire = False
                # Try to fire on the next frame
                self.fire_on_frames.add(self.eval_frames + 1)
        elif self.eval_frames in self.fire_on_frames and ship_state['is_respawning']:
            self.fire_on_frames.add(self.eval_frames + 1)
            fire = False
        else:
            fire = False
        #fire = self.eval_frames % 5 == 0
        # List to hold keys to be removed
        keys_to_remove = []
        #print(len(self.shot_at_asteroids))
        #print(self.eval_frames)
        #print(self.shot_at_asteroids)
        # Iterate through the dictionary items
        for key, value in self.shot_at_asteroids.items():
            # Decrease each value by 1
            self.shot_at_asteroids[key] = value - 1
            
            # If the value hits 0, add the key to the list of keys to be removed
            if self.shot_at_asteroids[key] <= 0:
                keys_to_remove.append(key)

        # Remove the keys from the dictionary
        for key in keys_to_remove:
            del self.shot_at_asteroids[key]
        #turn_rate = 0
        #print(f"Turn rate: {turn_rate}")
        #thrust = shooting.output['ship_thrust']
        thrust = 0
        #if shooting.output['ship_fire'] >= 0 and not ship_state['is_respawning']:
        #    fire = True
        #else:
        #    fire = False
        
        # And return your three outputs to the game simulation. Controller algorithm complete.
        
        # position = ctrl.ControlSystemSimulation(self.position_conrol,flush_after_run=1)
        # if(ship_pos_x >= 730 or ship_pos_x <= 70):
        #     position.input['ship_pos_x'] = ship_pos_x
        #     # position.input['ship_pos_y'] = ship_pos_y
        #     position.input['theta_delta'] = shooting_theta
        
        #     position.compute()
        #     # turn_rate = position.output['ship_turn']
        #     thrust = position.output['ship_thrust']
        
        # this controller will look at out current speed and thrust and adjust so we dont uncontrollably runaway
        thrust_controller = ctrl.ControlSystemSimulation(self.thrust_control,flush_after_run=1)
        
        thrust_controller.input['ship_speed'] = ship_state['speed']
        thrust_controller.input['current_ship_thrust'] = thrust     
        thrust_controller.compute()   
        
        #thrust = thrust_controller.output['ship_thrust']
        thrust = 0
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
