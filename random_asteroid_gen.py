import random

def generate_asteroids(num_asteroids, position_range_x, position_range_y, speed_range, angle_range, size_range):
    """
    Generate a list of asteroid states with random attributes.

    :param num_asteroids: Number of asteroids to generate
    :param position_range: Tuple of (min, max) for position (applied to both x and y)
    :param speed_range: Tuple of (min, max) for speed
    :param angle_range: Tuple of (min, max) for angle
    :param size_range: Tuple of (min, max) for size
    :return: String representation of the list of asteroids
    """
    asteroids = []
    for _ in range(num_asteroids):
        position = (random.randint(*position_range_x), random.randint(*position_range_y))
        speed = random.randint(*speed_range)
        angle = random.randint(*angle_range)
        size = random.randint(*size_range)
        asteroids.append({'position': position, 'speed': speed, 'angle': angle, 'size': size})

    return str(asteroids)

# Example usage
asteroid_string = generate_asteroids(
    num_asteroids=50,
    position_range_x=(0, 3400),
    position_range_y=(0, 1700),
    speed_range=(1, 750),
    angle_range=(-180, 180),
    size_range=(1, 3)
)
print(asteroid_string)
