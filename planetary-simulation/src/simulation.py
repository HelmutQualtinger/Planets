from vpython import sphere, vector, rate, color

class Planet:
    def __init__(self, name, mass, radius, position, velocity):
        self.name = name
        self.mass = mass
        self.radius = radius
        self.position = vector(*position)
        self.velocity = vector(*velocity)
        self.sphere = sphere(pos=self.position, radius=self.radius, color=color.white)

    def update_position(self, time_step):
        self.position += self.velocity * time_step
        self.sphere.pos = self.position

def calculate_gravitational_force(planet1, planet2):
    G = 6.67430e-11  # gravitational constant
    r_vector = planet2.position - planet1.position
    distance = r_vector.mag
    force_magnitude = G * (planet1.mass * planet2.mass) / (distance ** 2)
    force_direction = r_vector.norm()
    return force_direction * force_magnitude

def update_velocities(planets, time_step):
    forces = {planet: vector(0, 0, 0) for planet in planets}
    
    for i in range(len(planets)):
        for j in range(i + 1, len(planets)):
            force = calculate_gravitational_force(planets[i], planets[j])
            forces[planets[i]] += force
            forces[planets[j]] -= force

    for planet in planets:
        acceleration = forces[planet] / planet.mass
        planet.velocity += acceleration * time_step

def simulate(planets, time_step, total_time):
    for t in range(int(total_time / time_step)):
        rate(100)  # control the simulation speed
        update_velocities(planets, time_step)
        for planet in planets:
            planet.update_position(time_step)