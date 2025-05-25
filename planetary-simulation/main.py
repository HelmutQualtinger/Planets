from vpython import *
import math # Import math für Winkelfunktionen

# Gravitational constant
G = 6.674e-11 # m^3 kg^-1 s^-2

# Create the scene
scene = canvas(title='Planetary Simulation', width=1200, height=800, background=color.black)

# Define a consistent radius scaling factor for planets
PLANET_RADIUS_SCALE = 3000 # Passen Sie diesen Wert an, um die visuelle Größe der Planeten zu ändern

# Define a tilt angle for the orbital planes (in degrees)
ORBITAL_TILT_DEGREES = 5 # Beispiel: 5 Grad Neigung
ORBITAL_TILT_RADIANS = math.radians(ORBITAL_TILT_DEGREES)

# Define a factor to make orbits more elliptical (e.g., < 1 for initial position near aphelion)
ELLIPTICITY_FACTOR = 0.6 # Passen Sie diesen Wert an, um die Elliptizität zu ändern (0.8 für deutlich elliptisch)

# Create the Sun
# Keep sun radius large for visibility as the central body
sun = sphere(pos=vector(0, 0, 0), radius=1e10, color=color.yellow, emissive=True, mass=1.989e30) # Example mass (Sun), increased radius for visibility
sun.velocity = vector(0, 0, 0) # Initialize sun's velocity

# Function to apply tilt to a velocity vector (rotation around x-axis)
def apply_tilt(velocity_vector, tilt_angle_rad):
    vy = velocity_vector.y
    vz = velocity_vector.z
    new_vy = vy * math.cos(tilt_angle_rad) - vz * math.sin(tilt_angle_rad)
    new_vz = vy * math.sin(tilt_angle_rad) + vz * math.cos(tilt_angle_rad)
    return vector(velocity_vector.x, new_vy, new_vz)

# Create the planets
mercury_vel_xy = vector(0, 47.8e3 * ELLIPTICITY_FACTOR, 0) # Apply ellipticity factor
mercury = sphere(pos=vector(58e9, 0, 0), radius=2.44e6 * PLANET_RADIUS_SCALE, color=color.gray(0.8), make_trail=True, retain=27, mass=0.330e24)
mercury.velocity = apply_tilt(mercury_vel_xy, ORBITAL_TILT_RADIANS)

venus_vel_xy = vector(0, 35.0e3 * ELLIPTICITY_FACTOR, 0) # Apply ellipticity factor
venus = sphere(pos=vector(108e9, 0, 0), radius=6.052e6 * PLANET_RADIUS_SCALE, color=color.orange, make_trail=True, retain=27, mass=4.87e24)
venus.velocity = apply_tilt(venus_vel_xy, ORBITAL_TILT_RADIANS)

earth_vel_xy = vector(0, 30e3 * ELLIPTICITY_FACTOR, 0) # Apply ellipticity factor
earth = sphere(pos=vector(150e9, 0, 0), radius=6.371e6 * PLANET_RADIUS_SCALE, color=color.blue, make_trail=True, retain=27, mass=5.972e24) # Example mass (Earth), scaled radius
# Set initial velocity for Earth (approximate orbital velocity)
earth.velocity = apply_tilt(earth_vel_xy, ORBITAL_TILT_RADIANS)

mars_vel_xy = vector(0, 24.1e3 * ELLIPTICITY_FACTOR, 0) # Apply ellipticity factor
mars = sphere(pos=vector(228e9, 0, 0), radius=3.389e6 * PLANET_RADIUS_SCALE, color=color.red, make_trail=True, retain=27, mass=0.642e24)
mars.velocity = apply_tilt(mars_vel_xy, ORBITAL_TILT_RADIANS)

jupiter_vel_xy = vector(0, 13.1e3 * ELLIPTICITY_FACTOR, 0) # Apply ellipticity factor
jupiter = sphere(pos=vector(778e9, 0, 0), radius=69.911e6 * PLANET_RADIUS_SCALE/5, color=color.orange, make_trail=True, retain=1080, mass=1898e24) # Erhöhen Sie retain für Jupiter (ca. 15 Monate)
jupiter.velocity = apply_tilt(jupiter_vel_xy, ORBITAL_TILT_RADIANS)

saturn_vel_xy = vector(0, 9.7e3 * ELLIPTICITY_FACTOR, 0) # Apply ellipticity factor
saturn = sphere(pos=vector(1427e9, 0, 0), radius=58.232e6 * PLANET_RADIUS_SCALE/5, color=color.yellow, make_trail=True, retain=1080, mass=568e24) # Erhöhen Sie retain für Saturn (ca. 15 Monate)
saturn.velocity = apply_tilt(saturn_vel_xy, ORBITAL_TILT_RADIANS)
# Add a ring for Saturn (simplified)
# The ring radius and thickness should be relative to the *scaled* planet radius.
# Change the axis to match the tilted orbital plane (rotated around x-axis)
tilted_ring_axis = vector(0, -math.sin(ORBITAL_TILT_RADIANS), math.cos(ORBITAL_TILT_RADIANS))
saturn_ring = ring(pos=saturn.pos, axis=tilted_ring_axis, radius=saturn.radius * 1.5, thickness=saturn.radius * 0.2, color=color.gray(0.5))
# Make the ring follow Saturn
saturn.ring = saturn_ring # Attach ring object to planet object
# Saturn's ring doesn't have a trail itself, so no retain needed here

uranus_vel_xy = vector(0, 6.8e3 * ELLIPTICITY_FACTOR, 0) # Apply ellipticity factor
uranus = sphere(pos=vector(2871e9, 0, 0), radius=25.362e6 * PLANET_RADIUS_SCALE/5, color=color.cyan, make_trail=True, retain=1080, mass=86.8e24) # Erhöhen Sie retain für Uranus (ca. 6,25 Jahre)
uranus.velocity = apply_tilt(uranus_vel_xy, ORBITAL_TILT_RADIANS)

neptune_vel_xy = vector(0, 5.4e3 * ELLIPTICITY_FACTOR, 0) # Apply ellipticity factor
neptune = sphere(pos=vector(4498e9, 0, 0), radius=24.622e6 * PLANET_RADIUS_SCALE, color=color.blue, make_trail=True, retain=1080, mass=102e24) # Erhöhen Sie retain für Neptun (ca. 6,25 Jahre)
neptune.velocity = apply_tilt(neptune_vel_xy, ORBITAL_TILT_RADIANS)

# Create the Moon
# Moon's position is relative to Earth, so its initial position is Earth's pos + offset
# Calculate Earth's tilted velocity first
earth_initial_velocity_tilted = apply_tilt(earth_vel_xy, ORBITAL_TILT_RADIANS) # Use the scaled Earth velocity
moon = sphere(pos=earth.pos + vector(384.4e6, 0, 0), radius=1.737e6 * PLANET_RADIUS_SCALE, color=color.white, make_trail=True, retain=108, mass=7.346e22) # Example mass (Moon), scaled radius
# Set initial velocity for Moon (relative to Earth's velocity + orbital velocity around Earth)
# This initial velocity is simplified; a realistic setup is more complex
# Apply tilt to the Moon's relative velocity around Earth as well
moon_relative_vel_xy = vector(0, 1.022e3 * ELLIPTICITY_FACTOR, 0) # Apply ellipticity factor to relative velocity
moon.velocity = earth_initial_velocity_tilted + apply_tilt(moon_relative_vel_xy, ORBITAL_TILT_RADIANS) # m/s


# Set time step (using a larger dt for faster simulation, but less accuracy)
dt = 20 * 3600 # 20 hours in seconds

# Create a list of all bodies to simulate
bodies = [sun, mercury, venus, earth, mars, jupiter, saturn, uranus, neptune, moon] # Include all bodies

# Animation loop
while True:
    rate(100) # Limit the loop speed

    # Calculate gravitational forces for all pairs of bodies (N-body simulation)
    forces = {}
    for body in bodies:
        forces[body] = vector(0, 0, 0) # Initialize net force to zero

    for i in range(len(bodies)):
        body1 = bodies[i]
        for j in range(i + 1, len(bodies)): # Iterate through unique pairs
            body2 = bodies[j]

            r_vec = body2.pos - body1.pos # Use the sphere's position for force calculation (these are the real positions)
            mag_r = mag(r_vec)

            # Avoid division by zero or extremely large forces at very small distances
            if mag_r > 1e8: # Use a threshold to avoid issues with close objects
                F_mag = G * body1.mass * body2.mass / mag_r**2
                F_vec = F_mag * norm(r_vec)

                forces[body1] += F_vec # Force on body1 due to body2
                forces[body2] -= F_vec # Force on body2 due to body1 (Newton's 3rd law)

    # Update velocities and positions for all bodies
    for body in bodies:
        net_force = forces[body]
        # Update velocity (a = F/m)
        body.velocity += (net_force / body.mass) * dt
        # Update position
        body.pos += body.velocity * dt # Update the sphere's position directly
        # Update Saturn's ring position if it exists
        if hasattr(body, 'ring'):
             body.ring.pos = body.pos # Ring follows the sphere's position

