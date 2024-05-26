import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Set the dimensions of the map
height = 3000
width = 6000

# Set the downscale factor for visualization
scale = 10

# Set variables for the potential field planning algorithm
goal_threshold = 100
obstacle_threshold = 200
obstacle_charge = 50.0
step_size = 20
robot_charge = 5.0
goal_charge = 5.0

# Calculate the downscaled dimensions
downscaled_width = width // scale
downscaled_height = height // scale

# Create a figure with the downscaled size
fig = plt.figure(figsize=(downscaled_width/100, downscaled_height/100), dpi=100)

# Add a subplot to the figure
ax = fig.add_subplot(111, facecolor='black')

# Set the aspect of the plot to be equal
ax.set_aspect('equal')

# Set the x-axis and y-axis limits based on the actual dimensions
ax.set_xlim(0, width)
ax.set_ylim(0, height)

# Set the x-axis and y-axis ticks based on the actual dimensions
ax.set_xticks(range(0, width+1, 1000))
ax.set_yticks(range(0, height+1, 500))

# Set the title of the figure
fig.suptitle(f'Path found after scanning')

# Define the coordinates and radius*2s of the obstacles
c1 = {'x': 1120, 'y': 2425, 'radius*2': 800}
c2 = {'x': 2630, 'y': 900, 'radius*2': 1400}
c3 = {'x': 4450, 'y': 2200, 'radius*2': 750}

# Draw circles on the plot
circle1_radius = c1['radius*2'] / 2
circle1_plot = plt.Circle((c1['x'], c1['y']), circle1_radius, color='red')
ax.add_patch(circle1_plot)

circle2_radius = c2['radius*2'] / 2
circle2_plot = plt.Circle((c2['x'], c2['y']), circle2_radius, color='red')
ax.add_patch(circle2_plot)

circle3_radius = c3['radius*2'] / 2
circle3_plot = plt.Circle((c3['x'], c3['y']), circle3_radius, color='red')
ax.add_patch(circle3_plot)

# Define the coordinates of the start and goal points
start_point = {'x': 0, 'y': 1500}
goal_point = {'x': 6000, 'y': 1500}

# Plot goal pose
goal_point_plot = plt.plot(goal_point['x'], goal_point['y'], 'rD', markersize=5)

# Plot start pose
start_point_plot = plt.plot(start_point['x'], start_point['y'], 'rD', markersize=5)

# Function calculating the repulsive force from an obstacle
def repulsive_force(robot_pos, obstacle_pos, obstacle_radius):
    direction = robot_pos - obstacle_pos
    distance = np.linalg.norm(direction)
    if distance <= obstacle_radius + obstacle_threshold:
        if distance <= obstacle_radius:
            distance = obstacle_radius
        force = obstacle_charge * robot_charge / (distance ** 2)
        return force * direction / distance
    else:
        return np.zeros(2)

# Function calculating the attractive force to the goal
def attractive_force(robot_pos, goal_pos):
    direction = goal_pos - robot_pos
    distance = np.linalg.norm(direction)
    if distance <= goal_threshold:
        return np.zeros(2)
    force = goal_charge * robot_charge / (distance ** 2)
    return force * direction / distance

# Potential field planning algorithm
robot_pos = np.array([start_point['x'], start_point['y']], dtype=float)
path = [robot_pos]
exploration_nodes = []

max_counts = 1000
count = 0

while np.linalg.norm(robot_pos - np.array([goal_point['x'], goal_point['y']])) > goal_threshold and count < max_counts:

    # Calculate repulsive forces from the obstacles
    repulsive_force_vec = np.zeros(2)
    for circle in [c1, c2, c3]:
        repulsive_force_vec += repulsive_force(robot_pos, np.array([circle['x'], circle['y']]), circle['radius*2'] / 2)

    # Calculate attractive force from the goal
    attractive_force_vec = attractive_force(robot_pos, np.array([goal_point['x'], goal_point['y']]))
    
    # Calculate the resultant force
    resultant_force = attractive_force_vec + repulsive_force_vec
    
    # Normalize the resultant force
    if np.linalg.norm(resultant_force) > 0:
        resultant_force /= np.linalg.norm(resultant_force)
    
    # Calculate the new robot position
    new_robot_pos = robot_pos + step_size * resultant_force
    print(new_robot_pos)
    path.append(new_robot_pos)
    exploration_nodes.append(new_robot_pos)
    
    # Update the robot position
    robot_pos = new_robot_pos
    count += 1

# Create a line object for the animation
line, = ax.plot([], [], 'y-', linewidth=2)

# Function to update the animation
def animate(i):
    x, y = zip(*path[:i+1])
    line.set_data(x, y)
    return line,

# Create the animation
ani = FuncAnimation(fig, animate, frames=len(path), interval=50, blit=True, repeat=False)

# Display the animation
plt.show()