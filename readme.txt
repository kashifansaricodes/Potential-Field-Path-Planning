# Potential Field Path Planning Algorithm

This project demonstrates a potential field path planning algorithm using Python, NumPy, and Matplotlib. The algorithm navigates a robot from a start point to a goal point while avoiding obstacles in a 2D environment.

## Requirements

- Python 3.x
- NumPy
- Matplotlib

## Setup and Installation

1. Install Python 3.x from [python.org](https://www.python.org/).
2. Install the required Python libraries using pip:
   ```sh
   pip install numpy matplotlib
-------------------------------------------------------------------------------

# Code Overview
The code is divided into several sections:

## Environment Setup:

Set the dimensions of the map (height and width).
Define the downscale factor for visualization.
Set variables for the potential field algorithm (goal threshold, obstacle threshold, obstacle charge, step size, robot charge, goal charge).

## Visualization Setup:

Calculate the downscaled dimensions.
Create a figure and subplot for the visualization.
Define the coordinates and radii of the obstacles.
Plot the start and goal points.
Force Calculations:

repulsive_force: Calculates the repulsive force exerted by obstacles on the robot.
attractive_force: Calculates the attractive force exerted by the goal on the robot.

## Path Planning Algorithm:

Initialize the robot's position at the start point.
Iterate until the robot reaches the goal or the maximum number of iterations is reached.
Calculate the resultant force (sum of repulsive and attractive forces).
Update the robot's position based on the resultant force.
Append the new position to the path.

## Animation:

Create an animation of the robot's path using Matplotlib's FuncAnimation.

--------------------------------------------------------------------------------

## Customization
You can customize the following parameters to change the environment and behavior of the algorithm:

Map dimensions (height, width).
Obstacles' coordinates and sizes (c1, c2, c3).
Algorithm parameters (goal_threshold, obstacle_threshold, obstacle_charge, step_size, robot_charge, goal_charge).


--------------------------------------------------------------------------------