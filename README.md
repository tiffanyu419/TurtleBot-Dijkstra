# Solving Mazes using Dijkstra's Alogrithm - TurtleBot
By Tiffany Yu and Emily Cai

Using ROS and a turtlebot, we implemented Dijkstra's algorithm in the problem of
solving different mazes. Starting code was provided by Professor Matt Zucker. 

#### maze.py
This codes a simple module to store and query 2D mazes with grid-aligned walls. Also contains the
implementation of the Dijkstra's algorithm under the function find_path. The function path_to_command
translates the results from the algorithm into specific commands for the TurtleBot

#### main.py
Code that connects the algorithm to the Turtlebot. This code contains the specifics of the movement
of the robot. Calls on maze.py to find the solution and executes it. 
