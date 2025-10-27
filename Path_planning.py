from matplotlib.colors import ListedColormap
from heapq import heappush, heappop

import numpy as np
import matplotlib.pyplot as plt
import math

def heuristic(a, b):
    """ Implement the Manhattan distance heuristi c"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(map_grid, start, goal):
    """ A* algorith to determine the optimal path to follow based on the grid, the robot and the goal positions """
    
    # Initialize the open set as a priority queue and add the start node
    open_set = [] #  set of nodes to be explored, prioritized by their estimated total cost
    heappush(open_set, (heuristic(start, goal), 0, start))  # (f_cost, g_cost, position)

    # Initialize the came_from dictionary
    came_from = {}
    # Initialize g_costs dictionary with default value of infinity and set g_costs[start] = 0
    g_costs = {start: 0}
    # Initialize the explored set
    explored = set()
    operation_count = 0

    while open_set:
        # Pop the node with the lowest f_cost from the open set
        current_f_cost, current_g_cost, current_pos = heappop(open_set)

        # Add the current node to the explored set
        explored.add(current_pos)

        # For directly reconstruct path
        if current_pos == goal:
            break

        # Get the neighbors of the current node (up, down, left, right and )
        neighbors = [
            (current_pos[0] - 1, current_pos[1]),  # Up
            (current_pos[0] + 1, current_pos[1]),  # Down
            (current_pos[0], current_pos[1] - 1),  # Left
            (current_pos[0], current_pos[1] + 1),  # Right
            (current_pos[0] - 1, current_pos[1] - 1),  # Up-Left
            (current_pos[0] - 1, current_pos[1] + 1),  # Up-Right
            (current_pos[0] + 1, current_pos[1] - 1),  # Down-Left
            (current_pos[0] + 1, current_pos[1] + 1)   # Down-Right
        ]

        for neighbor in neighbors:
            # Check if neighbor is within bounds and not an obstacle
            if (0 <= neighbor[0] < map_grid.shape[0]) and (0 <= neighbor[1] < map_grid.shape[1]):
                if map_grid[neighbor[0], neighbor[1]] != -1 and neighbor not in explored:
                    # Calculate tentative_g_cost
                    tentative_g_cost = current_g_cost + 1 + map_grid[neighbor[0], neighbor[1]] # map_grid[neighbor[0], neighbor[1]] for weighted cost

                    # If this path to neighbor is better than any previous one
                    if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                        # Update came_from, g_costs, and f_cost
                        came_from[neighbor] = current_pos
                        g_costs[neighbor] = tentative_g_cost
                        f_cost = tentative_g_cost + heuristic(neighbor, goal)

                        # Add neighbor to open set
                        heappush(open_set, (f_cost, tentative_g_cost, neighbor))
                        operation_count += 1

    # Reconstruct path
    if current_pos == goal:
        path = []
        while current_pos in came_from:
            path.append(current_pos)
            current_pos = came_from[current_pos]
        path.append(start)
        return path[::-1], explored,operation_count
    else:
        # If we reach here, no path was found
        return None, explored,operation_count

def simplify_path(path):
    """ Symplify a path given to keep only the changes in direction """
    
    if len(path) < 2:
        return path

    # First point
    simplified_path = [path[0]]

    # Segments verification
    for i in range(1, len(path) - 1):
        prev = path[i - 1]
        curr = path[i]
        next = path[i + 1]

        # Vectors between consecutive points
        vec1 = (curr[0] - prev[0], curr[1] - prev[1])
        vec2 = (next[0] - curr[0], next[1] - curr[1])

        # Verification of colinearity (vector product = 0)
        if vec1[0] * vec2[1] != vec1[1] * vec2[0]:
            simplified_path.append(curr)

    # Last point
    simplified_path.append(path[-1])
    return simplified_path