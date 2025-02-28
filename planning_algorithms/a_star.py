import heapq
import numpy as np
import matplotlib.pyplot as plt
import time

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent

        self.g = float('inf')
        self.h = 0
        self.f = float('inf')

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def print_grid(grid):
    for row in grid:
        print(" ".join(row))
    print()

def a_star(grid, start, goal):
    start_node = Node(start)
    start_node.g = 0
    start_node.h = heuristic(start, goal)
    start_node.f = start_node.g + start_node.h

    open_list = []
    closed_set = set()

    heapq.heappush(open_list, start_node)

    moves = [(-1, 0), (1, 0), (0, 1), (0, -1), (-1, -1), (1, 1), (1, -1), (-1, 1)]

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.position == goal:
            path = []

            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]
        
        closed_set.add(current_node.position)

        for move in moves:

            new_pos = (current_node.position[0] + move[0], current_node.position[1] + move[1])

            if (0 <= new_pos[0] < grid.shape[0] and
                0 <= new_pos[1] < grid.shape[1] and
                grid[new_pos] == 0 and
                new_pos not in closed_set):

                new_node = Node(new_pos, current_node)
                new_node.g = current_node.g + (1.41 if move[0] != 0 and move[1] != 0 else 1)  
                new_node.h = heuristic(new_pos, goal)
                new_node.f = new_node.g + new_node.h

                heapq.heappush(open_list, new_node)    

    return None

def plot_grid(grid, path, start, goal):
    plt.figure(figsize=(6, 6))
    plt.imshow(grid, cmap="gray_r") 
    
    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_y, path_x, marker="o", color="red", markersize=5, linewidth=2, label="Path")
    
    plt.scatter(start[1], start[0], color="green", s=100, label="Start")
    plt.scatter(goal[1], goal[0], color="blue", s=100, label="Goal")

    plt.xticks(range(grid.shape[1]))
    plt.yticks(range(grid.shape[0]))
    plt.grid(True, which="both", linestyle="--", color="gray", linewidth=0.5)
    plt.gca().invert_yaxis()
    plt.legend()
    plt.show()

if __name__ == "__main__":
    grid = np.zeros((10, 10), dtype=int)
    grid[3, 2:5] = 1 #obstacles
    grid[6, 3:7] = 1
    start = (1, 1)
    goal = (9, 9)

    start_time = time.perf_counter()
    path = a_star(grid, start, goal)
    end_time = time.perf_counter()
    execution_time = end_time - start_time
    print(execution_time)

    plot_grid(grid, path, start, goal)