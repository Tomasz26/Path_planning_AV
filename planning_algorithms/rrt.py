import numpy as np
import matplotlib.pyplot as plt
import random
import time

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

def distance(n1, n2):
    return np.sqrt((n1.x - n2.x) ** 2 + (n1.y - n2.y) ** 2)

def get_random_point(bounds):
    return Node(random.uniform(bounds[0], bounds[1]), random.uniform(bounds[2], bounds[3]))

def nearest_node(nodes, qrand):
    return min(nodes, key=lambda node: distance(node, qrand))

def steer(qnearest, qrand, step_size):
    theta = np.arctan2(qrand.y - qnearest.y, qrand.x - qnearest.x)
    new_x = qnearest.x + step_size * np.cos(theta)
    new_y = qnearest.y + step_size * np.sin(theta)
    return Node(new_x, new_y, parent=qnearest)

def is_line_collision_free(qnearest, qnew, obstacles, steps=10):
    for i in range(steps + 1):
        x = qnearest.x + (qnew.x - qnearest.x) * i / steps
        y = qnearest.y + (qnew.y - qnearest.y) * i / steps
        for ox, oy, width, height in obstacles:
            if ox <= x <= ox + width and oy <= y <= oy + height:
                return False
    return True

def is_collision_free(qnew, qnearest, obstacles):
    return is_line_collision_free(qnearest, qnew, obstacles)

def generate_path(qgoal, nodes):
    path = []
    node = qgoal
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

def rrt(qstart, qgoal, bounds, obstacles, step_size=5, max_iter=1000, goal_threshold=5):

    nodes = [qstart]
    for _ in range(max_iter):
        qrand = get_random_point(bounds)
        qnearest = nearest_node(nodes, qrand)
        qnew = steer(qnearest, qrand, step_size)

        if is_collision_free(qnew, qnearest, obstacles):
            nodes.append(qnew)
            if distance(qnew, qgoal) < goal_threshold:
                qgoal.parent = qnew
                nodes.append(qgoal)
                return generate_path(qgoal, nodes), nodes

    return None, nodes

def plot_rrt(path, nodes, qstart, qgoal, obstacles):
    fig, ax = plt.subplots()
    ax.set_xlim(-10, 110)
    ax.set_ylim(-10, 110)
    
    for node in nodes:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "g.-")
    
    for ox, oy, width, height in obstacles:
        rect = plt.Rectangle((ox, oy), width, height, color='r', fill=True)
        ax.add_patch(rect)
    
    if path:
        px, py = zip(*path)
        plt.plot(px, py, 'b', linewidth=2)
    
    plt.plot(qstart.x, qstart.y, 'go', markersize=10, label="Start")
    plt.plot(qgoal.x, qgoal.y, 'ro', markersize=10, label="Goal")
    plt.legend()
    plt.show()

# Definiowanie środowiska
qstart = Node(10, 10)
qgoal = Node(90, 90)
bounds = (0, 100, 0, 100)
obstacles = [(15, 40, 30, 10), (25, 70, 40, 10)]

# Uruchomienie RRT i wizualizacja
start_time = time.time()
path, nodes = rrt(qstart, qgoal, bounds, obstacles)
end_time = time.time()
execution_time = end_time - start_time
print(execution_time)
plot_rrt(path, nodes, qstart, qgoal, obstacles)

