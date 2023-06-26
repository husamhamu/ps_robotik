#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

class RRT:
    def __init__(self, start, goal, num_obstacles, arena_size, obstacle_size=0.1, step_size=0.1, max_iter=1000):
        self.start = start
        self.goal = goal
        self.num_obstacles = num_obstacles
        self.arena_size = arena_size
        self.obstacle_size = obstacle_size
        self.step_size = step_size
        self.max_iter = max_iter
        self.tree = {tuple(start): None}  # Initialize the tree with the start node
        self.obstacles = self.generate_obstacles()

    def generate_obstacles(self):
        obstacles = []
        np.random.seed(50)
        for _ in range(self.num_obstacles):
            # Generate random positions for the obstacles within the arena size
            x = np.random.uniform(0.1, self.arena_size[0]-0.1)
            y = np.random.uniform(0.1, self.arena_size[1]-0.1)
            obstacles.append((x, y))
        return obstacles

    def generate_point(self):
        while True:
            # Generate a random point within the arena, excluding a border of 0.1 from each edge
            x = np.random.uniform(0.1, self.arena_size[0] - 0.1)
            y = np.random.uniform(0.1, self.arena_size[1] - 0.1) # Make sure generated point is at least 0.1m away from edges
            point = (x, y)
            valid = True
            # Check if the generated point is at least 0.1 distance from any obstacle
            for obstacle in self.obstacles:
                if np.linalg.norm(np.array(point) - np.array(obstacle)) < 0.1:
                    valid = False
                    break
            if valid:
                return point

    def find_nearest_node(self, point):
        # Find the nearest node in the tree to a given point
        distances = [np.linalg.norm(np.array(point) - np.array(node)) for node in self.tree.keys()]
        nearest_node = min(self.tree.keys(), key=lambda x: np.linalg.norm(np.array(point) - np.array(x)))
        return nearest_node

    def steer_towards_point(self, from_node, to_point):
        # Steer from a given node towards a target point with a certain step size
        direction = np.array(to_point) - np.array(from_node)
        direction = direction / np.linalg.norm(direction)
        new_point = np.array(from_node) + self.step_size * direction
        return tuple(new_point)

    def is_collision_free(self, from_node, to_point):
        # Check if the path from a node to a point is collision-free
        for obstacle in self.obstacles:
            if np.linalg.norm(np.array(to_point) - np.array(obstacle)) < 0.1: # Make sure the point is at least 0.2m away from obstacls
                return False
        return True

    def extend_tree(self):
        iter_count = 0
        while iter_count < self.max_iter:
            rand_point = self.generate_point()  # Generate a random point in the arena
            nearest_node = self.find_nearest_node(rand_point)  # Find the nearest node in the tree to the random point
            new_point = self.steer_towards_point(nearest_node, rand_point)  # Steer towards the random point
            if self.is_collision_free(nearest_node, new_point):  # Check if the path is collision-free
                self.tree[new_point] = nearest_node  # Add the new point to the tree with its nearest node as the parent
                if np.linalg.norm(np.array(new_point) - np.array(self.goal)) < self.step_size:
                    # If the new point is close enough to the goal, add the goal to the tree and return True
                    self.tree[self.goal] = new_point
                    return True
            iter_count += 1
        return False

    def find_path(self):
        path = []
        current = self.goal
        while current != self.start:
            # Reconstruct the path from the goal to the start by following the parent pointers in the tree
            path.append(current)
            current = self.tree[current]
        path.append(self.start)
        return path[::-1]  # Reverse the path to get it from start to goal

    def plot_path(self, path):
        # Plot the arena, obstacles, tree edges, and the resulting path
        arena = plt.Rectangle((0, 0), self.arena_size[0], self.arena_size[1], fc='white')
        plt.gca().add_patch(arena)
        for obstacle in self.obstacles:
            plt.gca().add_patch(plt.Circle(obstacle, self.obstacle_size, fc='gray'))
        for node, parent in self.tree.items():
            if parent:
                plt.plot([node[0], parent[0]], [node[1], parent[1]], 'b-')
        plt.plot([self.start[0], self.goal[0]], [self.start[1], self.goal[1]], 'r--')
        plt.plot([point[0] for point in path], [point[1] for point in path], 'go')
        plt.plot([point[0] for point in path], [point[1] for point in path], 'g-')
        plt.plot(self.start[0], self.start[1], 'ro')
        plt.plot(self.goal[0], self.goal[1], 'ro')
        plt.axis('equal')
        plt.xlim(0, self.arena_size[0])
        plt.ylim(0, self.arena_size[1])
        plt.show()

    def smooth_path(self, path):
        smoothed_path = [path[0]]  # Start with the first point in the path
        for point in path[1:]:
            if np.linalg.norm(np.array(point) - np.array(smoothed_path[-1])) >= 0.2:
                smoothed_path.append(point)
        smoothed_path.append(self.goal)  # Add the goal point to the smoothed path
        return smoothed_path

    def plot_smoothed_path(self, path):
        smoothed_path = self.smooth_path(path)
        self.plot_path(smoothed_path)


# Set the parameters for the RRT algorithm
start = (0.2, 0.2)
goal = (0.2, 1.2)
num_obstacles = 5
arena_size = (1.4, 1.4)

# Create an instance of the RRT class and run the algorithm
rrt = RRT(start, goal, num_obstacles, arena_size)

if rrt.extend_tree():
    # If a path is found, retrieve the path and plot it
    path1 = rrt.find_path()
    print(path1)
    print(len(path1))
    rrt.plot_path(path1)
    print("Path found!")
    smoothed_path  = rrt.smooth_path(path1)
    print('smoothed_path', smoothed_path)
    print('len(smoothed_path)', len(smoothed_path))
    rrt.plot_smoothed_path(smoothed_path)
else:
    print("Unable to find a path.")

