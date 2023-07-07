import matplotlib.pyplot as plt
import time

def plot_obstacles(previous_obstacles, new_obstacles, updated_obstacles, updated_labels):
    # Extract x and y coordinates from the obstacles lists
    prev_x, prev_y = zip(*previous_obstacles)
    new_x, new_y = zip(*new_obstacles)
    updated_x, updated_y = zip(*updated_obstacles)

    # Create a new figure
    plt.figure()

    # Plot the obstacle positions from each list
    plt.scatter(prev_x, prev_y, color='red', label='Previous Obstacles', s=100)
    plt.scatter(new_x, new_y, color='blue', label='New Obstacles', s=80)
    plt.scatter(updated_x, updated_y, color='green', label='Updated Obstacles')

    for obstacle, label in zip(updated_obstacles, updated_labels):
        plt.text(obstacle[0], obstacle[1], label, ha='center', va='bottom')  # Display the label near the obstacle

    # Add labels and legend
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.xlim(0, 1.4)
    plt.ylim(0, 1.4)
    plt.legend()

    # Show the plot
    plt.show(block=False)

def plot_obstacle_with_labels(obstacle_positions, obstacle_labels, title='plot'):
    fig, ax = plt.subplots()

    # Plot the obstacles
    for label, position in zip(obstacle_labels, obstacle_positions):
        x, y = position
        ax.plot(x, y, 'bo')  # Plot as blue circles
        ax.text(x, y, label, ha='center', va='bottom')  # Add the label as text
    plt.xlim(0, 1.4)
    plt.ylim(0, 1.4)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    plt.title(title)

    current_time = int(time.time())
    file_name = "/home/weilin/workspace/catkin_ws/src/pose_reader/plots/image{}.jpg".format(current_time)
    plt.savefig(file_name)

    # plt.show(block=False)

def plot_path(path, title='plot'):
    # Plot the arena, obstacles, tree edges, and the resulting path
    plt.figure()
    plt.plot([point[0] for point in path], [point[1] for point in path], 'go')
    plt.plot([point[0] for point in path], [point[1] for point in path], 'g-')
    plt.axis('equal')
    plt.xlim(0, 1.4)
    plt.ylim(0, 1.4)
    plt.title(title)

    current_time = int(time.time())
    file_name = "/home/weilin/workspace/catkin_ws/src/pose_reader/plots/image{}.jpg".format(current_time)
    plt.savefig(file_name)

    # plt.show(block=False)