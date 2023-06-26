import math
from path_planning import RRT
from go_to_point import follow_point
def calculate_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def traverse_points(start_point, end_point, point_set):
    current_point = start_point
    while point_set:
        if len(point_set)>= 2:
          distances = [calculate_distance(current_point, point) for point in point_set[0:3]] #determine the closeset point from the first 2 points in the set
          closest_point_index = distances.index(min(distances))
          closest_point = point_set[closest_point_index]
        
          current_point = closest_point
          
          # Remove the visited point from the set
          point_set.pop(closest_point_index)
          
          # Move the robot towards the closest point (assuming it takes some time to move)
          current_point = follow_point(closest_point)
        else:
          current_point = end_point
          point_set.pop(0)
    return current_point

def traverse_goal_points(start_point, goal_point_set):
    
    rospy.init_node('task1', anonymous=True)

    current_goal_point = start_point
    while goal_point_set:
        distances = [calculate_distance(current_goal_point, point) for point in goal_point_set] #determine the closeset point from the set of points 
        closest_goal_point_index = distances.index(min(distances))
        closest_goal_point = goal_point_set[closest_goal_point_index]
      
        # Move the robot towards the closest point (assuming it takes some time to move)
        current_goal_point = closest_goal_point
        rospy.loginfo("task1: closest_point: " + str(closest_goal_point))
        
        # Create an instance of the RRT class and run the algorithm
        rrt = RRT(start_point, current_goal_point, 3, (1.4, 1.4))
        if rrt.extend_tree():
            # If a path is found, retrieve the path and plot it
            path1 = rrt.find_path()
            #smooth path to less points
            smoothed_path  = rrt.smooth_path(path1)
            rrt.plot_smoothed_path(smoothed_path)
        else:
            rospy.loginfo("task1: unable to find path!")
        start_point = traverse_points(start_point, closest_goal_point, smoothed_path)

        # Remove the visited point from the set
        goal_point_set.pop(closest_goal_point_index)


if __name__ == '__main__':
    # Example usage
    start_point = (0.2, 0.2)
    goal_point_set = [(0.2, 1.2), (1.2, 1.2), (1.2, 0.2)]
    traverse_goal_points(start_point, goal_point_set)