"""Smart_Route_Controller controller."""

from controller import Supervisor

from astar import a_star
from grid import world_to_grid
from sensors import setup_sensors, read_sensors
from navigation import move_forward, stop, turn_left, turn_right, correct_left, correct_right
from fsm import RobotState
import math


# Create the Robot instance
robot = Supervisor()

# Current state of the robot
state = RobotState.FOLLOW_PATH
avoid_direction = None

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# Get the e-puck's position in the Webots world
robot_node = robot.getSelf()
position_field = robot_node.getField("translation")
rotation_field = robot_node.getField("rotation")

# Goal position
goal_x = 0.93
goal_y = 0.96

# Starting position
start_x = -0.92468
start_y = -0.69

# Size of each A* grid cell in meters
cell_size = 0.2
    
# Calculate the goal's grid position
goal_grid = world_to_grid(goal_x, goal_y, start_x, start_y, cell_size)
print("Goal grid position:", goal_grid)

# Calculate an A* path from the start to the goal
start_grid = world_to_grid(start_x, start_y, start_x, start_y, cell_size)

obstacles = set()

path = a_star(start_grid, goal_grid, obstacles)

print("A* path:", path)

def replan_path(x, y):
    current_grid = world_to_grid(x, y, start_x, start_y, cell_size)

    print("Replanning A*")
    print("New start grid:", current_grid)

    new_path = a_star(current_grid, goal_grid, obstacles)

    print("New A* path:", new_path)

    return new_path

path_index = 1

turning = True

turn_steps = 0

# Get e-puck motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# Put the motors into velocity control mode
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

# Set up the e-puck's 8 distance sensors
sensors = setup_sensors(robot, timestep)

# Main loop
while robot.step(timestep) != -1:
    
    if turning:
        turn_left(left_motor, right_motor, 3.0)
        turn_steps += 1

        if turn_steps >= 25:
            turning = False
            print("Initial turn complete")

        continue
        
     # Get the robot's current position
    position = position_field.getSFVec3f()
    rotation = rotation_field.getSFRotation()

    x = position[0]
    y = position[1]
    
    print("Actual position:", x, y)
    print("Webots position:", position)
    print("Rotation:", rotation)
    
    grid_position = world_to_grid(x, y, start_x, start_y, cell_size)

    print("Robot grid position:", grid_position)
    
    # Update path index when the robot reaches the current waypoint
    if grid_position == path[path_index] and path_index < len(path) - 1:
        path_index += 1
    
    next_grid_position = path[path_index]
    print("Next A* position:", next_grid_position)
    
    # Calculate where the next A* waypoint is in the Webots world
    target_x = start_x + next_grid_position[0] * cell_size
    target_y = start_y + next_grid_position[1] * cell_size

    # Calculate the angle from the robot to the next waypoint
    target_angle = math.atan2(target_y - y, target_x - x)
    
    # Calculate distance to the current A* waypoint
    distance_to_waypoint = ((target_x - x) ** 2 + (target_y - y) ** 2) ** 0.5

    # Get the robot's current rotation
    rotation = rotation_field.getSFRotation()
    current_angle = rotation[3]

    # Webots may use a negative Z axis when the robot turns
    if rotation[2] < 0:
        current_angle = -current_angle

    # Calculate how far the robot needs to turn
    angle_difference = target_angle - current_angle

    # Keep the angle between -pi and +pi
    while angle_difference > math.pi:
        angle_difference -= 2 * math.pi

    while angle_difference < -math.pi:
        angle_difference += 2 * math.pi

    print("Target angle:", target_angle)
    print("Current angle:", current_angle)
    print("Angle difference:", angle_difference)
    
    
    # Calculate the difference between the robot and the goal
    x_difference = goal_x - x
    y_difference = goal_y - y
    
    # Check if the robot has reached the goal
    distance_to_goal = ((goal_x - x) ** 2 + (goal_y - y) ** 2) ** 0.5

    if distance_to_goal < 0.1:
        print("Goal reached!")
        stop(left_motor, right_motor)
        break
    
    # Read the distance sensors
    sensor_values = read_sensors(sensors)

    # Front/right sensors
    right_front = sensor_values[0]
    right_fside = sensor_values[1]
    right_side = sensor_values[2]

    # Front/left sensors
    left_front = sensor_values[7]
    left_fside = sensor_values[6]
    left_side = sensor_values[5]
    
    # Back sensors
    back_right = sensor_values[3]
    back_left = sensor_values[4]

    print("Back sensors:", back_right, back_left)

    # Check if something is in front of the robot
    obstacle_detected = (
    right_front > 100 or right_fside > 100 or
    left_front > 100 or left_fside > 100
    )

    # If we are currently avoiding an obstacle
    if state == RobotState.AVOID_OBSTACLE:

        print("Avoiding obstacle")

        # If we have not chosen a direction yet,
        # decide which way to turn around the obstacle
        if avoid_direction is None:

            right_total = right_front + right_fside + right_side
            left_total = left_front + left_fside + left_side

            if right_total > left_total:

                avoid_direction = "left"
                print("Avoiding obstacle on the left")

            else:

                avoid_direction = "right"
                print("Avoiding obstacle on the right")

        # Keep turning until the front of the robot is clear
        if right_front > 100 or left_front > 100:

            if avoid_direction == "left":
                print("Turning left around obstacle")
                turn_left(left_motor, right_motor, 3.0)

            else:
                print("Turning right around obstacle")
                turn_right(left_motor, right_motor, 3.0)

        else:

            # The front is clear, so move forward around the obstacle
            print("Front is clear - moving forward")
            move_forward(left_motor, right_motor, 3.0)

            # Once all of the obstacle sensors are clear,
            # return control to A*
            if (right_front <= 100 and right_fside <= 100 and
                    right_side <= 100 and
                    left_front <= 100 and left_fside <= 100 and
                    left_side <= 100 and
                    back_right <= 80 and
                    back_left <= 80):

                print("Obstacle completely clear")
                
                # Recalculate A* from the robot's current position
                path = replan_path(x, y)
                path_index = 1
                
                print("CHANGING STATE TO FOLLOW_PATH")
                state = RobotState.FOLLOW_PATH
                avoid_direction = None

    # If an obstacle is detected while following the path
    elif state == RobotState.FOLLOW_PATH and obstacle_detected:

        print("Obstacle detected!")
        state = RobotState.AVOID_OBSTACLE

        # Decide which direction to initially turn
        right_total = right_front + right_fside + right_side
        left_total = left_front + left_fside + left_side

        if right_total > left_total:

            avoid_direction = "left"
            print("Starting left obstacle avoidance")
            turn_left(left_motor, right_motor, 3.0)

        else:

            avoid_direction = "right"
            print("Starting right obstacle avoidance")
            turn_right(left_motor, right_motor, 3.0)
   
   # Follow the A* path normally
    elif state == RobotState.FOLLOW_PATH:
   
       # Follow the A* waypoint
       if angle_difference < -0.2:
           
           print("Turning right toward A* waypoint") 
           turn_right(left_motor, right_motor, 3.0)
           
       elif angle_difference > 0.2: 
       
           print("Turning left toward A* waypoint") 
           turn_left(left_motor, right_motor, 3.0)
           
       else: 
           # We are facing close enough to the waypoint 
           move_forward(left_motor, right_motor, 3.0)
