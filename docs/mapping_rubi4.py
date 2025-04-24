import time
import math
import roslibpy
import numpy as np

# ROS connection setup
client = roslibpy.Ros(host='192.168.8.104', port=9012)
client.run()

robot_name = 'juliet'
mapper_topic = roslibpy.Topic(client, f'/{robot_name}/mapper', 'nav_msgs/OccupancyGrid')
lidar_topic = roslibpy.Topic(client, f'/{robot_name}/scan', 'sensor_msgs/LaserScan')
odom_topic = roslibpy.Topic(client, f'/{robot_name}/odom', 'nav_msgs/Odometry')

# Grid setup
grid_width = 200
grid_height = 200
resolution = 0.05
occupancy_grid = [-1] * (grid_width * grid_height)

# Robot position and orientation
robot_pos = [0.0, 0.0]  # x, y position of the robot
robot_yaw = 0.0  # Robot's yaw (orientation)

# LiDAR callback
# going through each box in the grid to make it a 0 or 1, the ranges, need the boxes between the wall for consideration
# steps to the box 
import math

# LiDAR callback to update occupancy grid
def lidar_callback(msg):
    global occupancy_grid
    angle = msg['angle_min']
    
    # Iterate through the LiDAR range data
    for r in msg['ranges']:
        if math.isinf(r) or math.isnan(r):
            angle += msg['angle_increment']
            continue
        
        # Calculate the x, y coordinates of the point based on the range and angle
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        angle += msg['angle_increment']
        
        # Convert the world coordinates to grid coordinates
        gx = int((x + (grid_width * resolution) / 2) / resolution)
        gy = int((y + (grid_height * resolution) / 2) / resolution)
        
        # If the grid coordinates are valid, update the grid
        if 0 <= gx < grid_width and 0 <= gy < grid_height:
            idx = gy * grid_width + gx
            occupancy_grid[idx] = 100  # Mark the cell as occupied (obstacle)
        
        # Now, trace back from the robot's current position to this point
        # Mark cells as free along the path, avoiding obstacles
        x_robot = robot_pos[0]
        y_robot = robot_pos[1]
        x_diff = x - x_robot
        y_diff = y - y_robot
        dist = math.sqrt(x_diff**2 + y_diff**2)
        
        steps = int(dist / resolution)  # Number of steps to traverse along the line
        
        for i in range(steps):
            # Intermediate points along the line from robot to the obstacle
            step_x = x_robot + (x_diff * i / steps)
            step_y = y_robot + (y_diff * i / steps)
            
            # Convert intermediate points to grid coordinates
            gx_step = int((step_x + (grid_width * resolution) / 2) / resolution)
            gy_step = int((step_y + (grid_height * resolution) / 2) / resolution)
            
            if 0 <= gx_step < grid_width and 0 <= gy_step < grid_height:
                idx_step = gy_step * grid_width + gx_step
                if occupancy_grid[idx_step] != 100:  # Don't overwrite occupied cells
                    occupancy_grid[idx_step] = 0  # Mark as free space (no object)



# Odometry callback to get the robot's position
def odom_callback(msg):
    global robot_pos, robot_yaw
    robot_pos[0] = msg['pose']['pose']['position']['x']
    robot_pos[1] = msg['pose']['pose']['position']['y']
    # Calculate yaw from quaternion
    o = msg['pose']['pose']['orientation']
    x, y, z, w = o['x'], o['y'], o['z'], o['w']
    yaw = math.atan2(2 * (x * y + z * w), 1 - 2 * (y ** 2 + z ** 2))
    robot_yaw = math.degrees(yaw)  # Convert to degrees

# Create occupancy grid message
def make_grid():
    return {
        'header': {
            'frame_id': 'map',
            'stamp': {'secs': int(time.time()), 'nsecs': 0}
        },
        'info': {
            'map_load_time': {'secs': int(time.time()), 'nsecs': 0},
            'resolution': resolution,
            'width': grid_width,
            'height': grid_height,
            'origin': {
                'position': {'x': robot_pos[0], 'y': robot_pos[1], 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': math.radians(robot_yaw), 'w': 1.0}
            }
        },
        'data': occupancy_grid
    }

# Subscribe to LiDAR and Odometry
lidar_topic.subscribe(lidar_callback)
odom_topic.subscribe(odom_callback)
print(f"Subscribed to {lidar_topic.name} and {odom_topic.name}")

# Publish the occupancy grid at regular intervals
try:
    while client.is_connected:
        # print("Publishing occupancy grid...")
        mapper_topic.publish(roslibpy.Message(make_grid()))
        time.sleep(1)  # Publish every 1 second (can adjust the frequency as needed)
except KeyboardInterrupt:
    print("Interrupted by user")

# Cleanup
lidar_topic.unsubscribe()
odom_topic.unsubscribe()
mapper_topic.unadvertise()
client.terminate()
