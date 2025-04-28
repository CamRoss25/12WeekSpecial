## mapping attempt

import time
import math
import roslibpy
import pygame
import numpy as np

# ROS connection setup
client = roslibpy.Ros(host='192.168.8.104', port=9012)
client.run()

robot_name = 'foxtrot'
mapper_topic = roslibpy.Topic(client, f'/{robot_name}/mapper', 'nav_msgs/OccupancyGrid')
lidar_topic = roslibpy.Topic(client, f'/{robot_name}/scan', 'sensor_msgs/LaserScan')
odom_topic = roslibpy.Topic(client, f'/{robot_name}/odom', 'nav_msgs/Odometry')

# Grid setup
grid_width = 100
grid_height = 100
resolution = 0.05
occupancy_grid = [-1] * (grid_width * grid_height)

# Robot position and orientation
robot_pos = [0.0, 0.0]  # x, y position of the robot
robot_yaw = 0.0  # Robot's yaw (orientation)

# LiDAR callback
def lidar_callback(msg):
    global occupancy_grid
    occupancy_grid = [-1] * (grid_width * grid_height)  # Reset occupancy grid
    angle = msg['angle_min']
    for r in msg['ranges']:
        if math.isinf(r) or math.isnan(r):
            angle += msg['angle_increment']
            continue
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        angle += msg['angle_increment']
        gx = int((x + (grid_width * resolution) / 2) / resolution)
        gy = int((y + (grid_height * resolution) / 2) / resolution)
        if 0 <= gx < grid_width and 0 <= gy < grid_height:
            idx = gy * grid_width + gx
            occupancy_grid[idx] = 100  # Mark the cell as occupied

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

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((300, 100))
pygame.display.set_caption('Press P to Publish Map')

try:
    running = True
    while running and client.is_connected:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p:
                    print("Publishing occupancy grid due to 'p' press.")
                    #mapper_topic.publish(roslibpy.Message(make_grid()))
                if event.key == pygame.K_r:
                    print("Resetting occupancy grid due to 'r' press.")
                    occupancy_grid = [-1] * (grid_width * grid_height)
except KeyboardInterrupt:
    print("Interrupted by user")

# Cleanup
lidar_topic.unsubscribe()
odom_topic.unsubscribe()
mapper_topic.unadvertise()
client.terminate()
pygame.quit()
