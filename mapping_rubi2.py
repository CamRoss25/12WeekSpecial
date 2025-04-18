import time
import math
import roslibpy
import pygame

# ROS connection setup
client = roslibpy.Ros(host='192.168.8.104', port=9012)
client.run()

robot_name = 'juliet'
mapper_topic = roslibpy.Topic(client, f'/{robot_name}/mapper', 'nav_msgs/OccupancyGrid')
lidar_topic = roslibpy.Topic(client, f'/{robot_name}/scan', 'sensor_msgs/LaserScan')

# Grid setup
grid_width = 100
grid_height = 100
resolution = 0.05
occupancy_grid = [-1] * (grid_width * grid_height)

# LiDAR callback
def lidar_callback(msg):
    global occupancy_grid
    occupancy_grid = [-1] * (grid_width * grid_height)
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
            occupancy_grid[idx] = 100

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
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
            }
        },
        'data': occupancy_grid
    }

# Subscribe to LiDAR
lidar_topic.subscribe(lidar_callback)
print(f"Subscribed to {lidar_topic.name}")

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
                    mapper_topic.publish(roslibpy.Message(make_grid()))
                if event.key == pygame.K_r:
                    print("Resetting occupancy grid due to 'r' press.")
                    occupancy_grid = [-1] * (grid_width * grid_height)
except KeyboardInterrupt:
    print("Interrupted by user")

# Cleanup
lidar_topic.unsubscribe()
mapper_topic.unadvertise()
client.terminate()
pygame.quit()
