import time
import math
import roslibpy
import numpy as np

# ROS connection setup
client = roslibpy.Ros(host='192.168.8.104', port=9012)
client.run()

robot_name = 'india'
mapper_topic = roslibpy.Topic(client, f'/{robot_name}/mapper', 'nav_msgs/OccupancyGrid')
lidar_topic = roslibpy.Topic(client, f'/{robot_name}/scan', 'sensor_msgs/LaserScan')
odom_topic = roslibpy.Topic(client, f'/{robot_name}/odom', 'nav_msgs/Odometry')

# Grid setup
grid_width = 500
grid_height = 500
resolution = 0.1
occupancy_grid = [-1] * (grid_width * grid_height)

# Robot state
robot_pos = [0.0, 0.0]
robot_yaw = 0.0
robot_speed = 0.0

# Threshold for detecting if robot is moving
MOVEMENT_THRESHOLD = 0.01  # m/s

# LiDAR callback
def lidar_callback(msg):
    global occupancy_grid
    angle = msg['angle_min']
    
    for r in msg['ranges']:
        if math.isinf(r) or math.isnan(r):
            angle += msg['angle_increment']
            continue
        
        # Point in robot's local frame
        local_x = r * math.cos(angle)
        local_y = r * math.sin(angle)
        angle += msg['angle_increment']

        # Transform to map (world) frame using robot's pose
        yaw_rad = math.radians(robot_yaw)
        map_x = robot_pos[0] + (local_x * math.cos(yaw_rad) - local_y * math.sin(yaw_rad))
        map_y = robot_pos[1] + (local_x * math.sin(yaw_rad) + local_y * math.cos(yaw_rad))

        # Convert world coordinates to map grid index
        gx = int((map_x + (grid_width * resolution) / 2) / resolution)
        gy = int((map_y + (grid_height * resolution) / 2) / resolution)
        
        if 0 <= gx < grid_width and 0 <= gy < grid_height:
            idx = gy * grid_width + gx
            occupancy_grid[idx] = 100  # Mark obstacle

        # Free space between robot and obstacle
        x_diff = map_x - robot_pos[0]
        y_diff = map_y - robot_pos[1]
        dist = math.sqrt(x_diff**2 + y_diff**2)
        steps = int(dist / resolution)

        for i in range(steps):
            step_x = robot_pos[0] + (x_diff * i / steps)
            step_y = robot_pos[1] + (y_diff * i / steps)

            gx_step = int((step_x + (grid_width * resolution) / 2) / resolution)
            gy_step = int((step_y + (grid_height * resolution) / 2) / resolution)

            if 0 <= gx_step < grid_width and 0 <= gy_step < grid_height:
                idx_step = gy_step * grid_width + gx_step
                if occupancy_grid[idx_step] != 100:
                    occupancy_grid[idx_step] = 0  # Free space

# Odometry callback
def odom_callback(msg):
    global robot_pos, robot_yaw, robot_speed
    robot_pos[0] = msg['pose']['pose']['position']['x']
    robot_pos[1] = msg['pose']['pose']['position']['y']

    # Orientation
    o = msg['pose']['pose']['orientation']
    x, y, z, w = o['x'], o['y'], o['z'], o['w']
    yaw = math.atan2(2 * (x * y + z * w), 1 - 2 * (y**2 + z**2))
    robot_yaw = math.degrees(yaw)

    # Linear speed
    vx = msg['twist']['twist']['linear']['x']
    vy = msg['twist']['twist']['linear']['y']
    robot_speed = math.sqrt(vx**2 + vy**2)

# Create grid message
# Create occupancy grid message with fixed map origin and orientation
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
                'position': {
                    'x': - (grid_width * resolution) / 2,  # Fixed origin (centered map)
                    'y': - (grid_height * resolution) / 2,
                    'z': 0.0
                },
                'orientation': {
                    'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0  # No rotation
                }
            }
        },
        'data': occupancy_grid
    }


# Subscribe
lidar_topic.subscribe(lidar_callback)
odom_topic.subscribe(odom_callback)
print(f"Subscribed to {lidar_topic.name} and {odom_topic.name}")

# Auto-publish only if robot is still
try:
    while client.is_connected:
        if robot_speed < MOVEMENT_THRESHOLD:
            print("Robot is still — publishing map...")
            mapper_topic.publish(roslibpy.Message(make_grid()))
        else:
            print("Robot moving — skipping map update.")
        time.sleep(1)
except KeyboardInterrupt:
    print("Interrupted by user")

# Cleanup
lidar_topic.unsubscribe()
odom_topic.unsubscribe()
mapper_topic.unadvertise()
client.terminate()
