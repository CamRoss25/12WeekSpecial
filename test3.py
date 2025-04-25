import math
import time
import roslibpy
from collections import deque
import keyboard


class OccupancyGridMapper:
    def __init__(self, ip, port, robot_name, topic_name):
        # ROS connection settings
        self.ip = ip
        self.port = port
        self.robot_name = robot_name
        self.topic_name = topic_name

        # ROS connection
        self.ros = roslibpy.Ros(host=self.ip, port=self.port)
        self.ros.run()
        print(f"Connected to ROS: {self.ros.is_connected}")

        # ROS Topic setup
        self.map_topic = roslibpy.Topic(self.ros, f'/{self.robot_name}/{self.topic_name}', 'nav_msgs/OccupancyGrid')
        self.odom_topic = roslibpy.Topic(self.ros, f'/{self.robot_name}/odom', 'nav_msgs/Odometry')
        self.scan_topic = roslibpy.Topic(self.ros, f'/{self.robot_name}/scan', 'sensor_msgs/LaserScan')

        # Robot and map parameters
        self.resolution = 0.1 # 0.1 meters / cell
        self.map_width = 600  # 60 meters 
        self.map_height = 600 # 60 meters 
        self.map_origin_x = -30.0
        self.map_origin_y = -30.0
        self.robot_radius_m = 0.15
        self.robot_radius_cells = int(self.robot_radius_m / self.resolution)

        # Odometry and laser data
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.initial_yaw = None
        self.odom_buffer = deque(maxlen=100)
        self.ranges = []
        self.angs = []
        self.scan_time = 0.0
        self.scan_duration = 0.1 # seconds

        # Map hit and miss counters
        self.hit_counts = [0] * (self.map_width * self.map_height)
        self.miss_counts = [0] * (self.map_width * self.map_height)
        self.persistent_map = [-1] * (self.map_width * self.map_height)

        # Rest ODOM (X - Front) (Y - Left) (Z - Up)
        self.reset_odometry()

        # Subscribe to ROS Topics
        self.odom_topic.subscribe(self.callback_odom)
        self.scan_topic.subscribe(self.callback_scan)

    def wrap_to_pi(self, angle):
        """Ensure the yaw angle is wraped from -pi to pi"""
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    def callback_odom(self, message):
        """Function utilized to store ODOM Ros 2 messages in initialized variables"""
        try: 
            pos = message['pose']['pose']['position']
            o = message['pose']['pose']['orientation']
            qx, qy, qz, qw = o.get('x'), o.get('y'), o.get('z'), o.get('w')
            raw_yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

            if self.initial_yaw is None:
                self.initial_yaw = raw_yaw

            self.x = pos.get('x')
            self.y = pos.get('y')
            self.yaw = self.wrap_to_pi(raw_yaw - self.initial_yaw) # ensures that the first yaw angle is 0 rads

            # Check for timestamp
            stamp = message['header']['stamp']
            if stamp:
                time_sec = stamp['sec'] + stamp['nanosec'] * 1e-9 # time stamp from odom data subscription in seconds
            else:
                time_sec = time.time()  # fallback to system time

            # Store timestamped pose
            self.odom_buffer.append({
                'time': time_sec,
                'x': self.x,
                'y': self.y,
                'yaw': self.yaw
            })

        except KeyError as e:
            print(f"KeyError in callback_odom: {e}")
            print(f"Message received: {message}")

    def callback_scan(self, msg):
        try:
            # Safely extract time
            stamp = msg['header']['stamp']
            if stamp:
                self.scan_time = stamp['sec'] + stamp['nanosec'] * 1e-9
            else:
                self.scan_time = time.time()  # fallback to system time

            ang_min = msg['angle_min']
            ang_inc = msg['angle_increment']
            self.ranges = msg['ranges']
            self.angs = [ang_min + i * ang_inc for i in range(len(self.ranges))] # calcualte the angle of LIDAR hits of objects

            # Try to get scan duration safely
            if msg['time_increment'] > 0:
                self.scan_duration = msg['time_increment'] * len(self.ranges) # calculate the scan time based on the number of ranges detected
            else:
                self.scan_duration = 0.1  # reasonable default for scan time

        except KeyError as e:
            print(f"KeyError in callback_scan: {e}")
            print(f"Message received: {msg}")

    def reset_odometry(self):
        """Reset odometry to the origin and set Initial Yaw to 0 rads."""
        reset_odom_service = roslibpy.Service(self.ros, f'/{self.robot_name}/reset_pose', 'irobot_create_msgs/ResetPose')
        result = reset_odom_service.call(roslibpy.ServiceRequest())
        print("ODOM Reset:", result)
        self.initial_yaw = None
        self.yaw = 0

    def reset_map(self):
        """Reset the entire occupancy map to unknown (-1)."""
        print("Resetting map to unknown (-1)...")
        self.hit_counts = [0] * (self.map_width * self.map_height)
        self.miss_counts = [0] * (self.map_width * self.map_height)
        self.persistent_map = [-1] * (self.map_width * self.map_height)
    

    def interpolate_pose(self, query_time):
        """Used to sync the odom and scan time stamps"""

        # This is mainly used to eliminate our ODOM map from becoming blurry or having ghost readings
        # Interpolating estimates the position of the robot when a LIDAR scan is taking place

        if len(self.odom_buffer) < 2: # need at least 2 odom time stamps in order to sync with scan data
            return None

        for i in range(len(self.odom_buffer) - 1): # if the number of ODOM time stamps are greater than 2 (so 3 scans) run this For loop for those 2 ODOM messagess
            t0, t1 = self.odom_buffer[i]['time'], self.odom_buffer[i + 1]['time'] # loop through the two ODOM timestamps (t0 and t1)
            if t0 <= query_time <= t1: # found two ODOM positions that "surround" the desired time
                alpha = (query_time - t0) / (t1 - t0) # calculates ratio between 0 and 1 to see how far query_time is between t0 and t1
                # Linear Interpolation of x and y positions
                x = (1 - alpha) * self.odom_buffer[i]['x'] + alpha * self.odom_buffer[i + 1]['x']
                y = (1 - alpha) * self.odom_buffer[i]['y'] + alpha * self.odom_buffer[i + 1]['y']
                # Angle Interpolation of yaw
                yaw0 = self.odom_buffer[i]['yaw']
                yaw1 = self.odom_buffer[i + 1]['yaw']
                # Handle yaw wraparound
                dyaw = self.wrap_to_pi(yaw1 - yaw0)
                yaw = self.wrap_to_pi(yaw0 + alpha * dyaw)
                return x, y, yaw

        return None  # If time is outside buffer

    def make_grid(self):
        if not self.ranges or not self.angs:
            print("LIDAR data empty")
            return None

        MAX_CONFIDENCE = 10

        def set_cell(xi, yi, is_occupied):
            if 0 <= xi < self.map_width and 0 <= yi < self.map_height:
                idx = yi * self.map_width + xi # index the locatoin of cell

                if is_occupied:
                    self.hit_counts[idx] = min(self.hit_counts[idx] + 1, MAX_CONFIDENCE)
                else:
                    self.miss_counts[idx] = min(self.miss_counts[idx] + 1, MAX_CONFIDENCE)

        # Convert robots postion from ODOM Coordinate to Ocupancy Coordinate
        robot_grid_x = int((self.x - self.map_origin_x) / self.resolution)
        robot_grid_y = int((self.y - self.map_origin_y) / self.resolution)

        # MAX and MIN range settings of LIDAR 
        min_valid_range = 0.25 # meters
        max_valid_range = 15.0  # LIDAR range is 30 meters however max_valid_range set to 5 meters for more accurate ODOM map

        # Iterate over all of the LIDAR rays and look at that rays' specific angle and range
        for i, (r, a) in enumerate(zip(self.ranges, self.angs)): # zip a range angle pair into a tuple then enumerate (index) the range and angle pair
            if not (min_valid_range < r < max_valid_range): # only look at LIDAR rays inbetween the MAX and MIN range
                continue

            # Estimate robot's position at the exact time the scan was taken using Interpolation
            time_offset = (i / len(self.ranges)) * self.scan_duration # calculate any offset time from the scan time
            ray_time = self.scan_time + time_offset 

            pose = self.interpolate_pose(ray_time)
            if pose is None:
                continue

            pose_x, pose_y, pose_yaw = pose

            # Convert LIDAR measurment into ODOM Coord System
            hit_x = pose_x + r * math.cos(pose_yaw + a)
            hit_y = pose_y + r * math.sin(pose_yaw + a)

            # ODOM to Ocupancy Map
            grid_x = int((hit_x - self.map_origin_x) / self.resolution)
            grid_y = int((hit_y - self.map_origin_y) / self.resolution)

            # Bresenham's algorithm to trace free cells
            # This marks all cells along the scan as "Free" and the final cell as "Occupied"
            x0, y0 = robot_grid_x, robot_grid_y
            x1, y1 = grid_x, grid_y
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            sx = 1 if x0 < x1 else -1
            sy = 1 if y0 < y1 else -1
            err = dx - dy

            # Aligning the ODOM map to Ocupancy Map for free and occupied cells
            while True:
                set_cell(x0, y0, is_occupied=False)  # Free cell
                if x0 == x1 and y0 == y1:
                    break
                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    x0 += sx
                if e2 < dx:
                    err += dx
                    y0 += sy

            set_cell(x1, y1, is_occupied=True)  # Hit point = occupied

        hit_threshold = 3
        miss_threshold = 5

        # This will mark a cell as being "Free" or "Occupied"
        for idx in range(len(self.persistent_map)):
            if self.hit_counts[idx] >= hit_threshold:
                self.persistent_map[idx] = 100
            elif self.miss_counts[idx] >= miss_threshold:
                self.persistent_map[idx] = 0
            else:
                self.persistent_map[idx] = -1

        # Create a copy of the persistent map for publishing
        map_with_robot = self.persistent_map[:]

        # Draw the robot's current position as a filled circle (temporary overlay)
        for dx in range(-self.robot_radius_cells, self.robot_radius_cells + 1):
            for dy in range(-self.robot_radius_cells, self.robot_radius_cells + 1):
                if dx * dx + dy * dy <= self.robot_radius_cells ** 2:
                    cx = robot_grid_x + dx
                    cy = robot_grid_y + dy
                    if 0 <= cx < self.map_width and 0 <= cy < self.map_height:
                        idx = cy * self.map_width + cx
                        map_with_robot[idx] = 50  # semi-occupied value for robot

        arrow_length_cells = 5  # Length of the arrow in cells

        # Draw the robot's heading direction as a line
        for i in range(arrow_length_cells):
            arrow_xi = robot_grid_x + int(round(i * math.cos(self.yaw)))
            arrow_yi = robot_grid_y + int(round(i * math.sin(self.yaw)))

            if 0 <= arrow_xi < self.map_width and 0 <= arrow_yi < self.map_height:
                idx = arrow_yi * self.map_width + arrow_xi
                map_with_robot[idx] = 75  # Arbitrary value to show arrow (darker than robot circle)


        grid_msg = {
            'header': {
                'frame_id': 'odom',
                'stamp': {'secs': int(time.time()), 'nsecs': 0}
            },
            'info': {
                'map_load_time': {'secs': int(time.time()), 'nsecs': 0},
                'resolution': self.resolution,
                'width': self.map_width,
                'height': self.map_height,
                'origin': {
                    'position': {'x': self.map_origin_x, 'y': self.map_origin_y, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
            },
            'data': map_with_robot,
        }
        return grid_msg
    
    def start_mapping(self):
        print("Press 'r' to reset the map.")
        while self.ros.is_connected:
            # Check for reset key
            if keyboard.is_pressed('r'):
                self.reset_map()
                time.sleep(0.5)  # debounce to avoid rapid multiple resets
                print("Map has been reset")

            # Send mapping message
            grid_msg = self.make_grid()
            if grid_msg:
                self.map_topic.publish(roslibpy.Message(grid_msg))
                print("Publishing Map")
            else:
                print("No map data to publish")

            time.sleep(2)  # mapping frequency


# Example of how to initialize and run the mapper
if __name__ == "__main__":
    mapper = OccupancyGridMapper(ip='192.168.8.104', port=9012, robot_name='echo', topic_name="mapmike")
    mapper.start_mapping()