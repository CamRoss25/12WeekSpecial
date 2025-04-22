# import necessary files
import roslibpy
import roslibpy.actionlib
from time import *
from threading import *
from colors import *
from Star_spangled_banner import star_spangled_banner
import pygame
from pprint import pp
import math
from math import atan2, degrees
from race_3 import RaceVel
import numpy as np

#initialize pygame and joystics for controll
pygame.init()
pygame.joystick.init() 

class RobotController:
    def __init__(self):
        # Establish Robot Connection to the ROS network
        ip = '192.168.8.104'
        # ip = '127.0.0.1'
        port = 9012
        self.ros = roslibpy.Ros(host = ip, port = port)

        # Define the colors for the light ring from the imported file
        self.ring_colors = ring_colors

        # Define the Robot that we want to control
        self.robot_name = 'foxtrot'
        # create robot topics
        self.control_name = ['cmd_vel', 'cmd_lightring', 'cmd_audio', 'mapper']
        self.topic_types = ['geometry_msgs/Twist', 'irobot_create_msgs/LightringLeds', 'irobot_create_msgs/AudioNoteVector', 'nav_msgs/OccupancyGrid']
        self.topic_names = [f'/{self.robot_name}/{c}' for c in self.control_name]
        self.topics = {}

        # create subscribers
        self.sub_name = ['ir_intensity', 'odom', 'imu', 'mode', 'scan']
        self.sub_types = ['irobot_create_msgs/IrIntensityVector', 'nav_msgs/Odometry', 'sensor_msgs/Imu','std_msgs/String', 'sensor_msgs/LaserScan']
        self.sub_names = [f'/{self.robot_name}/{c}' for c in self.sub_name]
        self.subs = {}

        # self.undock = roslibpy.actionlib.SimpleActionServer(self.ros, '/foxtrot/undock', 'irobot_create_msgs/action/Undock')

        # Define the threads that we want to run
        self.thread_targets = [self.vel_msg, self.light_msg, self.sound_msg, self.grid_msg]
        self.threads = [Thread(target = t) for t in self.thread_targets]

        #overide the backup safety parameter
        # self.param = roslibpy.Param(self.ros, f'/{self.robot_name}/motion_control/safety_overide/')
        # self.param.set('backup_only/')
        
        # initialize changing variables
        self.stop = False
        self.start = False
        self.event = Event()
        self.lock = Lock()
        self.locked = False
        self.armed  = False
        self.manual = True
        self.x_cmd = 0
        self.turn_cmd = 0
        self.turn_vel = 0
        self.x_vel = 0
        self.light_cmd = None
        self.sound_cmd  = None
        self.color_butt = 0
        self.mode_msg = None


        # initialize starting values
        self.yaw_deg = 0
        self.odom_values = [0,0,0]
        self.pose = [0,0,0]
        self.ir_values = [0,0,0,0,0,0,0]
        self.ir_call = 0
        self.ranges = []
        self.grid_array = []
        self.angle_min = 0
        self.angle_max = 0
        self.angle_increment = 0
        self.room_made = 0

        self.velcon = RaceVel(self)

    # this control code is not used in this program but will help with future implementations
    def pygame_events(self):
        while not self.stop: 
            """This retrieves pygame events"""
            pygame.event.get()
            joy = pygame.joystick.Joystick(0)
            hx, hy = joy.get_hat(0)
            A = joy.get_button(0)
            B = joy.get_button(1)
            X = joy.get_button(2)
            Y = joy.get_button(3)
            left_bump = joy.get_button(4)
            right_bump = joy.get_button(5)
            alt_button = joy.get_button(6)
            start_button = joy.get_button(7)
            left_stick_button = joy.get_button(8)
            right_stick_button = joy.get_button(9)
            left_stick_x = round(joy.get_axis(0), 2)
            left_stick_y = round(joy.get_axis(1), 2)    
            right_stick_x = round(joy.get_axis(2), 2)
            right_stick_y = round(joy.get_axis(3), 2)

            if alt_button == 1:
                self.show_subs()

            if left_bump == 1:
                self.reset_pose()
                print("Pose Reset")

            # Up Hat is Armed
            if hy == 1:
                self.armed = True
                self.color_butt = 1
                print(f'\n Robot is Armed')
            # Down Hat is Disarmed
            if hy == -1:
                self.armed = False
                self.color_butt = 1
                print('\n Robot is Disarmed')
            # Left Hat is Manual
            if hx == -1:
                self.manual = True
                self.light_cmd = self.ring_colors[3]
                self.color_butt = 1
                print('\n Robot is in Manual Mode')
            # Right Hat is Autonomous
            if hx == 1:
                self.manual = False
                self.light_cmd = self.ring_colors[2]
                self.color_butt = 1
                print('\n Robot is in Autonomous Mode') 

            if start_button ==1:
                self.stop = True
            
            if B == 1 and self.color_butt != 1:
                self.light_cmd = self.ring_colors[0]
                self.color_butt = 1
                self.ir_call = 1
                print(f"IR Call: {self.ir_call}")
                # print('B is pressed')
            if A == 1 and self.color_butt != 1:
                self.light_cmd = self.ring_colors[1]
                self.color_butt = 1
                self.ir_call = 0
                print(f"IR Call: {self.ir_call}")
                # print('A is pressed')
            if X == 1 and self.color_butt != 1:
                self.light_cmd = self.ring_colors[2]
                self.color_butt = 1
                self.ir_call = -1
                print(f"IR Call: {self.ir_call}")
            
            
            if Y == 1:
                self.sound_cmd = star_spangled_banner[5]
                # print('y is pressed')
            if Y == 0:
                self.sound_cmd = None
            
            if 0.1 < right_stick_x or right_stick_x < -0.1:
                self.turn_cmd = right_stick_x
                # print(f'Right Stick X: {self.turn_cmd}')
            else:
                self.turn_cmd = 0
                
            if  0.1 < left_stick_y or left_stick_y < -0.1:
                self.x_cmd = left_stick_y
                # print(f'Left Stick Y {self.x_cmd}')
            else:
                self.x_cmd = 0
            #print(f"Armed: {self.armed}\tManual: {self.manual}")
    
    # velocity control mesage for the robot
    def vel_msg(self):
        """This publishes a velocity message to our robot"""

        while not self.stop:
            if self.armed == True:
                # If the robot is in manual mode, we will use the joystick to control the robot
                if self.manual == True:
                    if self.x_cmd or self.turn_cmd != 0:
                        x_vel = round((self.x_cmd / 3), 1)* -1.5
                        turn_vel = round(self.turn_cmd, 1) * -1.5

                        msg = {'linear': {'x': x_vel, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': turn_vel}}
                        sleep(0.1)
                        self.topics['topic_cmd_vel'].publish(msg)
                        # print(f"{msg}")
                # if the bot is in autonomous mode, we will use the velocity controller to control the robot
                else:
                    self.vel_msg = self.velcon.vel_msg(self)
                    self.topics['topic_cmd_vel'].publish(self.vel_msg)
                    sleep(0.05)
            # if the robot is not armed, we will stop the robot                         
            else:
                msg = {'linear': {'x': 0, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0}}
                self.topics['topic_cmd_vel'].publish(msg)
                sleep(0.1)
    
    # light controll message for the robot
    def light_msg(self):
        while not self.stop:
                if self.color_butt == 1:
                    if self.armed == False:
                        """This publishes a solid light message to our robot"""
                        msg = {'override_system': True, 
                            'leds': self.light_cmd}
                        self.topics['topic_cmd_lightring'].publish(msg)
                        # print(f"Sent light command")
                        self.color_butt = 0
                        sleep(.1)
                    else:
                        """This publishes a flashing light message to our robot"""
                        msg = {'override_system': True, 
                            'leds': self.light_cmd}
                        self.topics['topic_cmd_lightring'].publish(msg)
                        # print(f"Sent armed light command")
                        sleep(.3)
                        msg = {}
                        self.topics['topic_cmd_lightring'].publish(msg)
                        sleep(.3)
                else:
                    pass

    # Sound control message for the robot
    def sound_msg(self):
        while not self.stop:
            if self.sound_cmd != None:
                msg = {'append': True,
                        'notes': [star_spangled_banner[5]]}
                self.topics['topic_cmd_audio'].publish(msg)
                sleep(0.1)
                print("sent sound command")

    
    def make_grid(self):
        """This creates a grid message for our robot"""
        grid_width = 200
        grid_height = 200
        resolution = 0.05

        
        if self.room_made == 0:
            self.room_made = 1
            occupancy_grid = [-1] * (grid_width * grid_height)
            robot_pos = [0.0, 0.0]  # x, y position of the robot
            robot_yaw = 0.0  # Robot's yaw (orientation)
            print("Occupancy Grid Created")
        # Robot position and orientation

        angle = self.angle_min

        for r in self.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += self.angle_increment
                continue
        
        # Calculate the x, y coordinates of the point based on the range and angle
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        angle += self.angle_increment
        
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


        occupancy_msg = {
            'header': {
            'frame_id': 'map',
            'stamp': {'secs': int(time.time()), 'nsecs': 0}},
            
            'info': {
            'map_load_time': {'secs': int(time.time()), 'nsecs': 0},
            'resolution': resolution,
            'width': grid_width,
            'height': grid_height,
            'origin': {
                'position': {'x': robot_pos[0], 'y': robot_pos[1], 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': math.radians(robot_yaw), 'w': 1.0}}},
            'data': occupancy_grid}
        
        return(occupancy_msg)
           
    
    def grid_msg(self):
        while not self.stop:
            """This publishes a grid message to our robot"""
            self.topics['topic_map'].publish(roslibpy.Message(self.make_grid()))
            # print(f"Publishing occupancy grid on {self.robot_name}")
            sleep(.1)  # Publish at 10 Hz       

    def clbk_scan(self, msg):
        # print(msg)
        self.angle_min = msg['angle_min']
        self.angle_max = msg['angle_max']
        self.angle_increment = msg['angle_increment']
        self.ranges = msg['ranges']
        return(self.ranges, self.angle_min, self.angle_max, self.angle_increment)
    
    def reset_map(self):
        self.room_map = [[-1 for _ in range(100)] for _ in range(50)]
        print("Room Map Reset")
        return(self.room_map)
    
    def clbk_ir_intensity (self, msg):
        ir_far_left = msg['readings'][0]['value']
        ir_mid_left = msg['readings'][1]['value']
        ir_front_left = msg['readings'][2]['value']
        ir_center = msg['readings'][3]['value']
        ir_front_right = msg['readings'][4]['value']
        ir_mid_right = msg['readings'][5]['value']
        ir_far_right = msg['readings'][6]['value']
        self.ir_values = [ir_far_left, ir_mid_left, ir_front_left, ir_center, ir_front_right, ir_mid_right, ir_far_right]
        return(self.ir_values)

    def clbk_odom(self, msg):
        x_pos = msg['pose']['pose']['position']['x']
        y_pos = msg['pose']['pose']['position']['y']
        z_pos = msg['pose']['pose']['position']['z']
        self.odom_values = [x_pos, y_pos, z_pos]
        self.odom_values_round = [round(x_pos, 2), round(y_pos, 2), round(z_pos, 2)]
        return(self.odom_values)
    
    def clbk_imu(self, msg):
        o = msg.get('orientation')
        x = o.get('x')
        y = o.get('y')
        z = o.get('z')
        w = o.get('w')
        yaw = atan2(2*(x*y+z*w), 1-2 *(y**2 +z**2))
        self.yaw_deg = degrees(yaw)
        return(self.yaw_deg)
    
    def clbk_mode(self, msg):
        mode_msg = msg.get('data')
        if mode_msg == 'AUTO':
            self.manual = False
            self.light_cmd = self.ring_colors[2]
            self.color_butt = 1
            print('\n Robot is in Autonomous Mode')
        elif mode_msg == 'MANUAL':
            self.manual = True
            self.light_cmd = self.ring_colors[3]
            self.color_butt = 1
            print('\n Robot is in Manual Mode')
        return(self.manual)
    
    def reset_pose(self):
        reset_odom_service = roslibpy.Service(self.ros, f'/{self.robot_name}/reset_pose', 'irobot_create_msgs/ResetPose')
        reset_odom_service.call(roslibpy.ServiceRequest())
        self.reset_map()
        print("Pose Reset")

    # this function helps to create the topics for the robot
    def create_topics(self):
        for i in range(len(self.control_name)):
            self.topics['topic_'+ self.control_name[i]] = roslibpy.Topic(self.ros, self.topic_names[i], self.topic_types[i])
        print("Topics Created")
    # this function creates the subscribers for the robot
    def create_subs(self):
        for sub_name in self.sub_name:
            sub_var_name = 'sub_' + sub_name
            callback_name = 'clbk_' + sub_name
            # for each subscriber, create a new topic and subscribe to it
            self.subs[sub_var_name] = roslibpy.Topic(self.ros, f'/{self.robot_name}/{sub_name}', self.sub_types[self.sub_name.index(sub_name)])
            self.subs[sub_var_name].subscribe(getattr(self, callback_name))
            # print(f"Subscribed to {sub_var_name} with callback {callback_name}")
        print("Subscribers Created")
    # This function is used to show the values of the robot every time the alt button is pressed
    def show_subs(self):        
        #print(f'IR: {self.ir_values}') 
        #print(f'Odom: {self.odom_values_round}')
        #print(f'IMU: {round(self.yaw_deg, 2)}')
        #print(f'Vel MSG:: {self.vel_msg}')
        #print(f'MODE: {self.manual}')
        #print(f'Ranges: {self.ranges[1000]}')
        #print(f'Angle Min: {self.angle_min}')
        #print(f'Angle Max: {self.angle_max}')
        #print(f'Angle Increment: {self.angle_increment}')
        print(f'Grid Array: {self.grid_array}')
        # self.reset_map()
        sleep(.1)
    # start the threads function
    def start_threads(self):
        """This method starts our thread"""
        print(f'Starting {len(self.threads)} threads!')
        [t.start() for t in self.threads]
    # end the threads function
    def end_threads(self):
        """This ends / joins all of our threads"""
        print("Joining all threads!")
        [t.join() for t in self.threads]
        print("Ending Program")
        self.stop = True

        # main loop function to run the program
    def run_loop(self):
        self.create_subs()
        self.create_topics()
        # self.reset_pose()
        self.start_threads()


        while self.stop == False:
            self.pygame_events()

        if self.stop == True:
            self.end_threads()

    # function to test the connection to the robot
    def test_connection(self):
        self.ros.run()
        return self.ros.is_connected

    # Main Code that runs the program
if __name__ == '__main__':

    robot = RobotController()
    connect = robot.test_connection()
    print(f'Connection: {connect}')
    if connect == True:
        start_time = time()
        robot.run_loop()
        end_time = time()
        print(f"Total Time = {(end_time - start_time):.3f} seconds")  
    else:
        print("Connection Unsuccessful, Exiting Code")
        exit()
