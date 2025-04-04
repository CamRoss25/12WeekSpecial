# import necessary files
import roslibpy
import roslibpy.actionlib
from time import *
from threading import *
from colors import *
from Star_spangled_banner import star_spangled_banner
import pygame
from pprint import pp
from math import atan2, degrees
from race_3 import RaceVel

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
        self.robot_name = 'india'
        # create robot topics
        self.control_name = ['cmd_vel', 'cmd_lightring', 'cmd_audio']
        self.topic_types = ['geometry_msgs/Twist', 'irobot_create_msgs/LightringLeds', 'irobot_create_msgs/AudioNoteVector']
        self.topic_names = [f'/{self.robot_name}/{c}' for c in self.control_name]
        self.topics = {}

        # create subscribers
        self.sub_name = ['ir_intensity', 'odom', 'imu','mode']
        self.sub_types = ['irobot_create_msgs/IrIntensityVector', 'nav_msgs/Odometry',  'sensor_msgs/Imu','std_msgs/String']
        self.sub_names = [f'/{self.robot_name}/{c}' for c in self.sub_name]
        self.subs = {}

        # self.undock = roslibpy.actionlib.SimpleActionServer(self.ros, '/foxtrot/undock', 'irobot_create_msgs/action/Undock')

        # Define the threads that we want to run
        self.thread_targets = [self.vel_msg, self.light_msg, self.sound_msg]
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
                if self.manual == True:
                    if self.x_cmd or self.turn_cmd != 0:
                        x_vel = round((self.x_cmd / 3), 1)* -1.5
                        turn_vel = round(self.turn_cmd, 1) * -1.5

                        msg = {'linear': {'x': x_vel, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': turn_vel}}
                        sleep(0.1)
                        self.topics['topic_cmd_vel'].publish(msg)
                        # print(f"{msg}")
                else:
                    self.vel_msg = self.velcon.vel_msg(self)
                    self.topics['topic_cmd_vel'].publish(self.vel_msg)
                    sleep(0.05)
                                       
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
        print("Pose Reset")

    # this function helps to create the topics for the robot
    def create_topics(self):
        for i in range(len(self.control_name)):
            self.topics['topic_'+ self.control_name[i]] = roslibpy.Topic(self.ros, self.topic_names[i], self.topic_types[i])
        print("Topics Created")

    def create_subs(self):
        for sub_name in self.sub_name:
            sub_var_name = 'sub_' + sub_name
            callback_name = 'clbk_' + sub_name
            self.subs[sub_var_name] = roslibpy.Topic(self.ros, f'/{self.robot_name}/{sub_name}', self.sub_types[self.sub_name.index(sub_name)])
            self.subs[sub_var_name].subscribe(getattr(self, callback_name))
            print(f"Subscribed to {sub_var_name} with callback {callback_name}")
        print("Subscribers Created")
    
    def show_subs(self):        
        #print(f'IR: {self.ir_values}') 
        #print(f'Odom: {self.odom_values_round}')
        #print(f'IMU: {round(self.yaw_deg, 2)}')
        #print(f'Vel MSG:: {self.vel_msg}')
        print(f'MODE: {self.manual}')
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
