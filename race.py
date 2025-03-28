from time import *
from math import *

class RaceVel:

    def __init__(self, robot):
        # Initialize PID constants and other parameters
        self.head_kp = 0.005        # Heading Proportional gain
        self.head_ki = 0.0001      # Heading Integral gain
        self.head_kd = 0.001         # Heading Derivative gain
        self.head_prev_error = 0.0  # Previous heading error for derivative term
        self.head_integral = 0.0    # Integral heading sum
        self.desired_heading = 0
        self.head_error = 1
        self.robot = robot

        self.vel_kp = .6           # Velocity Proportional gain
        self.vel_ki = 0.005          # Velocity Integral gain
        self.vel_kd = .05          # Velocity Derivative gain
        self.vel_prev_error = 0.0   # Previous velocity error for derivative term
        self.vel_integral = 0.0     # Integral velocity sum

        self.head_max_output = 2  # Maximum allowable heading output
        self.head_min_output = -2 # Minimum allowable heading output
        self.vel_turn_max_output = 0.1
        self.vel_max_output = 0.3
        self.vel_min_output = -0.3

        self.start = True
        self.time = False
        self.start_time = time()

        # Initialize Values
        self.x_vel = 0
        self.turn_vel = 0
        self.desired_heading = 0
        self.current_heading = robot.yaw_deg
        self.current_position = robot.odom_values
        self.current_velocity = 0
        self.ir_prev_error = 0

        self.ir_values = robot.ir_values
        self.ir_values_0 = [0,0,0,0,0,0,0]
        self.ir_values_1 = [0,0,0,0,0,0,0]
        self.ir_values_2 = [0,0,0,0,0,0,0]
        self.ir_values_3 = [0,0,0,0,0,0,0]
        self.ir_values_4 = [0,0,0,0,0,0,0]

        self.ir_call = robot.ir_call

    def turn_compute(self, ir_call):
        
        ir_prev_error = self.ir_prev_error

        # Compute the desired heading based on the current position and the desired position
        current_ir = self.ir_values
        # print(f"Current Position: [{round(current_pos[0], 2)}, {round(current_pos[1], 2)}]")
        # print(f"Current Heading : {round(current_heading, 2)}")

        current_far_left, current_mid_left, current_front_left, current_front_right, current_mid_right, current_far_right = current_ir[0], current_ir[1], current_ir[2], current_ir[4], current_ir[5],  current_ir[6]

        #current_left = current_far_left
        current_left = max(current_far_left, current_mid_left, current_front_left)
        current_right = max(current_mid_right, current_far_right, current_front_right)

        desired_IR = 100

        # Switch between left and right wall following control
        if ir_call == -1:
            ir_error = desired_IR - current_left
        if ir_call == 1:
            ir_error = current_right - desired_IR
        if ir_call == 0:
            ir_error = current_right - current_left
        
        # Proportional term
        p_term = self.head_kp * ir_error
        
        # Integral term (accumulated error)
        self.head_integral += ir_error
        i_term = self.head_ki * self.head_integral
        
        # Derivative term (rate of change of error)
        d_term = self.head_kd * (ir_error - ir_prev_error)
        
        # Update previous error
        self.ir_prev_error = ir_error
        
        # Compute the total PID output
        output = p_term + i_term + d_term
        
        # restrict the output to the defined bounds

        output = round(max(self.head_min_output, min(self.head_max_output, output)), 2)
        
        if output >= 1 or output <= -1:
            self.turning = True
        else:
            self.turning = False

        return output
    
    def velocity_compute(self):

        ir_desired = 100
        ir_values = self.ir_values
        ir_error_left = ir_desired - ir_values[2]
        ir_error_center = ir_desired - ir_values[3]
        ir_error_right = ir_desired - ir_values[4]

        ir_error = min(ir_error_left, ir_error_center, ir_error_right)


        # Calculate the desired velocity based on the ir error
        desired_velocity = ir_error * self.vel_kp

        # Compute the PID output for the current velocity
        velocity_error = desired_velocity - self.current_velocity

        # Proportional term
        p_term = self.vel_kp * velocity_error
        
        # Integral term (accumulated error)
        self.vel_integral += velocity_error
        i_term = self.vel_ki * self.vel_integral
        
        # Derivative term (rate of change of error)
        d_term = self.vel_kd * (velocity_error - self.vel_prev_error)
        
        # Update previous error
        self.vel_prev_error = velocity_error
        
        # Compute the total PID output
        output = p_term + i_term + d_term

        # Adjust output based on distance error
        output += self.vel_kp * ir_error
        
        # Clamp the output to the defined bounds
        output = round(max(self.vel_min_output, min(self.vel_max_output, output)), 2)
        if output < 0:
            output = 0

        self.current_velocity = output

        return output
    
    def update_robot(self, robot):
        # get the robots current status
        self.current_heading = robot.yaw_deg
        self.current_position = robot.odom_values
        self.ir_call = robot.ir_call

        # Age the IR values and take the average of the last 5 to get the best current IR values
        self.ir_values_4 = self.ir_values_3
        self.ir_values_3 = self.ir_values_2
        self.ir_values_2 = self.ir_values_1
        self.ir_values_1 = self.ir_values_0
        self.ir_values_0 = robot.ir_values
        list_avg = [sum(x)/len(x) for x in zip(self.ir_values_4, self.ir_values_3, self.ir_values_2, self.ir_values_1, self.ir_values_0)]
        self.ir_values = list_avg

    def vel_msg(self, robot):
        """This publishes a velocity message to our robot"""

        self.update_robot(robot)

        self.turn_vel = self.turn_compute(self.ir_call)
        self.x_vel = self.velocity_compute()

        msg = {'linear': {'x': self.x_vel, 'y': 0.0, 'z': 0.0},
                    'angular': {'x': 0.0, 'y': 0.0, 'z': self.turn_vel}}
        return msg
                            
