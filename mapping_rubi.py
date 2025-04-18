# Camren is out of town, here is an attempt of mapping while he is out and we have the 2 hour period
# http://192.168.8.104:8080/map.html 

import time
import roslibpy
import math
from random import randint
# import necessary files
import roslibpy
import roslibpy.actionlib
from threading import *
import pygame
from pprint import pp
from math import atan2, degrees

# Connect to rosbridge server
client = roslibpy.Ros(host='192.168.8.104', port=9012)
client.run()

robot_name = 'juliet'
# Create the topic
topic = roslibpy.Topic(client, f'/{robot_name}/mapper', 'nav_msgs/OccupancyGrid')

wide = 100
high = 100
data_x = []
data_y = []

# Define the callback to process incoming messages
def lidar_callback(msg):
    #print('Received LiDAR data:')
    angle_max = msg['angle_max']
    angle_min = msg['angle_min']
    range_min = msg['range_min']
    range_max = msg['range_max']
    inc_angle = msg['angle_increment']
    #print('Angle min:', msg['angle_min'])
    #print('Angle max:', msg['angle_max'])
    #print('Ranges:', msg['ranges'][:5], '...')  # Print only first few for brevity
    a = 0
    for r in msg['ranges']:
        x = r * math.cos(a)
        y = r * math.cos(a)
        px = x/range_max * wide
        py = y/range_max * high
       # print(px,py)
        data_x.append(px)
        data_y.append(py)

        a = a + (inc_angle * 180/math.pi) 

# subscribe to the lidar
lidar_listener = roslibpy.Topic(client, f'/{robot_name}/scan', 'sensor_msgs/LaserScan')
lidar_listener.subscribe(lidar_callback)
print(f"subscribed to {lidar_listener}")



def make_grid():
  # Define a simple 10x10 grid with a diagonal wall
  #width, height = 100, 100
  width, height = wide, high
  resolution = 0.01 # each cell is 0.1m x 0.1m
  datay = []
  datax = []
  data = []

  for y in range(height):
      datay.append(data_y)
      for x in range(width):
          #datax.append(randint(-1,100))  # Random occupancy values between -1 and 100
          datax.append(datax)
  # Create the occupancy grid message
  grid_msg = {
      'header': {
          'frame_id': 'map',
          'stamp': {'secs': int(time.time()), 'nsecs': 0}
      },
      'info': {
          'map_load_time': {'secs': int(time.time()), 'nsecs': 0},
          'resolution': resolution,
          'width': width,
          'height': height,
          'origin': {
              'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
              'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
          }
      },
      'data': datax
  }

  return grid_msg


while client.is_connected:
    topic.publish(roslibpy.Message(make_grid()))
    print(f"Publishing occupancy grid on {topic.name}")
    time.sleep(1)  # Publish at 1 Hz



topic.unadvertise()
client.terminate()
