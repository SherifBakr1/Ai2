#!/usr/bin/env python3
import rospy
#import pandas as pd
#import numpy as np
import time
import random
import math
#import matplotlib.pyplot as plt
import rospkg
import csv

from std_msgs.msg import Float32, Int64
from geometry_msgs.msg import Twist         #cmd_vel publisher
from sensor_msgs.msg import LaserScan       #2d lidar data
from std_srvs.srv import Empty              #reset gazebo world
#from gazebo_msgs.msg import ModelState
#from gazebo_msgs.srv import SetModelState

rospack = rospkg.RosPack()
rospack_path = rospack.get_path("ros_project3")

total_action_taken_pub                      = rospy.Publisher('total_action_taken', Int64, queue_size=1)
total_action_taken2_pub                     = rospy.Publisher('total_action_taken2', Int64, queue_size=1)
action_forward_with_turn_little_left_pub    = rospy.Publisher('action_forward_with_turn_little_left', Float32, queue_size=1)
action_forward_pub                          = rospy.Publisher('action_forward', Float32, queue_size=1)
action_forward_with_turn_little_right_pub   = rospy.Publisher('action_forward_with_turn_little_right', Float32, queue_size=1)
action_forward_with_hard_left               = rospy.Publisher('action_forward_with_hard_left', Float32, queue_size=1)
action_forward_with_hard_right              = rospy.Publisher('action_forward_with_hard_right', Float32, queue_size=1)
episode_pub                                 = rospy.Publisher('episode', Int64, queue_size=1)

cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)      

N_STATES = 26
N_ACTIONS = 5
print("no of states received")
# Initialize all variables
range_front = []
range_right = []
range_left  = []
range_corner = []
min_front = 0
i_front = 0
min_right = 0
i_right = 0
min_left = 0
#i_left = 0
front = ""
left = ""
right = ""
state = 0

ACTIONS     = ['FORWARD_AND_TURN_LITTLE_LEFT', 'FORWARD', 'FORWARD_AND_TURN_LITTLE_RIGHT', 'FORWARD_AND_HARD_LEFT', 'FORWARD_AND_HARD_RIGHT']
    
MAX_EPISODE = 1200   # DURING THE DEVELOPMENT/IMPLEMENTATION PLEASE FEEL FREE TO CHANGE THIS VALUE
GAMMA       = 0.8   # discount rate
ALPHA       = 0.1  # (1-alpha)
EPSILON     = 0.9

episode = 0
#global epsilon
#epsilon = 0.9
#max_epsilon = EPSILON
#min_epsilon = 0.001       
#decay = 0.015         

total_training_rewards = 0
training_rewards = []  
epsilons = []
total_action_taken = 0
total_action_taken2 =0
forward_with_turn_little_left       = 0
forward                             = 0
forward_with_turn_little_right      = 0
forward_and_hard_left                       = 0
forward_and_hard_right                      = 0

ratio_forward_action_taken                  = []  
ratio_forward_lil_left_action_taken          = []  
ratio_forward_lil_right_action_taken          = []  
ratio_forward_and_hard_left                   = []  
ratio_forward_and_hard_right                   = []  
ratio_forward_and_V_hard_right                   = []  
ratio_forward_and_V_hard_left                   = []  

def actor(observation, q_table):

    state_action = q_table[observation]
    action = random.choice([key for key, value in state_action.items() if value == max(state_action.values())])
    return action

def move_robot(speed_linear_x, speed_angular_z):
    msg = Twist()
    msg.linear.x = speed_linear_x
    msg.angular.z = speed_angular_z
    cmd_vel_pub.publish(msg)
  
def move_robot_by_action(action):
    if action == 'FORWARD_AND_TURN_LITTLE_LEFT':
        move_robot(0.15, 0.20)
        rospy.sleep(0.5)
    elif action == 'FORWARD':
        move_robot(0.15, 0.0)
        rospy.sleep(0.5)
    elif action == 'FORWARD_AND_TURN_LITTLE_RIGHT':
        move_robot(0.15, -0.20)
        rospy.sleep(0.5)
    elif action == 'FORWARD_AND_HARD_LEFT':
        move_robot(0.15, 0.60)
        rospy.sleep(0.5)
    elif action == 'FORWARD_AND_HARD_RIGHT':
        move_robot(0.15, -0.60)
        rospy.sleep(0.5)

def stopRobot():
    move_robot(0.0,0.0)
    rospy.sleep(1)
def stopRobot2():
    move_robot(0.0,0.0)
    rospy.sleep(0.1)
def publish_all():
    global episode
    global total_action_taken
    global total_action_taken2
    global forward_with_turn_little_left
    global forward
    global forward_with_turn_little_right
    global forward_and_hard_left
    global forward_and_hard_right

    episode_msg = Int64()
    episode_msg.data = episode
    episode_pub.publish(episode_msg)

    action_msg = Int64()
    action_msg.data = total_action_taken
    total_action_taken_pub.publish(action_msg)

    action_msg = Int64()
    action_msg.data = total_action_taken2
    total_action_taken2_pub.publish(action_msg)

    msg = Float32()
    if total_action_taken!=0:
        msg.data = forward_with_turn_little_left        / total_action_taken
        action_forward_with_turn_little_left_pub.publish(msg)
        msg.data = forward                              / total_action_taken
        action_forward_pub.publish(msg)
        msg.data = forward_with_turn_little_right       / total_action_taken
        action_forward_with_turn_little_right_pub.publish(msg)
        msg.data = forward_and_hard_left                      / total_action_taken
        action_forward_with_hard_left.publish(msg)
        msg.data = forward_and_hard_right                       / total_action_taken
        action_forward_with_hard_right.publish(msg)
    
    if total_action_taken2!=0:
        #msg.data = forward_with_turn_little_left        / total_action_taken2
        #action_forward_with_turn_little_left_pub.publish(msg)
        msg.data = forward                              / total_action_taken2
        action_forward_pub.publish(msg)
        #msg.data = forward_with_turn_little_right       / total_action_taken2
        #action_forward_with_turn_little_right_pub.publish(msg)
        #msg.data = forward_and_hard_left                      / total_action_taken2
        #action_forward_with_hard_left.publish(msg)
        #msg.data = forward_and_hard_right                       / total_action_taken2
        #action_forward_with_hard_right.publish(msg)



    
def get_state(front, right):
    global state
    global range_corner
    global range_right
    global range_front
    end = False
    if (range_corner == 0.0):
        range_corner= 1.0


    if (right == "vclose" and front =="close" and range_corner <0.7 ):
        state = 0 
        reward = -100
        end = True
        return state, reward, end
    if (right == "vclose" and front =="normal" and range_corner <0.7 ):
        state = 1
        reward = -100
        end = True
        return state, reward, end
    if (right == "vclose" and front =="far" and range_corner <0.7 ):
        state = 2 
        reward = -100
        end = True
        return state, reward, end 
    elif (right == "close" and front == "close" and range_corner <0.7 ):
        state = 3
        reward = -100
        end = True
        return state, reward, end
    
    elif (right == "close" and front == "normal" and range_corner <0.7 ):
        state = 4
    elif (right == "close" and front == "far" and range_corner <0.7 ):
        state = 5
        #rewarded in main loop
    elif (right == "normal0" and front == "close" and range_corner <0.7 ):
        state = 6
        reward = -100
        end = True
        return state, reward, end
    elif (right == "normal0" and front == "normal" and range_corner <0.7 ):
        state = 7
    elif (right == "normal0" and front == "far" and range_corner <0.7):
        state = 8
    elif (right == "normal1" and front == "close" and range_corner <0.7 ):
        state = 9
        reward = -100
        end = True  
        return state, reward, end  
    elif (right == "normal1" and front == "normal" and range_corner <0.7 ):
        state = 10
    elif (right == "normal1" and front == "far" and range_corner <0.7 ):
        state = 11

    elif (right == "far" and front == "close" and range_corner <0.7 ):
        state = 12
        reward = -100
        end = True  
        return state, reward, end  
    elif (right == "far" and front == "normal" and range_corner <0.7 ):
        state = 13
        reward = -100
        end = True  
        return state, reward, end 
    elif (right == "far" and front == "far" and range_corner <0.7):
        state = 14
        reward = -100
        end = True  
        return state, reward, end 

    elif (right == "far" and front == "close" and range_corner>0.7):
        state = 15
        reward = -100
        end = True  
        return state, reward, end  
    elif (right == "far" and front == "normal" and range_corner>0.7):
        state = 16
        reward = -100
        end = True  
        return state, reward, end 
    elif (right == "far" and front == "far" and range_corner>0.7):
        state = 17
        reward = -100
        end = True  
        return state, reward, end 
    
    elif (right == "normal1" and front == "close" and range_corner > 0.7):
        state = 18
        reward = -100
        end = True  
        return state, reward, end  
    elif (right == "normal1" and front == "normal" and range_corner >0.7):
        state = 19
    elif (right == "normal1" and front == "far" and range_corner > 0.7):
        state = 20

    elif (right == "normal0" and front == "close" and range_corner > 0.7):
        state = 21
        reward = -100
        end = True
        return state, reward, end
    elif (right == "normal0" and front == "normal" and range_corner > 0.7):
        state = 22
    elif (right == "normal0" and front == "far" and range_corner > 0.7):
        state = 23

    elif (right == "close" and front == "normal" and range_corner > 0.7):
        state = 24

    elif (right == "close" and front == "far" and range_corner > 0.7):
        state = 25
    
    
    reward= 0
    return state, reward, end

def get_state_initial(front, right):
    global range_corner
    global range_front
    global range_right
    global state              
    if (range_corner ==0.0):
        range_corner = 1.0

    if (right == "vclose" and front =="close" and range_corner <0.7):
        state = 0 
    if (right == "vclose" and front =="normal" and range_corner <0.7):
        state = 1
    if (right == "vclose" and front =="far" and range_corner <0.7):
        state = 2 
    elif (right == "close" and front == "close" and range_corner <0.7):
        state = 3
    elif (right == "close" and front == "normal" and range_corner <0.7):
        state = 4
    elif (right == "close" and front == "far" and range_corner <0.7):
        state = 5
    elif (right == "normal0" and front == "close" and range_corner <0.7):
        state = 6
    elif (right == "normal0" and front == "normal" and range_corner <0.7):
        state = 7
    elif (right == "normal0" and front == "far" and range_corner <0.7):
        state = 8
    elif (right == "normal1" and front == "close" and range_corner <0.7):
        state = 9  
    elif (right == "normal1" and front == "normal" and range_corner <0.7):
        state = 10
    elif (right == "normal1" and front == "far" and range_corner <0.7):
        state = 11
    elif (right == "far" and front == "close" and range_corner <0.7):
        state = 12
    elif (right == "far" and front == "normal" and range_corner <0.7):
        state = 13
    elif (right == "far" and front == "far" and range_corner <0.7):
        state = 14
    elif (right == "far" and front == "close" and range_corner>0.7):
        state = 15  
    elif (right == "far" and front == "normal" and range_corner>0.7):
        state = 16
    elif (right == "far" and front == "far" and range_corner>0.7):
        state = 17
    elif (right == "normal1" and front == "close" and range_corner > 0.7):
        state = 18 
    elif (right == "normal1" and front == "normal" and range_corner >0.7):
        state = 19
    elif (right == "normal1" and front == "far" and range_corner > 0.7):
        state = 20
    elif (right == "normal0" and front == "close" and range_corner > 0.7):
        state = 21
    elif (right == "normal0" and front == "normal" and range_corner > 0.7):
        state = 22
    elif (right == "normal0" and front == "far" and range_corner > 0.7):
        state = 23
    elif (right == "close" and front == "normal" and range_corner > 0.7):
        state = 24
    elif (right == "close" and front == "far" and range_corner > 0.7):
        state = 25
    return state
 

### Qlearning algoritm
def Qlearn():
    global steps
    steps=0
    global left
    global front
    global right
    global total_training_rewards
    global total_action_taken
    global total_action_taken2

    global episode
    global state
    global f
    f=0
    global l
    l=0

    csv_File_path= "998.csv"
    try:
        with open(csv_File_path, mode='r') as file:
            reader = csv.reader(file)
            q_table = {int(rows[0]): {action: float(value) for action, value in zip(ACTIONS, rows[1:])} for rows in reader if rows[0].isdigit()}
    except FileNotFoundError:
        print("file not found")
        return
    episode = 801

    while episode < (MAX_EPISODE):
        print("entered while loop")
        if rospy.is_shutdown():
            break
        print("Before stopping")
        stopRobot()
        print("After stopping")

        
        total_training_rewards = 0
        end =False
        
        state = get_state_initial(front, right)     
        print("THE VERY FIRST STATE IS", state) 
        steps=0                
        while not end and not rospy.is_shutdown():

            print("STEPS", steps)
            steps+=1
            #print("EPSILON IS", epsilon)
            act = actor(state, q_table)
            move_robot_by_action(act)
            next_state, reward, end = get_state(front, right)

            print("Next State: ", next_state)
            
            if ((state==4) and range_corner <1.5):
                total_action_taken += 1
                if act == 'FORWARD_AND_HARD_LEFT':
                    l+=1
                    reward+=10
                if act == 'FORWARD_AND_HARD_RIGHT':
                    reward-=10

            if ((state==7 or state ==10) and range_corner <1.5):
                if act == 'FORWARD_AND_HARD_LEFT':
                    reward-=10
                if act == 'FORWARD_AND_TURN_LITTLE_LEFT':
                    reward+=10
                
            if (state==5 and range_corner<1.5):
                total_action_taken2 +=1
                if act == 'FORWARD':
                    f+=1
                    reward+=10
            
            #For u-turn
            if ((state==5 or state==8 or state==11) and (range_corner>1.5) and (act =='FORWARD_AND_HARD_LEFT' or act == 'FORWARD_AND_TURN_LITTLE_LEFT' )):
                 reward-=10

            state = next_state

            total_training_rewards += reward

        #epsilon = 0.01 + (0.99 - 0.01)*np.exp(-0.01*(episode))
        #training_rewards.append(total_training_rewards)
        
        #if total_action_taken != 0:
        #    ratio_forward_and_V_hard_left.append(l/total_action_taken)
        #    epsilons.append(epsilon)

        if total_action_taken2 !=0:
            ratio_forward_action_taken.append(f/total_action_taken2)
            
        episode = episode +1
        print("The episode we're currently in is episode: ", episode)

    return q_table

def laserscan_callback(msg):
    global range_front
    global range_right
    global range_left
    global range_corner
    global ranges
    global min_front,i_front, min_right,i_right, min_left, i_left
    
    global front
    global right
    global left
    ranges = msg.ranges
    # You changed the range of the right messages from 275:276 to what it currently is for training on u-turn
    range_front = min(msg.ranges[0:1]) # Front2 FOV (between 5 to -5 degrees)
    if range_front == 0:
        range_front= 1.0

    range_right = min(msg.ranges[260:280])  # right FOV (between 300 to 345 degrees)
    if range_right == 0:
        range_right == 1.0
    range_left=min(msg.ranges[85:95])
    range_corner = min(msg.ranges[300:302])
    if range_corner == 0:
        range_corner =1.0
    
    # find the shortest obstacle of each side 
    #min_front,i_front = min( (range_front[i_front],i_front) for i_front in range(len(range_front)) )
    #min_right,i_right = min( (range_right[i_right],i_right) for i_right in range(len(range_right)) )
    #min_left,i_left = min( (range_left[i_left],i_left) for i_left in range(len(range_left)) )
# HIT_DISTANCE_THRESHOLD = 0.18          
# VCLOSE_THRESHOLD= 0.30
# CLOSE_THRESHOLD = 0.42
# FAR_THRESHOLD=0.54  
# #The below was 0.65 before running the left turn training.
# LOST_DISTANCE_THRESHOLD =0.7    
    # front states
    if (range_front< 0.3):
        front= "close"
    elif (0.3 <range_front < 0.65):
        front = "normal"
    elif (0.65 < range_front):
        front = "far"
      
    if (range_right <0.10):
        right ="vclose"  
    elif ( 0.10<range_right< 0.35):
        right= "close"
    elif (0.35 <range_right < 0.4):
        right = "normal0"
    elif (0.4 <range_right < 0.65):
        right = "normal1"
    elif (0.65 < range_right ):
        right = "far"



if __name__ == "__main__":
    rospy.init_node('wall_follower_node', anonymous=True)

    #rospy.wait_for_service("/gazebo/reset_world")
    #rospy.wait_for_service('/gazebo/set_model_state')

    #gazebo_reset_world_srv = rospy.ServiceProxy("/gazebo/reset_world", Empty)
    #gazebo_set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    
    rospy.Subscriber("/scan", LaserScan, laserscan_callback, queue_size=1)

    q_table = Qlearn()
    # rospy.spin()
    print("====== Q TABLE AFTER LEARNING ======")
    # print(q_table)
    print(" ")
