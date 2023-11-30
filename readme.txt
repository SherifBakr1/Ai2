ROS_Project3 is a reinforcement learning project where a robot learns to follow a right wall using Q-learning. 

Reinforcement Learning for Wall Following
The Python script final2.py demonstrates a reinforcement learning algorithm to train a robot to follow a wall. The robot's behavior is controlled by a Q-learning algorithm, where it learns to take appropriate actions based on its sensory input from a 2D lidar sensor.

Dependencies
ROS (Robot Operating System)
Python libraries: rospy, pandas, numpy, matplotlib
Usage
Ensure that ROS is properly installed on your system.

Make sure the script has executable permissions:

bash
Copy code
chmod +x final2.py

Overview
The script uses Q-learning to train the robot to follow a wall. Q-learning is a model-free reinforcement learning algorithm that learns a policy, which tells an agent what action to take under what circumstances.

The robot receives information from a 2D lidar sensor, and based on this input, it decides its actions. The lidar sensor data is used to create a state space, and the robot learns the optimal actions for each state through Q-learning.

Nodes and Publishers
Nodes

wall_follower_node: The main node running the wall-following algorithm.
Publishers

total_action_taken: Publishes the total number of actions taken during training.
total_action_taken2: Publishes the total number of a different type of actions taken during training.
action_forward_with_turn_little_left: Publishes the ratio of a specific action to the total actions taken.
action_forward: Publishes the ratio of another specific action to the total actions taken.
action_forward_with_turn_little_right: Publishes the ratio of yet another specific action to the total actions taken.
action_forward_with_hard_left: Publishes the ratio of an additional specific action to the total actions taken.
action_forward_with_hard_right: Publishes the ratio of another additional specific action to the total actions taken.
episode: Publishes the current episode number.
Q-learning Parameters
MAX_EPISODE: Maximum number of training episodes.
GAMMA: Discount rate for future rewards.
ALPHA: Learning rate.
EPSILON: Exploration-exploitation trade-off parameter.
Q-learning Algorithm
The Q-learning algorithm is implemented in the Qlearn function. The robot explores the environment, updating its Q-table based on the rewards received for each action. The Q-table is used to make decisions on which actions to take in different states.

Sensor Data Processing
The laserscan_callback function processes data from the 2D lidar sensor. It extracts information about the distances in the front, right, left, and corner directions. The script defines distance thresholds to categorize the proximity of obstacles, which are used to determine the states of the robot.

Actions and Movement
The robot can take five different actions: FORWARD_AND_TURN_LITTLE_LEFT, FORWARD, FORWARD_AND_TURN_LITTLE_RIGHT, FORWARD_AND_HARD_LEFT, and FORWARD_AND_HARD_RIGHT. The move_robot_by_action function controls the robot's movement based on the chosen action.

Training Visualization
The script visualizes the training progress by plotting the total accumulated reward over episodes using Matplotlib. The plot updates dynamically during training.

Saving Q-table
After each episode, the Q-table is saved to a CSV file named Q_Table_episode_<episode_number>.csv in the specified directory.

Feel free to adjust the parameters and thresholds in the script for your specific environment and robot characteristics. 

The file final2test.py is for the test mode, of testing the best Q-table obtained during training.
Q-Table Loading:
csv_File_path = "998.csv"
final_q_table = pd.read_csv(csv_File_path, index_col=0)
q_table = final_q_table

Load the pre-trained Q-table from a CSV file. Adjust the file path based on your Q-table location.

Test Mode Execution:
q_table = Qlearn()

Make sure (in a different terminal) you load the gazebo environment prior to running the code to be able to view the robot as its moving.
Make sure the 998.csv file is in the same directory as final2test.py, and, of course, run the command roslaunch ros_project3 wallfollow.launch to run the environment, and then, in another terminal, run the final2test.py file!
