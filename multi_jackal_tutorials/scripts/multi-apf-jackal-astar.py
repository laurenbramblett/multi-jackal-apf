#!/usr/bin/env python3
"""Control multiple jackals using APF"""

from potential_field_class import PotentialField, get_model_pose
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String


real_robot = False

# Define the positions
global pos, ori, paths, pos_received
pos = [np.array([0., 0.]), np.array([0., 0.])]
pos_received = False
ori = [0., 0.]
paths = []

# Callback function for the subscriber to get relative pose information (for real robot)
def pos_cb(msg, args):
    global pos, ori, pos_received
    pos[args]    = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
    ori[args]    = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
    pos_received = True

def path_cb(msg):
    global paths
    paths = eval(msg.data)    
    print(paths)

# Initialize the dictionary for the environment
string_msg = {
    'numAgents': 2,
    'numCities': 5,
    'startPose': [[0,0],[0,1]],
    'vels': [1,1],
    'cityCoordinates': [[2,3],[2,5],[-2,-0.5],[14,3],[-10,2]],
    'numGenerations': 10,
    'populationSize': 10,
    'mutationRate': 0.1,
    'new_run': True
}

# Initialize dictionary
dict_init = string_msg

if real_robot:
    #TODO_CAPSTONE: change the topic names to be relative to the turtlebot
    sub_names = ['jackal0/odometry/local_filtered', 'jackal1/odometry/local_filtered']
    pub_names = ['jackal0/jackal_velocity_controller/cmd_vel', 'jackal1/jackal_velocity_controller/cmd_vel']
    # Create the subscribers
    subs = [rospy.Subscriber(sub_names[i], Odometry, pos_cb, (i)) for i in range(len(sub_names))]
else:
    sub_names = ['jackal0', 'jackal1']
    pub_names = ['jackal0/jackal_velocity_controller/cmd_vel', 'jackal1/jackal_velocity_controller/cmd_vel']


if __name__ == '__main__':

    # Initialize the node
    rospy.init_node('multi_apf_jackal')

    # Create the path callback
    rospy.Subscriber('best_paths', String, path_cb)

    # Create the velocity publishers
    pubs = [rospy.Publisher(pub_names[i], Twist, queue_size=1) for i in range(len(pub_names))]

    # Publisher for the astar path planner
    template_pub = rospy.Publisher('string_msg', String, queue_size=1)

    # Create the potential fields in a loop
    fields = [PotentialField(kappa_attr=1, kappa_rep_obs=10, kappa_rep_veh=1.5, d0=2.0, d1=2.0) for i in range(len(sub_names))]

    # Create the rate
    rate = rospy.Rate(10)

    #Initialize velocities
    linear_velocity  = [0, 0]
    angular_velocity = [0, 0]
    path_counter     = [0, 0]

    # Main loop
    while not rospy.is_shutdown():
        # Check if best paths is empty
        if pos_received and real_robot:
            # Create the message
            dict_init['startPose'] = [[pos[i][0], pos[i][1]] for i in range(len(sub_names))]
            dict_msg = str(dict_init)
            # Publish the message
            template_pub.publish(dict_msg)
        elif not real_robot and len(paths) < 1:
            # Get the positions from gazebo
            pos_hold = [get_model_pose(sub_names[i]) for i in range(len(sub_names))]
            pos  = [np.round([pos_hold[i].position.x, pos_hold[i].position.y]) for i in range(len(sub_names))]
            # Create the message
            dict_init['startPose'] = [[pos[i][0], pos[i][1]] for i in range(len(sub_names))]
            dict_msg = str(dict_init)
            print(dict_msg)
            # Publish the message
            template_pub.publish(dict_msg)

        # Check if the paths are published by astar node
        if len(paths) < 1:
            rate.sleep()
            continue
        for i in range(len(sub_names)):
            if path_counter[i] >= len(paths[i]):
                goal = np.array(dict_init['startPose'][i])
            else:
                goal = np.array(paths[i][path_counter[i]])

            if not real_robot:
                pos_hold = get_model_pose(sub_names[i])
                pos[i]  = np.array([pos_hold.position.x, pos_hold.position.y])
                # Change orientation from quaternion to euler
                ori[i] = euler_from_quaternion([pos_hold.orientation.x, pos_hold.orientation.y, pos_hold.orientation.z, pos_hold.orientation.w])[2]
            
            # Get the velocities   
            max_speed = dict_init['vels'][i]
            if not real_robot:
                linear_velocity[i], angular_velocity[i] = fields[i].get_velocities(pos[i], ori[i], goal, [], [pos[j] for j in range(len(sub_names)) if j != i], max_speed)
            else: 
                #TODO: transform other vehicle positions to be relative to the current vehicle
                linear_velocity[i], angular_velocity[i] = fields[i].get_velocities(pos[i], ori[i], goal, [], [], max_speed)

            # Check if the goal has been reached
            if np.linalg.norm(pos[i] - goal) < 0.3:
                path_counter[i] += 1

            # Print the velocities, goal, and position
            print("--------------------")
            print("Vehicle: ", i)
            print("Linear velocity: ", linear_velocity[i])
            print("Angular velocity: ", angular_velocity[i])
            print("Goal: ", goal)
            print("Position: ", pos[i])
            print("--------------------")

        # Check if all goals have been reached
        if all([path_counter[i] >= len(paths[i]) for i in range(len(sub_names))]):
            if all([np.linalg.norm(pos[i] - np.array(dict_init['startPose'][i])) < 0.1 for i in range(len(sub_names))]):
                break

        # Create the message
        for i in range(len(sub_names)):
            msg = Twist()
            msg.linear.x = linear_velocity[i]
            msg.angular.z = angular_velocity[i]
            # Publish the message
            pubs[i].publish(msg)
        
        # Sleep
        rate.sleep()