#!/usr/bin/env python3
"""Control multiple jackals using APF"""

from potential_field_class import get_model_pose
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
import actionlib


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

def publish_goal(goal,pub,agent):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = goal[0]
    msg.pose.position.y = goal[1]
    msg.pose.orientation.w = 1.0
    pub.publish(msg)
    print("Published goal for agent {}".format(agent))


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
    pub_names = ['jackal0/move_base_simple/goal', 'jackal1/move_base_simple/goal']


if __name__ == '__main__':

    # Initialize the node
    rospy.init_node('multi_apf_jackal')

    # Create the path callback
    rospy.Subscriber('assigned_tasks', String, path_cb)

    # Create the goal publishers
    pubs = [rospy.Publisher(pub_names[i], PoseStamped, queue_size=1) for i in range(len(pub_names))]
    pub_cancel_goal = [rospy.Publisher(sub_names[i] + 'move_base/cancel', GoalID, queue_size=1) for i in range(len(sub_names))]
    # Publisher for the astar path planner
    template_pub = rospy.Publisher('string_msg', String, queue_size=1,latch=True)

    # Create the potential fields in a loop
    # fields = [PotentialField(kappa_attr=1, kappa_rep_obs=10, kappa_rep_veh=1.5, d0=2.0, d1=2.0) for i in range(len(sub_names))]

    # Create the rate
    rate = rospy.Rate(10)

    #Initialize velocities
    linear_velocity  = [0, 0]
    angular_velocity = [0, 0]
    path_counter     = [0, 0]
    iter             = 0
    new_goal         = [True, True] 

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
            if iter == 0:
                dict_init['startPose'] = [[pos[i][0], pos[i][1]] for i in range(len(sub_names))]
                dict_msg = str(dict_init)
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
                goal = np.array(dict_init['cityCoordinates'][paths[i][path_counter[i]]])
            
            if new_goal[i]:
                publish_goal(goal,pubs[i],i)
                new_goal[i] = False

            if not real_robot:
                pos_hold = get_model_pose(sub_names[i])
                pos[i]  = np.array([pos_hold.position.x, pos_hold.position.y])
            
            # Check if the goal has been reached
            if np.linalg.norm(pos[i] - goal) < 0.5:
                path_counter[i] += 1
                new_goal[i] = True
                # Cancel the goal
                cancel_msg = GoalID()
                pub_cancel_goal[i].publish(cancel_msg)
                

            # Print the velocities, goal, and position
            print("--------------------")
            print("Vehicle: ", i)
            print("Goal: ", goal)
            print("Position: ", pos[i])
            print("--------------------")

        # Check if all goals have been reached
        if all([path_counter[i] >= len(paths[i]) for i in range(len(sub_names))]):
            if all([np.linalg.norm(pos[i] - np.array(dict_init['startPose'][i])) < 0.1 for i in range(len(sub_names))]):
                break
        
        # Sleep
        rate.sleep()