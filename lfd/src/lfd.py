#!/usr/bin/env python
import select
import sys

import rospy
import actionlib

from centroid_detector_msgs.msg import DetectCentroidGoal, DetectCentroidAction

# Global vars
# Construct a goal for the centroid detector
goal = DetectCentroidGoal()
goal.min_x = 0
goal.max_x = 1.5
goal.min_y = -0.5
goal.max_y = 0.5
goal.min_z = 0.6
goal.max_z = 2
client = actionlib.SimpleActionClient('/centroid_detector', DetectCentroidAction)
demos = []

# A model that takes in a state and produces an action
def model(state):
    return 'h'

# Produces a model from demos
def learn(demos):
    for demo in demos:
        h = 0
        n = 0
        if demo[1] == 'h':
            h += 1
        else:
            n += 1
    if h >= n:
        return lambda state: 'h'
    else:
        return lambda state: 'n'

def detect_centroid():
    # Send a goal to centroid detector
    rospy.loginfo('Detecting centroid')
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    return result

def execute():
    state = detect_centroid()
    print model(state)

def main():
    global model
    # Initilise the node
    rospy.init_node('lfd')

    # Define an action client for centroid detector
    rospy.loginfo('Waiting for centroid detector action server')
    client.wait_for_server()
    rospy.loginfo('Centroid detector action server is running')

    # State
    state = 'AskUser'

    # Loop every 10hz
    sleep_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if state == 'AskUser':
            print 'Demonstrate(d), learn(l), execute(e):'
            state = 'WaitForUser'
        elif state == 'WaitForUser':
            if select.select([sys.stdin,], [], [], 0.0)[0]:
                user_input = raw_input()
                if user_input == 'd':
                    state = 'Demonstrate'
                elif user_input == 'l':
                    state = 'Learn'
                elif user_input == 'e':
                    state = 'Execute'
                else:
                    state = 'AskUser'
        elif state == 'Execute':
            print 'Execute'
            execute()
            state = 'AskUser'
        elif state == 'Learn':
            print 'Learning'
            model = learn(demos)
            state = 'AskUser'
        elif state == 'Demonstrate':
            print 'Demonstrate'
            result = detect_centroid()
            print 'World state is\n' + str(result)
            user_input = raw_input('What action should be taken; say hi (h) or nop (n):')
            demos.append((result, user_input))
            state = 'AskUser'

        # Sleep if needed
        sleep_rate.sleep()

if __name__ == '__main__':
    main()
