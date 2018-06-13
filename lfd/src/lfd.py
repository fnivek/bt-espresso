#!/usr/bin/env python
import select
import sys

from sklearn.svm import LinearSVC
import numpy

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
states = None
actions = None

# A model that takes in a state and produces an action
def model(state):
    return 'h'

# Produces a model from states
def learn(states, actions):
    clf = LinearSVC()
    clf.fit(states, actions)
    return clf.predict

def detect_centroid():
    # Send a goal to centroid detector
    rospy.loginfo('Detecting centroid')
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    # Convert result to a numpy array
    result = numpy.array([
        [
            result.centroid.position.x,
            result.centroid.position.y,
            result.centroid.position.z,
            result.centroid.orientation.x,
            result.centroid.orientation.y,
            result.centroid.orientation.z,
            result.centroid.orientation.w,
            result.success
        ]
    ])
    return result

def execute():
    state = detect_centroid()
    print 'World state is:\n' + str(state)
    print 'Resulting action is: ' + model(state)[0]

def main():
    global model
    global states
    global actions
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
            model = learn(states, actions)
            state = 'AskUser'
        elif state == 'Demonstrate':
            print 'Demonstrate'
            result = detect_centroid()
            print 'World state is\n' + str(result)
            user_input = numpy.array([[raw_input('What action should be taken; say hi (h) or nop (n):')]])
            if states is not None:
                states = numpy.append(states, result, axis=0)
                actions = numpy.append(actions, user_input)
            else:
                states = result
                actions = user_input
            state = 'AskUser'

        # Sleep if needed
        sleep_rate.sleep()

if __name__ == '__main__':
    main()
