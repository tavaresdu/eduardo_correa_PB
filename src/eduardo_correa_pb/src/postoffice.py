#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseActionResult
from tf.transformations import quaternion_from_euler
from time import sleep
import math

caminho = False

postoffice = [-1, -2, -135]
houses = {
    '1': [1, -2, -45],
    '2': [1, 2, 45],
    '3': [-1, 2, 135]
}

nav = {
    'pos': -1,
    'des': -1 
}

def main():
    global nav
    rospy.init_node('pose_goal')
    rospy.Subscriber(
        '/move_base/result',
        MoveBaseActionResult,
        listener_callback
    )
    initial_pose()
    while True:
        while nav['des'] >= 0:
            sleep(1)
        if nav['pos'] != 0:
            sleep(5)
            go_to_house(postoffice, 'correio')
            nav['des'] = 0
        else:
            address = input('Digite o endereço da próxima encomenda: ')
            goal = houses[address]
            go_to_house(goal, 'casa '+address)
            nav['des'] = int(address)

def initial_pose():
    pub = rospy.Publisher(
        '/initialpose',
        PoseWithCovarianceStamped,
        queue_size=10
    )
    rospy.init_node('pose_goal')
    rate = rospy.Rate(10)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = -2.0
    msg.pose.pose.position.y = -0.5
    quat = quaternion_from_euler(0, 0, 0)
    msg.pose.pose.orientation.x = quat[0]
    msg.pose.pose.orientation.y = quat[1]
    msg.pose.pose.orientation.z = quat[2]
    msg.pose.pose.orientation.w = quat[3]
    if not rospy.is_shutdown():
        for i in range(2):
            pub.publish(msg)
            sleep(1)

def set_goal(x, y, theta):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = x
    goal.pose.position.y = y
    quat = quaternion_from_euler(0, 0, math.radians(theta))
    goal.pose.orientation.x = quat[0]
    goal.pose.orientation.y = quat[1]
    goal.pose.orientation.z = quat[2]
    goal.pose.orientation.w = quat[3]
    return goal

def listener_callback(data):
    global nav
    msg = ''
    if data.status.status == 3:
        msg = 'O robô chegou ao destino!'
        nav['pos'] = nav['des']
    elif data.status.status == 4:
        msg = 'Impossível chegar ao destino.'

    if msg:
        rospy.loginfo(msg)
        nav['des'] = -1

def go_to_house(goal, name):
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(4)
    goal = set_goal(goal[0], goal[1], goal[2])

    if not rospy.is_shutdown():
        pub.publish(goal)
        sleep(1)
        pub.publish(goal)
        rospy.loginfo('A caminho do destino ' + name)

if __name__ == '__main__':
    main()