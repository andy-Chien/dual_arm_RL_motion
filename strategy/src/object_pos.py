#!/usr/bin/env python

"""Use to reset pose of model for simulation."""

import rospy
from gazebo_msgs.msg import ModelState


def set_object(name, pos, ori):
    msg = ModelState()
    msg.model_name = name
    msg.pose.position.x = pos[0]
    msg.pose.position.y = pos[1]
    msg.pose.position.z = pos[2]
    msg.pose.orientation.w = ori[0]
    msg.pose.orientation.x = ori[1]
    msg.pose.orientation.y = ori[2]
    msg.pose.orientation.z = ori[3]
    msg.reference_frame = 'world'
    
    set_mode_pub.publish(msg)
    print msg

    
if __name__ == '__main__':
    rospy.init_node('set_obj')
    print("set_link_state")
    
    set_mode_pub = rospy.Publisher(
        '/gazebo/set_model_state',
        ModelState,
        queue_size=1,
        latch=True
    )

    names = ('lunchbox1', 'lunchbox2', 'lunchbox3', 'lunchbox4', 
             'drink1', 'drink2', 'drink3', 'drink4',
             'riceball1', 'riceball2', 'riceball3', 'riceball4')
    pos  = ((-0.492,   -0.16, 0.7),
            (-0.492,   -0.16, 0.75),
            (-0.492,    0.16, 0.7),
            (-0.492,    0.16, 0.75),
            (-0.26, 0.11, 0.76),
            (-0.36,  0.11, 0.76),
            (-0.26, 0.21, 0.76),
            (-0.36,  0.21, 0.76),
            (-0.235,  -0.2,  0.7),
            (-0.33,   -0.2,  0.7),
            (-0.235,  -0.1,  0.7),
            (-0.33,   -0.1,  0.7))
    ori  = ((0, 0, 0, 1),
            (0, 0, 0, 1),
            (0, 0, 0, 1),
            (0, 0, 0, 1),
            (0, 0, 0, 0),
            (0, 0, 0, 0),
            (0, 0, 0, 0),
            (0, 0, 0, 0),
            (0, -1, 0, 1),
            (0, -1, 0, 1),
            (0, -1, 0, 1),
            (0, -1, 0, 1))
    
    for i, name in enumerate(names):
        set_object(name, pos[i], ori[i])
        rospy.sleep(0.1)
