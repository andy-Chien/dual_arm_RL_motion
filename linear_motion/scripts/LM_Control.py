#!/usr/bin/env python
"""description"""

# pylint: disable = invalid-name
# pylint: disable = C0326
# pylint: disable = W0105, C0303


import sys
import rospy
import arm_task

from std_msgs.msg import Int32
from std_msgs.msg import String
from linear_motion.msg   import LM_Cmd

TargetId =      ['a',  'b',  'c',  'd',  'e',  'f',  'g',  'h',  'i',  'j',   'k',   'l']
TargetShift_X = [  0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 50000, 65000, 80000]
TargetShift_Z = [  0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 40000, 60000]

def GetShift(LM_Dir):
    """ description """
    if ch.data in TargetId:
        if LM_Dir == 'x':
            return TargetShift_X[TargetId.index(ch.data)]
        elif LM_Dir=='z':
            return TargetShift_Z[TargetId.index(ch.data)]
        else:
            print 'Error input dir'
    else:
        print 'Error input character'

def Show_FB_callback(msg):
    """ description """
    # print msg.status
    print msg.status
    if msg.status == 'LM_idle':
        print 'LM_idle'

    elif msg.status == 'LM_busy':
        print 'LM_busy'  
            
    elif msg.status == 'LM_complete':
        print 'LM_complete' # execute arm task
        Is_LM_Complete = True
    else:
        print 'err'

if __name__ == '__main__':
    """ Check and initialize input parameters """
    if len(sys.argv) == 3:
        status = 'idle'
        id =    int(sys.argv[1])
        ch = String(sys.argv[2])
        # sys.exit(1)
    else:
        print 'error input arguments'
        sys.exit(1)

    """ Init """
    Is_LM_Complete = False
    """ Initialize ros node and publish cmd """
    try:
        rospy.init_node('LinearMove', anonymous=True)
        rospy.loginfo('running')

        set_pls_pub = rospy.Publisher(
            '/position_topic',
            LM_Cmd,
            latch = True,
            queue_size=1
        )
        msg = LM_Cmd()
        msg.id = id
        msg.x  = GetShift('x')
        msg.z  = GetShift('z')
        print msg.x
        print msg.z

        set_pls_pub.publish(msg)
        rate = rospy.Rate(10) # 10hz
        # task = arm_task.ArmTask()
        tmp_pos = 0.1
        while not rospy.is_shutdown():
            """ subscribe """
            task = arm_task.ArmTask()
            task.pub_ikCmd(
                'ptp',
                (0.4, tmp_pos, 0.2),
                (-90, 0, 0)
            )
            # if task.__is_busy is True:
            #     tmp_pos = 0.1
            print task.Is_Busy()
            # if task.__is_busy is True:
            #     task.xyz_relative_control('ptp', 0.1, 0, 0)


            # if Is_LM_Complete==False:
            #     rospy.Subscriber('/LM_FeedBack', LM_Cmd, Show_FB_callback)
            # else:
            #     task.pub_ikCmd(
            #         'ptp',
            #         (0.4, 0.1, 0.2),
            #         (0, 0, 0)
            #     )

            # rospy.spin()
            rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo('error')
        pass
