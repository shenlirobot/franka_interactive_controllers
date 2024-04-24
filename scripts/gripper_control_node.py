#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
# from psutil import Popen
# import shlex
import subprocess


class GripperControlNode:
    def __init__(self):
        self.curr_gripper_state = 1 # open, 0 for close
        rospy.init_node('gripper_control_node', anonymous=True)
        rospy.Subscriber('/gripper_command', Int32, self.gripper_control_callback)
        rospy.spin()

    def gripper_control_callback(self, msg):
        if self.curr_gripper_state == 0 and msg.data == 1:
            self.open()
            self.curr_gripper_state = 1
        elif self.curr_gripper_state == 1 and msg.data == 0:
            self.close()
            self.curr_gripper_state = 0

    def open(self):
        # node_process = Popen(shlex.split('rosrun franka_interactive_controllers libfranka_gripper_run 1'))
        process = subprocess.Popen("rosrun franka_interactive_controllers franka_gripper_run_node 1", shell=True)
        rospy.loginfo("Opening gripper")
        process.wait()
        process.terminate()

    def close(self):
        # node_process = Popen(shlex.split('rosrun franka_interactive_controllers libfranka_gripper_run 0'))
        process = subprocess.Popen("rosrun franka_interactive_controllers franka_gripper_run_node 0", shell=True)
        rospy.loginfo("Closing gripper")
        process.wait()
        process.terminate()

if __name__ == '__main__':
    try:
        gripper_control = GripperControlNode()
    except rospy.ROSInterruptException:
        pass
