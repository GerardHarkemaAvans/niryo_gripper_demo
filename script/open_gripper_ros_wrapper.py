#!/usr/bin/env python

# Imports
from niryo_robot_python_ros_wrapper import *
import rospy

# Initializing ROS node
rospy.init_node('niryo_ned_example_python_ros_wrapper')

# Connecting to the ROS Wrapper & calibrating if needed
niryo_robot = NiryoRosWrapper()
niryo_robot.calibrate_auto()
niryo_robot.open_gripper()
rospy.sleep(3)
niryo_robot.close_gripper()
rospy.sleep(3)
niryo_robot.open_gripper()





if 0:
    # Updating tool
    tool_used = ToolID.XXX
    niryo_robot.update_tool()

    if tool_used in [ToolID.GRIPPER_1, ToolID.GRIPPER_2, ToolID.GRIPPER_3, ToolID.GRIPPER_4]:
        niryo_robot.close_gripper(speed=500)
    elif tool_used == ToolID.ELECTROMAGNET_1:
        pin_electromagnet = PinID.XXX
        niryo_robot.setup_electromagnet(pin_electromagnet)
        niryo_robot.activate_electromagnet(pin_electromagnet)
    elif tool_used == ToolID.VACUUM_PUMP_1:
        niryo_robot.pull_air_vacuum_pump()
