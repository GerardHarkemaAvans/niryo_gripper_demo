#!/usr/bin/env python

import rospy

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Services
from tools_interface.srv import ToolCommand

from std_msgs.msg import Int32


class gripper:

    def __init__(self):

        namespace = '/niryo_robot_tools_commander/' #rospy.get_param("~namespace_topics")
	rospy.logout(namespace)
        #namespace = rospy.get_param("~namespace_topics")
	#rospy.logout(namespace)
        #self.__service_open_gripper = rospy.ServiceProxy(namespace + 'open_gripper', ToolCommand)
                                                         
        #self.__service_close_gripper = rospy.ServiceProxy(namespace + 'close_gripper',ToolCommand)
       
	# Tool
        self.current_tool_id = None
        #self.list_id_grippers = [11, 12, 13, 14]
        rospy.Subscriber('/niryo_robot_tools_commander/current_id', Int32,
                         self.sub_selected_tool_id)
	rospy.logout(' end constructor')

    def sub_selected_tool_id(self, msg):
        value = int(msg.data)
        self.current_tool_id = value
	rospy.logout("sub_selected_tool_id")


    def open(self, gripper_id, open_position, open_speed, open_hold_torque, open_max_torque):
        self.__service_open_gripper(id=gripper_id, position=open_position, speed=open_speed,
                                    hold_torque=open_hold_torque, max_torque=open_max_torque)
        return True
        # try:
        #     resp = self.__service_open_gripper(id=gripper_id, position=open_position, speed=open_speed,
        #                                        hold_torque=open_hold_torque, max_torque=open_max_torque)
        #     return resp.state
        # except rospy.ServiceException:
        #     rospy.logerr("ROS Tool Interface - Failed to Open Gripper")
        #     return self.__state_ros_communication_problem

    def close(self, gripper_id, close_position, close_speed, close_hold_torque, close_max_torque):
        resp = self.__service_close_gripper(id=gripper_id, position=close_position,
                                            speed=close_speed, hold_torque=close_hold_torque,
                                            max_torque=close_max_torque)
        return True
        # try:
        #     resp = self.__service_close_gripper(id=gripper_id, position=close_position,
        #                                         speed=close_speed, hold_torque=close_hold_torque,
        #                                         max_torque=close_max_torque)
        #     return resp.state
        # except rospy.ServiceException:
        #     rospy.logerr("ROS Tool Interface - Failed to Close Gripper")
        #     return self.__state_ros_communication_problem                                                          ToolCommand)

class air_vacuum_pump:

    def __init__(self):

        namespace = rospy.get_param("~namespace_topics")

        self.__service_pull_air_vacuum_pump = rospy.ServiceProxy(namespace + 'pull_air_vacuum_pump',
                                                                 ToolCommand)
        self.__service_push_air_vacuum_pump = rospy.ServiceProxy(namespace + 'push_air_vacuum_pump',
                                                                 ToolCommand)

    def pull(
            self, vp_id, vp_pull_air_velocity, vp_pull_air_position,
            vp_pull_air_max_torque, vp_pull_air_hold_torque):
        try:
            resp = self.__service_pull_air_vacuum_pump(
                id=vp_id, speed=vp_pull_air_velocity, position=vp_pull_air_position,
                max_torque=vp_pull_air_max_torque, hold_torque=vp_pull_air_hold_torque)
            return resp.state
        except rospy.ServiceException:
            rospy.logerr("ROS Tool Interface - Failed to Pull Air")
            return self.__state_ros_communication_problem

    def push(self, vp_id, vp_push_air_velocity, vp_push_air_position, vp_push_air_max_torque):
        try:
            resp = self.__service_push_air_vacuum_pump(id=vp_id, speed=vp_push_air_velocity,
                                                       position=vp_push_air_position, max_torque=vp_push_air_max_torque,
                                                       hold_torque=0)
            return resp.state
        except rospy.ServiceException:
            rospy.logerr("ROS Tool Interface - Failed to Push Air")
            return self.__state_ros_communication_problem


# Initializing ROS node
rospy.init_node('niryo_ned_example_python_ros_wrapper')

#get topic gripper_id


#std_msgs/Int32

#/niryo_robot_tools_commander/current_id

grijper = gripper()

#rospy.spin()
rospy.sleep(10)

rospy.logout("ready")


#grijper.open(speed=500, max_torque_percentage=100, hold_torque_percentage=20)
