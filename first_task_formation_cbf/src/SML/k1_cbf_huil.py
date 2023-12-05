#!/usr/bin/env python
#=====================================
#       Code to control nexus0
#    with direct keyboard controls
#=====================================

from re import I, U
import sys
import rospy
import copy
import geometry_msgs.msg
from rospy.core import loginfo
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64

import numpy as np
from scipy.optimize import minimize, LinearConstraint


class KCBFHuIL():
    #=====================================
    #         Class constructor
    #  Initializes node and subscribers
    #=====================================
    def __init__(self):
        #----------------
        # Initialisation 
        #----------------
        #Initialize node
        rospy.init_node('k1_cbf_guil')

        #Controller gains (for x, y, heading)
        gains = (1, 1, 1)

        #Init robot pose
        self.robot_pose = geometry_msgs.msg.PoseStamped()
        #Init last received pose time
        self.last_received_robot_pose = rospy.Time()

        #Init HuIL controller velocity
        self.vel_huil = geometry_msgs.msg.Twist()

        #Timeout in seconds
        timeout = 0.5

        #Booleans to check is positions have been received
        init_pose = False

        #Setup robot pose subscriber
        rospy.Subscriber("/qualisys/nexus3/pose", geometry_msgs.msg.PoseStamped, self.robot_pose_callback)
        
        #Setup HuIL controller subscriber
        rospy.Subscriber("/HuIL/key_vel", geometry_msgs.msg.Twist, self.huil_callback)

        #Setup velocity command publisher
        vel_pub = rospy.Publisher("/nexus3/cmd_vel", geometry_msgs.msg.Twist, queue_size=100)

        #Setup transform subscriber
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        
        #Wait for transform to be available
        while not tf_buffer.can_transform('mocap', "nexus3", rospy.Time()):
            rospy.loginfo("Wait for transform to be available for nexus0")
            rospy.sleep(1)

        #Create a ROS Twist message for velocity command
        vel_cmd_msg = geometry_msgs.msg.Twist()

        #-----------------------------------------------------------------
        # Loop at set frequency and publish position command if necessary
        #-----------------------------------------------------------------
        #Loop frequency in Hz
        loop_frequency = 100
        r = rospy.Rate(loop_frequency)

        rospy.sleep(1)

        rospy.loginfo("CBF-Formation controller Centralized Initialized for nexus3")

        while not rospy.is_shutdown():
            #Run controller if robot position and neighbours position have been received
            now  = rospy.Time.now()
            init_pose = (now.to_sec() < self.last_received_robot_pose.to_sec() + timeout)

            #Get euler angle from robot pose quaternion
            (roll, pitch, yaw) = euler_from_quaternion([self.robot_pose.pose.orientation.x,
                                                        self.robot_pose.pose.orientation.y,
                                                        self.robot_pose.pose.orientation.z,
                                                        self.robot_pose.pose.orientation.w])
            heading = yaw

            rospy.loginfo("X: "+str(self.robot_pose.pose.position.x)+", Y: "+str(self.robot_pose.pose.position.y))

            #if init_pose:
            u_x = self.vel_huil.linear.x
            u_y = self.vel_huil.angular.z
            u_heading = -heading

            #-------------
            # Send output
            #-------------
            vel_cmd_msg.linear.x = u_x * gains[0]
            vel_cmd_msg.linear.y = u_y * gains[1]
            vel_cmd_msg.angular.z = u_heading * gains[2]

            #Else stop robot
            #else:
            #    vel_cmd_msg.linear.x = 0
            #    vel_cmd_msg.linear.y = 0
            #    vel_cmd_msg.angular.z = 0

            #------------------------------------------
            # Publish command & controller output norm
            #------------------------------------------
            try:
                #Get transform from mocap frame to robot frame
                transform = tf_buffer.lookup_transform('mocap', "nexus3", rospy.Time())
                vel_cmd_msg_transformed = transform_twist(vel_cmd_msg, transform)
                #Publish cmd message
                vel_pub.publish(vel_cmd_msg_transformed)
            except:
                continue

            #---------------------------------
            # Sleep to respect loop frequency
            #---------------------------------
            r.sleep()

    #=====================================
    #          Callback function 
    #      for robot pose feedback
    #=====================================
    def robot_pose_callback(self, pose_stamped_msg):
        #Save robot pose as class variable
        self.robot_pose = pose_stamped_msg

        #Save when last pose was received
        self.last_received_robot_pose = rospy.Time.now()

    #=====================================
    #          Callback function 
    #         for huil controller
    #=====================================
    def huil_callback(self, twist_msg):
        #Save huil controller twist as class variable
        self.vel_huil = twist_msg


#=====================================
# Apply transform to a twist message 
#     including angular velocity
#=====================================
def transform_twist(twist= geometry_msgs.msg.Twist, transform_stamped = geometry_msgs.msg.TransformStamped):

    transform_stamped_ = copy.deepcopy(transform_stamped)
    #Inverse real-part of quaternion to inverse rotation
    transform_stamped_.transform.rotation.w = - transform_stamped_.transform.rotation.w

    twist_vel = geometry_msgs.msg.Vector3Stamped()
    twist_rot = geometry_msgs.msg.Vector3Stamped()
    twist_vel.vector = twist.linear
    twist_rot.vector = twist.angular
    out_vel = tf2_geometry_msgs.do_transform_vector3(twist_vel, transform_stamped_)
    out_rot = tf2_geometry_msgs.do_transform_vector3(twist_rot, transform_stamped_)

    #Populate new twist message
    new_twist = geometry_msgs.msg.Twist()
    new_twist.linear = out_vel.vector
    new_twist.angular = out_rot.vector

    return new_twist

#=====================================
#               Main
#=====================================
if __name__ == "__main__":
    k3_cbf_huil = KCBFHuIL()
    try:
        rospy.spin()
    except ValueError as e:
        rospy.logerr(e)
        sys.exit(0)
    except rospy.ROSInterruptException:
        print("Shutting down")
        sys.exit(0)
