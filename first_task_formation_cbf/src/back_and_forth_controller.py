#!/usr/bin/env python
#=====================================
#      Simple controller for the        
#          SML Nexus robot
#               ----
#   Listen to the robot pose from
#    the mocap system and output a
#   velocity command to drive the
#    robot back and forth between
#             two points
#=====================================
import sys
import rospy
import copy
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion

class BackAndForthController():
    #=====================================
    #         Class constructor
    #  Initializes node and subscribers
    #=====================================
    def __init__(self):
        #Initialize node
        rospy.init_node('back_and_forth_controller')

        #First pose to reach, written as (x, y, heading angle)
        goal1 = (1.5, 0, 0)
        #Second pose to reach, written as (x, y, heading angle)
        goal2 = (-1.0, -1.0, 1.57)
        #Set current goal
        current_goal = goal1

        #Tolerance interval for declaring a position as reached (x, y, heading angle)
        reach_tolerance = (0.025, 0.025, 0.1)

        #Controller gains (for x, y, heading)
        gains = (0.5, 0.5, 0.5)

        #Init robot pose
        self.robot_pose = geometry_msgs.msg.PoseStamped()

        #Init last received pose time
        self.last_received_pose = rospy.Time()

        #Timeout in seconds
        timeout = 0.5

        #Setup pose subscriber
        rospy.Subscriber("/qualisys/nexus1/pose", geometry_msgs.msg.PoseStamped, self.pose_callback)

        #Setup velocity command publisher
        vel_pub = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=100)

        #Setup transform subscriber
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        #Wait for transform to be available
        while not tf_buffer.can_transform('mocap', 'nexus1', rospy.Time()):
            rospy.loginfo("Wait for transform to be available")
            rospy.sleep(1)

        #Create a ROS Twist message for velocity command
        vel_cmd_msg = geometry_msgs.msg.Twist()

        #-----------------------------------------------------------------
        # Loop at set frequency and publish position command if necessary
        #-----------------------------------------------------------------
        #loop frequency in Hz
        loop_frequency = 50
        r = rospy.Rate(loop_frequency)
        
        while not rospy.is_shutdown():
            now  = rospy.Time.now()

            #-----------------------------------
            # Compute error on x and y position
            #-----------------------------------
            error_x = current_goal[0] - self.robot_pose.pose.position.x 
            error_y = current_goal[1] - self.robot_pose.pose.position.y

            #-----------------------
            # Compute heading error
            #-----------------------
            #Get euler angle from pose quaternion
            (roll, pitch, yaw) = euler_from_quaternion([self.robot_pose.pose.orientation.x,
                                                        self.robot_pose.pose.orientation.y,
                                                        self.robot_pose.pose.orientation.z,
                                                        self.robot_pose.pose.orientation.w])
            error_heading = current_goal[2] - yaw

            #----------------
            # Compute output
            #----------------
            #If pose feedback has been received recently
            if (now.to_sec() < self.last_received_pose.to_sec() + timeout):
                vel_cmd_msg.linear.x = error_x * gains[0]
                vel_cmd_msg.linear.y = error_y * gains[1]
                vel_cmd_msg.angular.z = error_heading * gains[2]
            #Else stop robot
            else:
                vel_cmd_msg.linear.x = 0
                vel_cmd_msg.linear.y = 0
                vel_cmd_msg.angular.z = 0               

            #-----------------
            # Publish command
            #-----------------
            try:
                #Get transform from mocap frame to robot frame
                transform = tf_buffer.lookup_transform('mocap', 'nexus1', rospy.Time())
                #
                vel_cmd_msg_transformed = transform_twist(vel_cmd_msg, transform)
                #Publish cmd message
                vel_pub.publish(vel_cmd_msg_transformed)
            except:
                continue

            #----------------------------------------
            # Check if current goal has been reached
            #----------------------------------------
            if ((abs(error_x) < reach_tolerance[0]) and (abs(error_y) < reach_tolerance[1]) and (abs(error_heading) < reach_tolerance[2])):
                if current_goal == goal1:
                    rospy.loginfo("goal 1 reached")
                    current_goal = goal2
                else:
                    rospy.loginfo("goal 2 reached")
                    current_goal = goal1

            #---------------------------------
            # Sleep to respect loop frequency
            #---------------------------------
            r.sleep()

    #=====================================
    #          Callback function 
    #      for robot pose feedback
    #=====================================
    def pose_callback(self, pose_stamped_msg):
        #Save robot pose as class variable
        self.robot_pose = pose_stamped_msg

        #Save when last pose was received
        self.last_received_pose = rospy.Time.now()


#=====================================
# Apply transform to a twist message 
#     including angular velocity
#=====================================
def transform_twist(twist = geometry_msgs.msg.Twist, transform_stamped = geometry_msgs.msg.TransformStamped):

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
    back_and_forth_controller = BackAndForthController()
    try:
        rospy.spin()
    except ValueError as e:
        rospy.logerr(e)
        sys.exit(0)
    except rospy.ROSInterruptException:
        print "Shutting down"
        sys.exit(0)