#!/usr/bin/env python
#=====================================
#     Follower controller for the        
#          SML Nexus robot
#               ----
#   Listen to both the robot pose 
# and another tracked object from
#    the mocap system and output a
#   velocity command to drive the
#    robot to the follow the object
#=====================================
import sys
import rospy
import copy
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
import std_msgs.msg

class FollowerController():
    #=====================================
    #         Class constructor
    #  Initializes node and subscribers
    #=====================================
    def __init__(self):
        #----------------
        # Initialisation 
        #----------------
        #Initialize node
        rospy.init_node('follower_controller')

        #Get tracked object name from parameters
        robot_name = rospy.get_param('~robot_name')

        #Get tracked object name from parameters
        tracked_object = rospy.get_param('~tracked_object')

        #Get offset from tracked object
        tracking_offset = rospy.get_param('~tracking_offset')

        #Tolerance interval for declaring a position as reached (x, y, heading angle)
        reach_tolerance = (0.05, 0.05, 0.02)

        #Controller gains (for x, y, heading)
        gains = (2, 2, 2)

        #Init robot pose
        self.robot_pose = geometry_msgs.msg.PoseStamped()

        #Init tracked object pose
        self.tracked_object_pose = geometry_msgs.msg.PoseStamped()

        #Init last received pose time
        self.last_received_robot_pose = rospy.Time()

        #Init last received pose time
        self.last_received_tracked_pose = rospy.Time()

        #Timeout in seconds
        timeout = 0.5

        #Booleans to check is positions have been received
        init_pose = False
        init_track = False

        #Setup robot pose subscriber
        rospy.Subscriber("/qualisys/"+robot_name+"/pose", geometry_msgs.msg.PoseStamped, self.robot_pose_callback)

        #Setup tracked object pose subscriber
        rospy.Subscriber("/qualisys/"+tracked_object+"/pose", geometry_msgs.msg.PoseStamped, self.tracked_object_pose_callback)

        #Setup velocity command publisher
        vel_pub = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=100)

        #Setup error publishers
        error_x_pub = rospy.Publisher("error/x", std_msgs.msg.Float32, queue_size=100)
        error_y_pub = rospy.Publisher("error/y", std_msgs.msg.Float32, queue_size=100)
        error_heading_pub = rospy.Publisher("error/heading", std_msgs.msg.Float32, queue_size=100)

        #Setup transform subscriber
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        #Wait for transform to be available
        while not tf_buffer.can_transform('mocap', robot_name, rospy.Time()):
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

        rospy.loginfo("Follower controller Initialized for "+robot_name+
                      " tracking "+tracked_object+
                      " with offset x:"+str(tracking_offset[0])+
                      ", y:"+str(tracking_offset[1])+
                      ", heading:"+str(tracking_offset[2])
                      )
        
        while not rospy.is_shutdown():
            #Run controller if robot position and tracked object position have been received
            now  = rospy.Time.now()
            init_pose = (now.to_sec() < self.last_received_robot_pose.to_sec() + timeout)
            init_track = (now.to_sec() < self.last_received_tracked_pose.to_sec() + timeout)

            if init_pose and init_track:
                #-----------------------------------
                # Compute error on x and y position
                #-----------------------------------
                error_x = (self.tracked_object_pose.pose.position.x + tracking_offset[0]) - self.robot_pose.pose.position.x 
                error_y = (self.tracked_object_pose.pose.position.y + tracking_offset[1]) - self.robot_pose.pose.position.y

                #-----------------------
                # Compute heading error
                #-----------------------
                #Get euler angle from robot pose quaternion
                (roll, pitch, yaw) = euler_from_quaternion([self.robot_pose.pose.orientation.x,
                                                            self.robot_pose.pose.orientation.y,
                                                            self.robot_pose.pose.orientation.z,
                                                            self.robot_pose.pose.orientation.w])

                #Get euler angle from robot pose quaternion
                (track_roll, track_pitch, track_yaw) = euler_from_quaternion([self.tracked_object_pose.pose.orientation.x,
                                                                              self.tracked_object_pose.pose.orientation.y,
                                                                              self.tracked_object_pose.pose.orientation.z,
                                                                              self.tracked_object_pose.pose.orientation.w])

                error_heading = (track_yaw + tracking_offset[2]) - yaw

                #---------------
                # Publish error
                #---------------
                error_x_pub.publish(std_msgs.msg.Float32(error_x))
                error_y_pub.publish(std_msgs.msg.Float32(error_y))
                error_heading_pub.publish(std_msgs.msg.Float32(error_heading))

                #----------------
                # Compute output
                #----------------
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
                transform = tf_buffer.lookup_transform('mocap', robot_name, rospy.Time())
                #
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
    #  for tracked object pose feedback
    #=====================================
    def tracked_object_pose_callback(self, pose_stamped_msg):
        #Save robot pose as class variable
        self.tracked_object_pose = pose_stamped_msg

        #Save when last pose was received
        self.last_received_tracked_pose = rospy.Time.now()


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
    follower_controller = FollowerController()
    try:
        rospy.spin()
    except ValueError as e:
        rospy.logerr(e)
        sys.exit(0)
    except rospy.ROSInterruptException:
        print "Shutting down"
        sys.exit(0)