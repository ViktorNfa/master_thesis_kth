#!/usr/bin/env python
#=====================================
#       Formation keeping with
#     communication maintenance
#       or obstacle avoidance 
#      for the SML Nexus robot
#    done in a centralized manner
#   with only one controller node
#               ----
#     Listen to both the robot pose 
#   and the other neighbouring robots 
#  from the mocap system and output a
#    velocity command to drive the
#   robot to keep the formation and
#  keep the communication bw robots or
#   avoid colliding with other robots
#=====================================
from re import U
import sys
import rospy
import copy
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion

import numpy as np
from scipy.optimize import minimize, LinearConstraint


class CBFFormationControllerCentralized():
    #=====================================
    #         Class constructor
    #  Initializes node and subscribers
    #=====================================
    def __init__(self):
        #----------------
        # Initialisation 
        #----------------
        #Initialize node
        rospy.init_node('cbf_formation_controller_centralized')

        #Get the robots number from parameters
        robots_number = rospy.get_param('~robots_number')
        number_robots = len(robots_number)

        #Get neighbour numbers from parameters
        neighbours = rospy.get_param('~neighbours')
        number_neighbours = []
        for i in range(number_robots):
            number_neighbours.append(len(neighbours[i]))

        #Get the ideal positions of the formation
        formation_positions = rospy.get_param('/formation_positions')

        #Get parameter representing communication maintenance and/or collision avoidance
        cbf_cm = rospy.get_param('/cbf_cm')
        cbf_oa = rospy.get_param('/cbf_oa')

        #Maximum/minimum safe distance for CBF
        safe_distance_cm = rospy.get_param('/safe_distance_cm')
        safe_distance_oa = rospy.get_param('/safe_distance_oa')

        #Controller gains (for x, y, heading)
        gains = (1, 1, 1)

        #CBF constraint parameters
        alfa = 1

        #Init robot pose
        self.robot_pose = []
        #Init last received pose time
        self.last_received_robot_pose = []

        # For each robot
        for i in range(number_robots):
            self.robot_pose.append(geometry_msgs.msg.PoseStamped())
            self.last_received_robot_pose.append(rospy.Time())

        #Timeout in seconds
        timeout = 0.5

        #Booleans to check is positions have been received
        init_pose = []
        for i in range(number_robots):
            init_pose.append(bool())

        #Setup robot pose subscriber
        for i in range(number_robots):
            rospy.Subscriber("/qualisys/nexus"+str(robots_number[i])+"/pose", geometry_msgs.msg.PoseStamped, self.robot_pose_callback, i)

        #Setup velocity command publisher
        vel_pub = []
        for i in range(number_robots):
            vel_pub.append(rospy.Publisher("/nexus"+str(robots_number[i])+"/cmd_vel", geometry_msgs.msg.Twist, queue_size=100))

        #Setup transform subscriber
        tf_buffer = []
        tf_listener = []
        for i in range(number_robots):
            tf_buffer.append(tf2_ros.Buffer())
            tf_listener.append(tf2_ros.TransformListener(tf_buffer[i]))
        
            #Wait for transform to be available
            while not tf_buffer[i].can_transform('mocap', "nexus"+str(robots_number[i]), rospy.Time()):
                rospy.loginfo("Wait for transform to be available for nexus"+str(robots_number[i]))
                rospy.sleep(1)

        #Create a ROS Twist message for velocity command
        vel_cmd_msg = []
        for i in range(number_robots):
            vel_cmd_msg.append(geometry_msgs.msg.Twist())

        #-----------------------------------------------------------------
        # Loop at set frequency and publish position command if necessary
        #-----------------------------------------------------------------
        #loop frequency in Hz
        loop_frequency = 50
        r = rospy.Rate(loop_frequency)

        rospy.loginfo("CBF-Formation controller Centralized Initialized for nexus"+str(robots_number)+
                      " with neighbours "+str(neighbours)+" with ideal positions "+str(formation_positions)
                      )
        
        while not rospy.is_shutdown():
            #Run controller if robot position and neighbours position have been received
            now  = rospy.Time.now()
            for i in range(number_robots):
                init_pose[i] = (now.to_sec() < self.last_received_robot_pose[i].to_sec() + timeout)

            if all(init_pose):
                
                u_x = []
                u_y = []
                u_heading = []

                for i in range(number_robots):

                    #-----------------------------------
                    # Compute nominal (auto) controllers
                    #-----------------------------------
                    u_nom_x = 0
                    u_nom_y = 0
                    for j in neighbours[i]:
                        u_nom_x += self.robot_pose[j-1].pose.position.x - self.robot_pose[i].pose.position.x + \
                            (formation_positions[i][0] - formation_positions[j-1][0])
                        u_nom_y += self.robot_pose[j-1].pose.position.y - self.robot_pose[i].pose.position.y  + \
                            (formation_positions[i][1] - formation_positions[j-1][1])

                    #-----------------------
                    # Compute heading error
                    #-----------------------
                    #Get euler angle from robot pose quaternion
                    (roll, pitch, yaw) = euler_from_quaternion([self.robot_pose[i].pose.orientation.x,
                                                                self.robot_pose[i].pose.orientation.y,
                                                                self.robot_pose[i].pose.orientation.z,
                                                                self.robot_pose[i].pose.orientation.w])

                    #Get euler angle from neighbours pose quaternion
                    error_heading = 0
                    for j in neighbours[i]:
                        (neighbour_roll, neighbour_pitch, neighbour_yaw) = euler_from_quaternion([self.robot_pose[j-1].pose.orientation.x,
                                                                                self.robot_pose[j-1].pose.orientation.y,
                                                                                self.robot_pose[j-1].pose.orientation.z,
                                                                                self.robot_pose[j-1].pose.orientation.w])
                        error_heading += neighbour_yaw - yaw

                    #----------------------------
                    # Solve minimization problem
                    #----------------------------
                    #Construct a vector for the nominal controller and robot states
                    u_nom = np.array([u_nom_x, u_nom_y])
                    x_i = np.array([self.robot_pose[i].pose.position.x, self.robot_pose[i].pose.position.y])
                    
                    #Calculate CBF constraint in the form of a*u + b >= 0
                    h_cm = np.zeros((number_neighbours[i], 1))
                    grad_h_cm = np.zeros((number_neighbours[i], 2))
                    h_oa = np.zeros((number_neighbours[i], 1))
                    grad_h_oa = np.zeros((number_neighbours[i], 2))
                    for j in range(number_neighbours[i]):
                        x_j = np.array([self.robot_pose[neighbours[i][j]-1].pose.position.x, self.robot_pose[neighbours[i][j]-1].pose.position.y])
                        h_cm[j] = safe_distance_cm**2 - np.linalg.norm(x_i - x_j)**2
                        grad_h_cm[j] = -2*np.transpose(np.array([x_i[0] - x_j[0], x_i[1] - x_j[1]]))
                        h_oa[j] = np.linalg.norm(x_i - x_j)**2 - safe_distance_oa**2
                        grad_h_oa[j] = 2*np.transpose(np.array([x_i[0] - x_j[0], x_i[1] - x_j[1]]))

                    # Comunication maintenance CBF
                    a_cm = grad_h_cm
                    b_cm = alfa*h_cm.reshape(number_neighbours[i],)

                    # Obstacle avoidance CBF
                    a_oa = grad_h_oa
                    b_oa = alfa*h_oa.reshape(number_neighbours[i],)

                    # Define linear constraints
                    constraint_cm = LinearConstraint(a_cm*cbf_cm, lb=-b_cm, ub=np.inf)
                    constraint_oa = LinearConstraint(a_oa*cbf_oa, lb=-b_oa, ub=np.inf)
                    
                    # Define objective function
                    def objective_function(u, u_nom):
                        return np.linalg.norm(u - u_nom)**2
                    
                    # Construct the problem
                    u = minimize(
                        objective_function,
                        x0=u_nom,
                        args=(u_nom,),
                        constraints=[constraint_cm, constraint_oa],
                    )

                    # The optimal value for u
                    u_x.append(u.x[0])
                    u_y.append(u.x[1])

                    u_heading.append(error_heading)

                    #----------------
                    # Compute output
                    #----------------
                    vel_cmd_msg[i].linear.x = u_x[i] * gains[0]
                    vel_cmd_msg[i].linear.y = u_y[i] * gains[1]
                    vel_cmd_msg[i].angular.z = u_heading[i] * gains[2]

            #Else stop robot
            else:
                for i in range(number_robots):
                    vel_cmd_msg[i].linear.x = 0
                    vel_cmd_msg[i].linear.y = 0
                    vel_cmd_msg[i].angular.z = 0

            #-----------------
            # Publish command
            #-----------------
            try:
                transform = []
                vel_cmd_msg_transformed = []
                for i in range(number_robots):
                    #Get transform from mocap frame to robot frame
                    transform.append(tf_buffer[i].lookup_transform('mocap', "nexus"+str(robots_number[i]), rospy.Time()))
                    #
                    vel_cmd_msg_transformed.append(transform_twist(vel_cmd_msg[i], transform[i]))
                    #Publish cmd message
                    vel_pub[i].publish(vel_cmd_msg_transformed[i])
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
    def robot_pose_callback(self, pose_stamped_msg, i):
        #Save robot pose as class variable
        self.robot_pose[i] = pose_stamped_msg

        #Save when last pose was received
        self.last_received_robot_pose[i] = rospy.Time.now()


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
    cbf_follower_controller_centralized = CBFFormationControllerCentralized()
    try:
        rospy.spin()
    except ValueError as e:
        rospy.logerr(e)
        sys.exit(0)
    except rospy.ROSInterruptException:
        print("Shutting down")
        sys.exit(0)
