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


class K3CBFExtraHuIL():
    #=====================================
    #         Class constructor
    #  Initializes node and subscribers
    #=====================================
    def __init__(self):
        #----------------
        # Initialisation 
        #----------------
        #Initialize node
        rospy.init_node('k3_cbf_extra_huil')

        #Get the robots number from parameters
        robots_number = rospy.get_param('/robots_number')
        number_robots = len(robots_number)

        # Create safety constraint for arena
        As = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
        A_arena = np.zeros((number_robots*4, number_robots*2))
        for i in range(number_robots):
            A_arena[4*i:4*i+4, 2*i:2*i+2] = As
        b_arena = np.zeros((number_robots*4))
        xmax = 2.25
        xmin = -2.25
        ymax = 3
        ymin = -3

        #Get neighbour numbers from parameters
        neighbours = rospy.get_param('/neighbours')
        number_neighbours = []
        for i in range(number_robots):
            number_neighbours.append(len(neighbours[i]))

        #Create Laplacian matrix for the graph
        L_G = np.zeros((number_robots,number_robots))
        for i in range(number_robots):
            L_G[i, i] = number_neighbours[i]
            for j in neighbours[i]:
                L_G[i, j-1] = -1

        #Create edge list
        edges = []
        for i in range(number_robots):
            for j in neighbours[i]:
                if (i+1,j) not in edges and (j,i+1) not in edges:
                    edges.append((i+1,j))

        #Get the ideal positions of the formation
        formation_positions = rospy.get_param('/formation_positions')
        p_d = np.array(formation_positions)

        #Get parameter representing communication maintenance and/or collision avoidance
        cbf_cm = rospy.get_param('/cbf_cm')
        cbf_oa = rospy.get_param('/cbf_oa')

        #Maximum/minimum safe distance for CBF
        safe_distance_cm = rospy.get_param('/safe_distance_cm')
        safe_distance_oa = rospy.get_param('/safe_distance_oa')

        #See if HuIL controller is activated and which robot it affects
        huil = rospy.get_param('/huil')
        human_robot = rospy.get_param('/human_robot')

        #Dimension of the problem
        n = 2

        #Controller gains (for x, y, heading)
        gains = (1, 1, 1)

        #CBF constraint parameters
        alfa = 1

        #Init robot pose
        self.robot_pose = []
        #Init last received pose time
        self.last_received_robot_pose = []

        #For each robot
        for i in range(number_robots):
            self.robot_pose.append(geometry_msgs.msg.PoseStamped())
            self.last_received_robot_pose.append(rospy.Time())

        #Init HuIL controller velocity
        self.vel_huil = geometry_msgs.msg.Twist()

        #Timeout in seconds
        timeout = 0.5

        #Booleans to check is positions have been received
        init_pose = []
        for i in range(number_robots):
            init_pose.append(bool())

        #Setup robot pose subscriber
        for i in range(number_robots):
            rospy.Subscriber("/qualisys/nexus"+str(robots_number[i])+"/pose", geometry_msgs.msg.PoseStamped, self.robot_pose_callback, i)
        
        #Setup HuIL controller subscriber
        rospy.Subscriber("/HuIL/key_vel", geometry_msgs.msg.Twist, self.huil_callback)

        #Setup velocity command publisher
        vel_pub = []
        for i in range(number_robots):
            vel_pub.append(rospy.Publisher("/nexus"+str(robots_number[i])+"/cmd_vel", geometry_msgs.msg.Twist, queue_size=100))

        #Setup cbf functions publisher
        cbf_cm_pub = []
        cbf_oa_pub = []
        for i in range(len(edges)):
            cbf_cm_pub.append(rospy.Publisher("/cbf_function_cm"+"/".join(map(str,edges[i])), Float64, queue_size=100))
            cbf_oa_pub.append(rospy.Publisher("/cbf_function_oa"+"/".join(map(str,edges[i])), Float64, queue_size=100))
        #Setup message type
        cbf_cm_msg = Float64()
        cbf_oa_msg = Float64()

        #Setup controller output publisher and message
        nom_controller_pub = []
        controller_pub = []
        for i in range(number_robots):
            nom_controller_pub.append(rospy.Publisher("/nom_controller"+str(i+1), geometry_msgs.msg.Vector3, queue_size=100))
            controller_pub.append(rospy.Publisher("/controller"+str(i+1), geometry_msgs.msg.Vector3, queue_size=100))
        #Setup message type
        nom_controller_msg = geometry_msgs.msg.Vector3()
        controller_msg = geometry_msgs.msg.Vector3()

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
        #Loop frequency in Hz
        loop_frequency = 50
        r = rospy.Rate(loop_frequency)

        rospy.sleep(4)

        rospy.loginfo("CBF-Formation controller Centralized Initialized for nexus"+str(robots_number)+
                      " with neighbours "+str(neighbours)+" with ideal positions "+str(formation_positions)+
                      " and CBF comunication maintenance "+str(cbf_cm)+" with distance "+str(safe_distance_cm)+
                      " and obstacle avoidance "+str(cbf_oa)+" with distance "+str(safe_distance_oa)
                      )

        while not rospy.is_shutdown():
            #Run controller if robot position and neighbours position have been received
            now  = rospy.Time.now()
            for i in range(number_robots):
                init_pose[i] = (now.to_sec() < self.last_received_robot_pose[i].to_sec() + timeout)

            if all(init_pose):

                #---------------------------------------
                # Get position and orientation matrices
                #---------------------------------------
                p = np.zeros((number_robots, 2))
                heading = np.zeros((number_robots))
                for i in range(number_robots):
                    #Get x and y position from robot pose
                    p[i, 0] = self.robot_pose[i].pose.position.x
                    p[i, 1] = self.robot_pose[i].pose.position.y

                    #Get euler angle from robot pose quaternion
                    (roll, pitch, yaw) = euler_from_quaternion([self.robot_pose[i].pose.orientation.x,
                                                                self.robot_pose[i].pose.orientation.y,
                                                                self.robot_pose[i].pose.orientation.z,
                                                                self.robot_pose[i].pose.orientation.w])
                    heading[i] = yaw

                dist_p = p - p_d

                #------------------------------------
                # Compute nominal (auto) controllers
                #------------------------------------
                u_nom = np.dot(-L_G, dist_p)
                u_nom_heading = np.dot(-L_G, heading)

                #Convert nominal controller to CBF controller format and add HuIL
                u_n = np.zeros((number_robots*n))
                for i in range(number_robots):
                    #For the robot controlled with HuIL
                    if i == human_robot-1:
                        u_n[2*i] = huil*self.vel_huil.linear.x
                        u_n[2*i+1] = huil*self.vel_huil.angular.z
                    else:
                        u_n[2*i] = u_nom[i, 0]
                        u_n[2*i+1] = u_nom[i, 1]

                    #Publish nominal controller
                    nom_controller_msg.x = u_n[2*i]
                    nom_controller_msg.y = u_n[2*i+1]
                    nom_controller_pub[i].publish(nom_controller_msg)

                #Create CBF constraint matrices
                A_cm = np.zeros((len(edges), number_robots*n))
                b_cm = np.zeros((len(edges)))
                A_oa = np.zeros((len(edges), number_robots*n))
                b_oa = np.zeros((len(edges)))
                for i in range(len(edges)):
                    aux_i = edges[i][0]-1
                    aux_j = edges[i][1]-1

                    b_cm[i] = alfa*self.cbf_h(p[aux_i], p[aux_j], safe_distance_cm, 1)
                    b_oa[i] = alfa*self.cbf_h(p[aux_i], p[aux_j], safe_distance_oa, -1)
                    #Publish cbf function message
                    cbf_cm_msg = b_cm[i]
                    cbf_oa_msg = b_oa[i]
                    cbf_cm_pub[i].publish(cbf_cm_msg)
                    cbf_oa_pub[i].publish(cbf_oa_msg)

                    grad_h_value_cm = np.transpose(self.cbf_gradh(p[aux_i], p[aux_j], 1))
                    grad_h_value_oa = np.transpose(self.cbf_gradh(p[aux_i], p[aux_j], -1))

                    A_cm[i, 2*aux_i:2*aux_i+2] = grad_h_value_cm
                    A_cm[i, 2*aux_j:2*aux_j+2] = -grad_h_value_cm
                    A_oa[i, 2*aux_i:2*aux_i+2] = grad_h_value_oa
                    A_oa[i, 2*aux_j:2*aux_j+2] = -grad_h_value_oa

                #Calculate CBF for arena safety
                for i in range(number_robots):
                    b_arena[4*i] = xmax - p[i, 0]
                    b_arena[4*i+1] = p[i, 0] - xmin
                    b_arena[4*i+2] = ymax - p[i, 1]
                    b_arena[4*i+3] = p[i, 1] - ymin

                #----------------------------
                # Solve minimization problem
                #----------------------------
                #Define linear constraints
                constraint_cm = LinearConstraint(A_cm*cbf_cm, lb=-b_cm*cbf_cm, ub=np.inf)
                constraint_oa = LinearConstraint(A_oa*cbf_oa, lb=-b_oa*cbf_oa, ub=np.inf)
                constraint_arena = LinearConstraint(A_arena, lb=-b_arena, ub=np.inf)
                
                #Define objective function
                def objective_function(u, u_n):
                    return np.linalg.norm(u - u_n)**2
                
                #Construct the problem
                u = minimize(
                    objective_function,
                    x0=u_n,
                    args=(u_n,),
                    constraints=[constraint_cm, constraint_oa, constraint_arena],
                )

                #-------------
                # Send output
                #-------------
                for i in range(number_robots):
                    vel_cmd_msg[i].linear.x = u.x[2*i] * gains[0]
                    vel_cmd_msg[i].linear.y = u.x[2*i+1] * gains[1]
                    vel_cmd_msg[i].angular.z = u_nom_heading[i] * gains[2]

                    #Publish controller output
                    controller_msg.x = u.x[2*i]
                    controller_msg.y = u.x[2*i+1]
                    controller_pub[i].publish(controller_msg)

            #Else stop robot
            else:
                for i in range(number_robots):
                    vel_cmd_msg[i].linear.x = 0
                    vel_cmd_msg[i].linear.y = 0
                    vel_cmd_msg[i].angular.z = 0

            #------------------------------------------
            # Publish command & controller output norm
            #------------------------------------------
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
    #          Callback function 
    #         for huil controller
    #=====================================
    def huil_callback(self, twist_msg):
        #Save huil controller twist as class variable
        self.vel_huil = twist_msg

    #=====================================
    #        Function to calculate 
    #       the CBF safety function
    #=====================================
    def cbf_h(self, p_i, p_j, safe_distance, dir):
        # Dir 1 corresponds to CM and -1 to OA
        return dir*(safe_distance**2 - np.linalg.norm(p_i - p_j)**2)

    #=====================================
    #  Function to calculate the gradient 
    #     of the CBF safety function
    #=====================================
    def cbf_gradh(self, p_i, p_j, dir):
        # Dir 1 corresponds to CM and -1 to OA
        return dir*(-2*np.array([[p_i[0]-p_j[0]], [p_i[1]-p_j[1]]]))


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
    k3_cbf_huil = K3CBFExtraHuIL()
    try:
        rospy.spin()
    except ValueError as e:
        rospy.logerr(e)
        sys.exit(0)
    except rospy.ROSInterruptException:
        print("Shutting down")
        sys.exit(0)
