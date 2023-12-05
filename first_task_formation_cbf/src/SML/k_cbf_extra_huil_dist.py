#!/usr/bin/env python
#=====================================
#       Formation keeping with
#     communication maintenance
#       or obstacle avoidance 
#      for the SML Nexus robot
#    done in a distributed manner
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


class KCBFExtraHuILDist():
    #=====================================
    #         Class constructor
    #  Initializes node and subscribers
    #=====================================
    def __init__(self):
        #----------------
        # Initialisation 
        #----------------
        #Initialize node
        rospy.init_node('k_cbf_extra_huil_dist')

        #Get the robots number from parameters
        robots_number = rospy.get_param('/robots_number')
        number_robots = len(robots_number)

        # Create safety constraint for arena
        xmax = 1.6
        xmin = -1.7
        ymax = 2.0
        ymin = -2.3
        #Arena walls (defined as a rectangle clockwise starting from the top) 
        #Defined as [wall size, axis(1 is y, 0 is x), direction(1 is positive, -1 neg)]
        walls = [[ymax, 1, 1], [xmax, 0, 1], [ymin, 1, -1], [xmin, 0, -1]]
        wall_grad = np.transpose(np.array([[0, -1], [-1, 0], [0, 1], [1, 0]]))
        
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
        x_d = np.array(formation_positions)

        #Get parameter representing communication maintenance and/or collision avoidance
        cm = rospy.get_param('/cm')
        oa = rospy.get_param('/oa')
        arena = rospy.get_param('/arena')
        obstacle = rospy.get_param('/obstacle')
        extra = rospy.get_param('/extra')

        #Maximum/minimum safe distance for CBF
        d_cm = rospy.get_param('/d_cm')
        d_oa = rospy.get_param('/d_oa')
        d_obstacle = rospy.get_param('/d_obstacle')
        d_extra = rospy.get_param('/d_extra')
        #if d_oa < 2*0.27:
        #    d_oa = 2*0.27
        #if d_oa < 0.27:
        #    d_oa = 0.27

        #See if HuIL controller is activated and which robot it affects
        huil = rospy.get_param('/huil')
        human_robot = rospy.get_param('/human_robot')

        #Dimension of the problem
        dim = 2

        #Controller gains (for x, y, heading)
        gains = (2.0, 2.0, 2.0)

        #CBF constraint parameters
        alpha = 100

        #Exponential parameter
        p = 10

        #Adaptative law parameter
        k0 = 20

        #Maximum value of control input
        u_max = 0.05
        u_min = -u_max

        #Parameter for the sign filter
        filter_param = 5

        #Max speed of HIL
        vxe = 0.8
        vye = 0.8

        #Obstacle point
        obstacle_x = np.array([-0.13, -0.14])

        #Calculate the number of total constraints
        num_constraints = cm*len(edges)+oa*len(edges)+arena*len(walls)*number_robots+extra*number_robots+obstacle*number_robots
        if num_constraints == 0:
            num_constraints = 1

        #Initialize auxiliary variables
        y = np.zeros((number_robots, 1))
        c = np.zeros((number_robots, 1))
        # Create a counter for the times a=0 error happen
        a_counter = np.zeros((number_robots,1))

        #Init robot pose
        self.robot_pose = []
        #Init last received pose time
        self.last_received_robot_pose = []

        #For each robot
        for i in range(number_robots):
            self.robot_pose.append(geometry_msgs.msg.PoseStamped())
            self.last_received_robot_pose.append(rospy.Time())

        #Init extra-HuIL robot pose
        self.huil_robot_pose = geometry_msgs.msg.PoseStamped()
        #Init extra-HuIL robot last received pose time
        self.huil_last_received_robot_pose = rospy.Time()

        #Init HuIL controller velocity
        self.vel_huil = geometry_msgs.msg.Twist()

        #Timeout in seconds
        timeout = 0.5

        #Booleans to check is positions have been received
        init_pose = []
        for i in range(number_robots):
            init_pose.append(bool())

        #Boolean to check if extra-HuIL position has been received
        huil_init_pose = False

        #Setup robot pose subscriber
        for i in range(number_robots):
            rospy.Subscriber("/qualisys/nexus"+str(robots_number[i]-1)+"/pose", geometry_msgs.msg.PoseStamped, self.robot_pose_callback, i)
        
        #Setup extra-HuIL robot pose subscriber
        rospy.Subscriber("/qualisys/nexus"+str(human_robot-1)+"/pose", geometry_msgs.msg.PoseStamped, self.huil_robot_pose_callback)

        #Setup HuIL controller subscriber
        #rospy.Subscriber("/HuIL/key_vel", geometry_msgs.msg.Twist, self.huil_callback)

        #Setup velocity command publisher
        vel_pub = []
        for i in range(number_robots):
            vel_pub.append(rospy.Publisher("/nexus"+str(robots_number[i]-1)+"/cmd_vel", geometry_msgs.msg.Twist, queue_size=100))

        #Setup cbf functions publisher
        cbf_cm_pub = []
        cbf_oa_pub = []
        for i in range(len(edges)):
            cbf_cm_pub.append(rospy.Publisher("/cbf_function_cm"+"/".join(map(str,edges[i])), Float64, queue_size=100))
            cbf_oa_pub.append(rospy.Publisher("/cbf_function_oa"+"/".join(map(str,edges[i])), Float64, queue_size=100))
        #Setup message type
        cbf_cm_msg = Float64()
        cbf_oa_msg = Float64()

        #Setup cbf arena and obstacle, controller output publishers and messages
        nom_controller_pub = []
        controller_pub = []
        cbf_arena_top_pub = []
        cbf_arena_right_pub = []
        cbf_arena_bottom_pub = []
        cbf_arena_left_pub = []
        cbf_obstacle_pub = []
        cbf_extra_pub = []
        for i in range(number_robots):
            nom_controller_pub.append(rospy.Publisher("/nom_controller"+str(i+1), geometry_msgs.msg.Vector3, queue_size=100))
            controller_pub.append(rospy.Publisher("/controller"+str(i+1), geometry_msgs.msg.Vector3, queue_size=100))
            cbf_arena_top_pub.append(rospy.Publisher("/cbf_function_arena_top"+str(i+1), Float64, queue_size=100))
            cbf_arena_right_pub.append(rospy.Publisher("/cbf_function_arena_right"+str(i+1), Float64, queue_size=100))
            cbf_arena_bottom_pub.append(rospy.Publisher("/cbf_function_arena_bottom"+str(i+1), Float64, queue_size=100))
            cbf_arena_left_pub.append(rospy.Publisher("/cbf_function_arena_left"+str(i+1), Float64, queue_size=100))
            cbf_obstacle_pub.append(rospy.Publisher("/cbf_function_obstacle"+str(i+1), Float64, queue_size=100))
            cbf_extra_pub.append(rospy.Publisher("/cbf_function_extra"+str(i+1), Float64, queue_size=100))
        #Setup message type
        nom_controller_msg = geometry_msgs.msg.Vector3()
        controller_msg = geometry_msgs.msg.Vector3()
        cbf_arena_top_msg = Float64()
        cbf_arena_right_msg = Float64()
        cbf_arena_bottom_msg = Float64()
        cbf_arena_left_msg = Float64()
        cbf_obstacle_msg = Float64()
        cbf_extra_msg = Float64()

        #Setup transform subscriber
        tf_buffer = []
        tf_listener = []
        for i in range(number_robots):
            tf_buffer.append(tf2_ros.Buffer())
            tf_listener.append(tf2_ros.TransformListener(tf_buffer[i]))
        
            #Wait for transform to be available
            while not tf_buffer[i].can_transform('mocap', "nexus"+str(robots_number[i]-1), rospy.Time()):
                rospy.loginfo("Wait for transform to be available for nexus"+str(robots_number[i]-1))
                rospy.sleep(1)

        #Setup extra-HuIL transform subscriber
        huil_tf_buffer = tf2_ros.Buffer()
        huil_tf_listener = tf2_ros.TransformListener(huil_tf_buffer)        
        #Wait for transform to be available
        while not huil_tf_buffer.can_transform('mocap', "nexus"+str(human_robot-1), rospy.Time()):
            rospy.loginfo("Wait for transform to be available for nexus"+str(human_robot-1))
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

        rospy.sleep(1)

        rospy.loginfo("CBF-Formation controller Distributed Initialized for nexus"+str(robots_number)+
                      " with neighbours "+str(neighbours)+" with ideal positions "+str(formation_positions)+
                      " and CBF comunication maintenance "+str(cm)+" with distance "+str(d_cm)+
                      " and obstacle avoidance "+str(oa)+" with distance "+str(d_oa)+
                      " as well as an extra HuIL robot "+str(human_robot)
                      )

        while not rospy.is_shutdown():
            #Run controller if robot position and neighbours position have been received
            now  = rospy.Time.now()
            for i in range(number_robots):
                init_pose[i] = (now.to_sec() < self.last_received_robot_pose[i].to_sec() + timeout)

            huil_init_pose = (now.to_sec() < self.huil_last_received_robot_pose.to_sec() + timeout)

            if all(init_pose) and huil_init_pose:

                #---------------------------------------
                # Get position and orientation matrices
                #---------------------------------------
                x = np.zeros((number_robots, 2))
                heading = np.zeros((number_robots))
                for i in range(number_robots):
                    #Get x and y position from robot pose
                    x[i, 0] = self.robot_pose[i].pose.position.x
                    x[i, 1] = self.robot_pose[i].pose.position.y

                    #Get euler angle from robot pose quaternion
                    (roll, pitch, yaw) = euler_from_quaternion([self.robot_pose[i].pose.orientation.x,
                                                                self.robot_pose[i].pose.orientation.y,
                                                                self.robot_pose[i].pose.orientation.z,
                                                                self.robot_pose[i].pose.orientation.w])
                    heading[i] = yaw

                dist_p = x - x_d

                #Position of the extra-HuIL robot
                huil_x = np.array([self.huil_robot_pose.pose.position.x, self.huil_robot_pose.pose.position.y])

                #------------------------------------
                # Compute nominal (auto) controllers
                #------------------------------------
                u_nom = np.dot(-L_G, dist_p)
                #For pointing in the same direction
                # u_nom_heading = np.dot(-L_G, heading)
                #In case mecanum-wheels friction is causing unwanted rotations (all of them pointing forward)
                u_nom_heading = -heading

                #Convert nominal controller to CBF controller format and add HuIL
                u_n = np.zeros((number_robots*dim))
                for i in range(number_robots):
                    #For the robot controlled with HuIL
                    if i == human_robot-1:
                        u_n[2*i] = huil*self.vel_huil.linear.x + u_nom[i, 0]
                        u_n[2*i+1] = huil*self.vel_huil.angular.z + u_nom[i, 1]
                    else:
                        u_n[2*i] = u_nom[i, 0]
                        u_n[2*i+1] = u_nom[i, 1]

                    #Publish nominal controller
                    nom_controller_msg.x = u_n[2*i]
                    nom_controller_msg.y = u_n[2*i+1]
                    nom_controller_pub[i].publish(nom_controller_msg)

                #Compute CBF constrained controller - Distributed
                u = np.zeros((dim*number_robots, 1))
                c = np.zeros((number_robots, 1))
                a = np.zeros((dim*number_robots, 1))
                b = np.zeros((number_robots, 1))
                for i in range(number_robots):

                    # Collective constraints
                    for e in range(len(edges)):
                        aux_i = edges[e][0]-1
                        aux_j = edges[e][1]-1

                        if i == aux_i:
                            # CM
                            a[2*i:2*i+2] += cm*np.nan_to_num(-p*np.exp(-p*self.cbf_h(x[aux_i], x[aux_j], d_cm, 1))*self.cbf_gradh(x[aux_i], x[aux_j], 1))
                            b[i] += cm*np.nan_to_num(-alpha/2*(1/num_constraints-np.exp(-p*self.cbf_h(x[aux_i], x[aux_j], d_cm, 1))))
                            # OA
                            a[2*i:2*i+2] += oa*np.nan_to_num(-p*np.exp(-p*self.cbf_h(x[aux_i], x[aux_j], d_oa, -1))*self.cbf_gradh(x[aux_i], x[aux_j], -1))
                            b[i] += oa*np.nan_to_num(-alpha/2*(1/num_constraints-np.exp(-p*self.cbf_h(x[aux_i], x[aux_j], d_oa, -1))))
                        elif i == aux_j:
                            # CM
                            a[2*i:2*i+2] += cm*np.nan_to_num(-p*np.exp(-p*self.cbf_h(x[aux_i], x[aux_j], d_cm, 1))*self.cbf_gradh(x[aux_j], x[aux_i], 1))
                            b[i] += cm*np.nan_to_num(-alpha/2*(1/num_constraints-np.exp(-p*self.cbf_h(x[aux_i], x[aux_j], d_cm, 1))))
                            # OA
                            a[2*i:2*i+2] += oa*np.nan_to_num(-p*np.exp(-p*self.cbf_h(x[aux_i], x[aux_j], d_oa, -1))*self.cbf_gradh(x[aux_j], x[aux_i], -1))
                            b[i] += oa*np.nan_to_num(-alpha/2*(1/num_constraints-np.exp(-p*self.cbf_h(x[aux_i], x[aux_j], d_oa, -1))))
                        else:
                            a[2*i:2*i+2] += np.zeros((dim, 1))
                            b[i] += 0

                    # Individual constraints
                    for k in range(len(walls)):
                        # Arena walls
                        a[2*i:2*i+2] += arena*np.nan_to_num(-p*np.exp(-p*self.cbf_walls(x[i], walls[k]))*wall_grad[:,k].reshape(2, 1))
                        b[i] += arena*np.nan_to_num(-alpha*(1/num_constraints-np.exp(-p*self.cbf_walls(x[i], walls[k]))))
                    # Mid-point obstacle
                    a[2*i:2*i+2] += obstacle*np.nan_to_num(-p*np.exp(-p*self.cbf_h(x[i], obstacle_x, d_obstacle, -1))*self.cbf_gradh(x[i], obstacle_x, -1))
                    b[i] += obstacle*np.nan_to_num(-alpha*(1/num_constraints-np.exp(-p*self.cbf_h(x[i], obstacle_x, d_obstacle, -1))))
                    # Extra robot avoidance
                    a[2*i:2*i+2] += extra*np.nan_to_num(-p*np.exp(-p*self.cbf_h(x[i], huil_x, d_extra, -1))*self.cbf_gradh(x[i], huil_x, -1))
                    b[i] += extra*np.nan_to_num(-alpha*(1/num_constraints-np.exp(-p*self.cbf_h(x[i], huil_x, d_extra, -1))) -
                        p*np.exp(-p*self.cbf_h(x[i], huil_x, d_extra, -1))*np.dot(np.transpose(self.cbf_gradh(huil_x, x[i], -1)),np.array([vxe,vye])))

                    #Publish cbf arena and obstacle function message
                    cbf_arena_top_msg = self.cbf_walls(x[i], walls[0])
                    cbf_arena_right_msg = self.cbf_walls(x[i], walls[1])
                    cbf_arena_bottom_msg = self.cbf_walls(x[i], walls[2])
                    cbf_arena_left_msg = self.cbf_walls(x[i], walls[3])
                    cbf_obstacle_msg = self.cbf_h(x[i], obstacle_x, d_obstacle, -1)
                    cbf_extra_msg = self.cbf_h(x[i], huil_x, d_extra, -1)
                    cbf_arena_top_pub[i].publish(cbf_arena_top_msg)
                    cbf_arena_right_pub[i].publish(cbf_arena_right_msg)
                    cbf_arena_bottom_pub[i].publish(cbf_arena_bottom_msg)
                    cbf_arena_left_pub[i].publish(cbf_arena_left_msg)
                    cbf_obstacle_pub[i].publish(cbf_obstacle_msg)
                    cbf_extra_pub[i].publish(cbf_extra_msg)

                    u[2*i:2*i+2] = np.expand_dims(u_n[2*i:2*i+2], axis=1)
                    if a[2*i] == 0 and a[2*i+1] == 0:
                        a_counter[i] += 1
                    else:
                        c[i] = (np.dot(L_G[i,:],y) + 
                            np.dot(np.transpose(a[2*i:2*i+2]),u_n[2*i:2*i+2])+b[i])/np.dot(np.transpose(a[2*i:2*i+2]),a[2*i:2*i+2])
                        u[2*i:2*i+2] -=  np.maximum(0,c[i])*a[2*i:2*i+2]

                u = np.squeeze(u, axis=1)
                
                # Update slack variable
                for i in range(number_robots):
                    if a[2*i] == 0 and a[2*i+1] == 0:
                        y[i] = 0
                    else:
                        y[i] = y[i] - np.transpose(k0*self.sign_filter(np.dot(L_G[i,:],c), filter_param))*(1/loop_frequency)

                #Publish cbf CM and OA function message
                for e in range(len(edges)):
                    aux_i = edges[e][0]-1
                    aux_j = edges[e][1]-1
            
                    cbf_cm_msg = self.cbf_h(x[aux_i], x[aux_j], d_cm, 1)
                    cbf_oa_msg = self.cbf_h(x[aux_i], x[aux_j], d_oa, -1)
                    cbf_cm_pub[e].publish(cbf_cm_msg)
                    cbf_oa_pub[e].publish(cbf_oa_msg)

                # Bound the control input
                for i in range(len(u)):
                    u[i] = max(u_min, min(u_max, u[i]))

                #-------------
                # Send output
                #-------------
                for i in range(number_robots):
                    vel_cmd_msg[i].linear.x = u[2*i] * gains[0]
                    vel_cmd_msg[i].linear.y = u[2*i+1] * gains[1]
                    vel_cmd_msg[i].angular.z = u_nom_heading[i] * gains[2]

                    #Publish controller output
                    controller_msg.x = u[2*i]
                    controller_msg.y = u[2*i+1]
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
                    transform.append(tf_buffer[i].lookup_transform('mocap', "nexus"+str(robots_number[i]-1), rospy.Time()))
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
    #    for HuIL robot pose feedback
    #=====================================
    def huil_robot_pose_callback(self, pose_stamped_msg):
        #Save robot pose as class variable
        self.huil_robot_pose = pose_stamped_msg

        #Save when last pose was received
        self.huil_last_received_robot_pose = rospy.Time.now()

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
    #           for CM and OA
    #=====================================
    def cbf_h(self, p_i, p_j, safe_distance, dir):
        # Dir 1 corresponds to CM and -1 to OA
        return dir*(safe_distance**2 - np.linalg.norm(p_i - p_j)**2)

    #=====================================
    #  Function to calculate the gradient 
    #     of the CBF safety function
    #           for CM and OA
    #=====================================
    def cbf_gradh(self, p_i, p_j, dir):
        # Dir 1 corresponds to CM and -1 to OA
        return dir*(-2*np.array([[p_i[0]-p_j[0]], [p_i[1]-p_j[1]]]))

    #=====================================
    #        Function to calculate 
    #       the CBF safety function
    #          for arena limits
    #=====================================
    def cbf_walls(self, p, wall):
        return wall[2]*(wall[0]-p[wall[1]])

    #=====================================
    #        Function to filter
    #        the sign operator
    #=====================================
    def sign_filter(self, x, a):
        if x >= a:
            sol = 1
        elif x <= a:
            sol = -1
        else:
            sol = 1/a*x

        return sol


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
    k_cbf_extra_huil_dist = KCBFExtraHuILDist()
    try:
        rospy.spin()
    except ValueError as e:
        rospy.logerr(e)
        sys.exit(0)
    except rospy.ROSInterruptException:
        print("Shutting down")
        sys.exit(0)
