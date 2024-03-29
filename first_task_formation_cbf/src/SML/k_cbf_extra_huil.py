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


class KCBFExtraHuIL():
    #=====================================
    #         Class constructor
    #  Initializes node and subscribers
    #=====================================
    def __init__(self):
        #----------------
        # Initialisation 
        #----------------
        #Initialize node
        rospy.init_node('k_cbf_extra_huil')

        #Get the robots number from parameters
        robots_number = rospy.get_param('/robots_number')
        number_robots = len(robots_number)

        # Create safety constraint for arena
        As = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
        A_arena = np.zeros((number_robots*4, number_robots*2))
        for i in range(number_robots):
            A_arena[4*i:4*i+4, 2*i:2*i+2] = As
        b_arena = np.zeros((number_robots*4))
        xmax = 1.5
        xmin = -1.7
        ymax = 1.9
        ymin = -2.3

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
        safe_distance_obstacle = rospy.get_param('/safe_distance_obstacle')
        #if safe_distance_oa < 2*0.27:
        #    safe_distance_oa = 2*0.27
        safe_distance_extra = rospy.get_param('/safe_distance_extra')
        if safe_distance_extra < 2*0.27:
            safe_distance_extra = 2*0.27

        #See if HuIL controller is activated and which robot it affects
        #This robot will also be the extra robot
        huil = rospy.get_param('/huil')
        human_robot = rospy.get_param('/human_robot')

        #Dimension of the problem
        n = 2

        #Controller gains (for x, y, heading)
        gains = (1, 1, 1)

        #CBF constraint parameters
        alfa = 1
        alfa_extra = 1

        #Obstacle point
        p_obs = np.array([-0.13, -0.14])

        #Max speed of HIL
        vxe = 0.8
        vye = 0.8

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

        #Booleans to check if positions have been received
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

        #Setup extra-HuIL robot velocity command publisher
        #huil_vel_pub = rospy.Publisher("/nexus"+str(human_robot-1)+"/cmd_vel", geometry_msgs.msg.Twist, queue_size=100)

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

        #Create a ROS Twist message for extra-HuIL velocity command
        #huil_vel_cmd_msg = geometry_msgs.msg.Twist()

        #-----------------------------------------------------------------
        # Loop at set frequency and publish position command if necessary
        #-----------------------------------------------------------------
        #Loop frequency in Hz
        loop_frequency = 50
        r = rospy.Rate(loop_frequency)

        rospy.sleep(1)

        rospy.loginfo("CBF-Formation controller Centralized Initialized for nexus"+str(robots_number)+
                      " with neighbours "+str(neighbours)+" with ideal positions "+str(formation_positions)+
                      " and CBF comunication maintenance "+str(cbf_cm)+" with distance "+str(safe_distance_cm)+
                      " and obstacle avoidance "+str(cbf_oa)+" with distance "+str(safe_distance_oa)+
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
                p = np.zeros((number_robots, n))
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

                #Position of the extra-HuIL robot
                huil_p = np.array([self.huil_robot_pose.pose.position.x, self.huil_robot_pose.pose.position.y])

                #------------------------------------
                # Compute nominal (auto) controllers
                #------------------------------------
                u_nom = np.dot(-L_G, dist_p)
                #For pointing in the same direction
                # u_nom_heading = np.dot(-L_G, heading)
                #In case mecanum-wheels friction is causing unwanted rotations
                u_nom_heading = - heading

                #Get euler angle from robot pose quaternion
                (roll, pitch, huil_heading) = euler_from_quaternion([self.huil_robot_pose.pose.orientation.x,
                                                                    self.huil_robot_pose.pose.orientation.y,
                                                                    self.huil_robot_pose.pose.orientation.z,
                                                                    self.huil_robot_pose.pose.orientation.w])

                #For the extra-HuIL robot, set its heading to just the first robot
                #huil_u_nom_heading = - (huil_heading - heading[0])

                #Convert nominal controller to CBF controller format and CBF for extra-HuIL robot
                A_extra = np.zeros((number_robots, number_robots*n))
                b_extra = np.zeros((number_robots))
                u_n = np.zeros((number_robots*n))
                for i in range(number_robots):
                    u_n[2*i] = u_nom[i, 0]
                    u_n[2*i+1] = u_nom[i, 1]

                    #Publish nominal controller
                    nom_controller_msg.x = u_n[2*i]
                    nom_controller_msg.y = u_n[2*i+1]
                    nom_controller_pub[i].publish(nom_controller_msg)

                    A_extra[i, 2*i:2*i+2] = 2*np.array([p[i, 0]-huil_p[0], p[i, 1]-huil_p[1]])
                    b_extra[i] = alfa_extra*(self.cbf_h(p[i], huil_p, safe_distance_extra, -1)-safe_distance_extra**2-2*(p[i, 0]-huil_p[0])*vxe-2*(p[i, 1]-huil_p[1])*vye)
                    #Publish cbf function message
                    cbf_extra_msg = b_extra[i]
                    cbf_extra_pub[i].publish(cbf_extra_msg)

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
                    #Publish cbf CM and OA function message
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

                A_obstacle = np.zeros((number_robots, number_robots*n))
                b_obstacle = np.zeros((number_robots))
                #Calculate CBF for arena and obstacle safety
                for i in range(number_robots):
                    #Obstacle
                    b_obstacle[i] = alfa*self.cbf_h(p[i], p_obs, safe_distance_obstacle, -1)
                    grad_h_value_obstacle = np.transpose(self.cbf_gradh(p[i], p_obs, -1))
                    A_obstacle[i, 2*i:2*i+2] = grad_h_value_obstacle
                    #Arena
                    b_arena[4*i] = alfa*(xmax - p[i, 0])
                    b_arena[4*i+1] = alfa*(p[i, 0] - xmin)
                    b_arena[4*i+2] = alfa*(ymax - p[i, 1])
                    b_arena[4*i+3] = alfa*(p[i, 1] - ymin)

                    #Publish cbf arena and obstacle function message
                    cbf_arena_top_msg = b_arena[4*i]
                    cbf_arena_right_msg = b_arena[4*i+1]
                    cbf_arena_bottom_msg = b_arena[4*i+2]
                    cbf_arena_left_msg = b_arena[4*i+3]
                    cbf_obstacle_msg = b_obstacle[i]
                    cbf_arena_top_pub[i].publish(cbf_arena_top_msg)
                    cbf_arena_right_pub[i].publish(cbf_arena_right_msg)
                    cbf_arena_bottom_pub[i].publish(cbf_arena_bottom_msg)
                    cbf_arena_left_pub[i].publish(cbf_arena_left_msg)
                    cbf_obstacle_pub[i].publish(cbf_obstacle_msg)

                #----------------------------
                # Solve minimization problem
                #----------------------------
                #Define linear constraints
                constraint_cm = LinearConstraint(A_cm*cbf_cm, lb=-b_cm*cbf_cm, ub=np.inf)
                constraint_oa = LinearConstraint(A_oa*cbf_oa, lb=-b_oa*cbf_oa, ub=np.inf)
                constraint_arena = LinearConstraint(A_arena, lb=-b_arena, ub=np.inf)
                constraint_obstacle = LinearConstraint(A_obstacle, lb=-b_obstacle, ub=np.inf)
                constraint_extra = LinearConstraint(A_extra, lb=-b_extra, ub=np.inf)
                
                #Define objective function
                def objective_function(u, u_n):
                    return np.linalg.norm(u - u_n)**2
                
                #Construct the problem
                u = minimize(
                    objective_function,
                    x0=u_n,
                    args=(u_n,),
                    constraints=[constraint_cm, constraint_oa, constraint_arena, constraint_obstacle, constraint_extra],
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

                #For the extra-HuIL robot controlled
                #huil_vel_cmd_msg.linear.x = huil*self.vel_huil.linear.x * gains[0]
                #huil_vel_cmd_msg.linear.y = huil*self.vel_huil.angular.z * gains[1]
                #huil_vel_cmd_msg.angular.z = huil*huil_u_nom_heading * gains[2]

            #Else stop robot
            else:
                for i in range(number_robots):
                    vel_cmd_msg[i].linear.x = 0
                    vel_cmd_msg[i].linear.y = 0
                    vel_cmd_msg[i].angular.z = 0

                #For the extra-HuIL robot controlled
                #huil_vel_cmd_msg.linear.x = 0
                #huil_vel_cmd_msg.linear.y = 0
                #huil_vel_cmd_msg.angular.z = 0

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

                #For the extra-HuIL robot controlled
                #Get transform from mocap frame to robot frame
                #huil_transform = huil_tf_buffer.lookup_transform('mocap', "nexus"+str(human_robot-1), rospy.Time())
                #
                #huil_vel_cmd_msg_transformed = transform_twist(huil_vel_cmd_msg, huil_transform)
                #Publish cmd message
                #huil_vel_pub.publish(huil_vel_cmd_msg_transformed)
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
    k_cbf_huil = KCBFExtraHuIL()
    try:
        rospy.spin()
    except ValueError as e:
        rospy.logerr(e)
        sys.exit(0)
    except rospy.ROSInterruptException:
        print("Shutting down")
        sys.exit(0)
