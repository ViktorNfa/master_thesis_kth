#!/usr/bin/env python
#=====================================
#       Formation keeping with
#     communication maintenance
#       or obstacle avoidance 
#      for the SML Nexus robot
#               ----
#     Listen to both the robot pose 
#   and the other neighbouring robots 
#  from the mocap system and output a
#    velocity command to drive the
#   robot to keep the formation and
#  keep the communication bw robots or
#   avoid colliding with other robots
#=====================================
import sys
import rospy
import copy
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion

class CBFFormationController():
    #=====================================
    #         Class constructor
    #  Initializes node and subscribers
    #=====================================
    def __init__(self):
        #----------------
        # Initialisation 
        #----------------
        #Initialize node
        rospy.init_node('cbf_formation_controller')

        #Get the robot's name from parameters
        robot_name = rospy.get_param('~robot_name')

        #Get robot number from the name (works for numbers>9)
        robot_number = int("".join(map(str, [int(i) for i in list(robot_name) if i.isdigit()])))

        #Get tracked neighbour names from parameters
        neighbours = rospy.get_param('~neighbours')
        Ni = [i for i in neighbours if i]

        #Get number of neighbours and total number of agents
        number_neighbours = rospy.get_param('~number_neighbours')
        num_neighbours = number_neighbours[0]
        num_agents = number_neighbours[1]

        #Get the ideal positions of the formation
        formation_positions = rospy.get_param('~formation_positions')

        #Get parameter representing communication maintenance or collision avoidance
        cbf_direction = rospy.get_param('~cbf_direction')

        #Maximum safe distance for CBF
        safe_distance = rospy.get_param('~safe_distance')

        #Controller gains (for x, y, heading)
        gains = (1, 1, 1)

        #Init robot pose
        self.robot_pose = geometry_msgs.msg.PoseStamped()

        #Init neighbouring pose
        self.neighbour_pose = geometry_msgs.msg.PoseStamped()

        #Init last received pose time
        self.last_received_robot_pose = rospy.Time()

        #Init last received pose time
        self.last_received_neighbour = rospy.Time()

        #Timeout in seconds
        timeout = 0.5

        #Booleans to check is positions have been received
        init_pose = False
        init_neighbours = []
        init_neighbour = False

        #Setup robot pose subscriber
        rospy.Subscriber("/qualisys/"+robot_name+"/pose", geometry_msgs.msg.PoseStamped, self.robot_pose_callback)

        #Setup neighbour pose subscriber
        for i in range(num_neighbours):
            neighbour_number = int("".join(map(str, [int(j) for j in list(Ni[i]) if j.isdigit()])))
            rospy.Subscriber("/qualisys/"+Ni[i]+"/pose", geometry_msgs.msg.PoseStamped, self.neighbour_pose_callback, neighbour_number)

        #Setup velocity command publisher
        vel_pub = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=100)

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

        rospy.loginfo("CBF-Formation controller Initialized for "+robot_name+
                      " with "+str(num_neighbours)+ " neighbours "+str(Ni)+
                      " with ideal positions "+str(formation_positions)
                      )
        
        while not rospy.is_shutdown():
            #Run controller if robot position and neighbours position have been received
            now  = rospy.Time.now()
            init_pose = (now.to_sec() < self.last_received_robot_pose.to_sec() + timeout)
            for t in range(num_neighbours):
                init_neighbours[t] = (now.to_sec() < self.last_received_neighbour[t].to_sec() + timeout)
            init_neighbour = max(init_neighbours)

            if init_pose and init_neighbour:
                #-----------------------------------
                # Compute error on x and y position
                #-----------------------------------
                error_x = 0
                error_y = 0
                for i in range(num_neighbours):
                    neighbour_number = int("".join(map(str, [int(j) for j in list(Ni[i]) if j.isdigit()])))
                    error_x += self.neighbour_pose[neighbour_number-1].pose.position.x - self.robot_pose.pose.position.x + \
                        (formation_positions[robot_number-1][0] - formation_positions[neighbour_number-1][0])
                    error_y += self.neighbour_pose[neighbour_number-1].pose.position.y - self.robot_pose.pose.position.y  + \
                        (formation_positions[robot_number-1][1] - formation_positions[neighbour_number-1][1])

                #-----------------------
                # Compute heading error
                #-----------------------
                #Get euler angle from robot pose quaternion
                (roll, pitch, yaw) = euler_from_quaternion([self.robot_pose.pose.orientation.x,
                                                            self.robot_pose.pose.orientation.y,
                                                            self.robot_pose.pose.orientation.z,
                                                            self.robot_pose.pose.orientation.w])

                #Get euler angle from neighbours pose quaternion
                error_heading = 0
                for i in range(num_neighbours):
                    neighbour_number = int("".join(map(str, [int(j) for j in list(Ni[i]) if j.isdigit()])))
                    (neighbour_roll, neighbour_pitch, neighbour_yaw) = euler_from_quaternion([self.neighbour_pose[neighbour_number-1].pose.orientation.x,
                                                                              self.neighbour_pose[neighbour_number-1].pose.orientation.y,
                                                                              self.neighbour_pose[neighbour_number-1].pose.orientation.z,
                                                                              self.neighbour_pose[neighbour_number-1].pose.orientation.w])
                    error_heading += neighbour_yaw - yaw

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
    #    for neighbours pose feedback
    #=====================================
    def neighbour_pose_callback(self, pose_stamped_msg, neighbour_number):
        #Save neigbours pose as class variable
        self.neighbour_pose[neighbour_number-1] = pose_stamped_msg

        #Save when last neighbours pose was received
        self.last_received_neighbour_pose[neighbour_number-1] = rospy.Time.now()


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
    cbf_follower_controller = CBFFormationController()
    try:
        rospy.spin()
    except ValueError as e:
        rospy.logerr(e)
        sys.exit(0)
    except rospy.ROSInterruptException:
        print "Shutting down"
        sys.exit(0)