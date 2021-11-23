#!/usr/bin/env python
#=====================================
#       Module for subscribing
#       to the data outputted
#         by the auto node
#         and saving it in
#          a pickle file
#=====================================

import rospy
import sys
import geometry_msgs.msg
from std_msgs.msg import Float64
import pandas as pd
import numpy as np

import matplotlib.pyplot as plt 


class CBFLogger():
    
    #=====================================
    #         Class constructor
    #  Initializes node and subscribers
    #=====================================
    def __init__(self):
        #----------------
        # Initialisation 
        #----------------
        #Initialize node
        rospy.init_node('logger')

        #What to do on shutdown
        rospy.on_shutdown(self.save_files)      

        #Variable to decide if the motion arrows will be shown (1 yes/0 no)
        show_motion = rospy.get_param('~show_motion')

        #Get the robots number from parameters
        robots_number = rospy.get_param('/robots_number')
        number_robots = len(robots_number)

        #Get neighbour numbers from parameters
        neighbours = rospy.get_param('/neighbours')
        number_neighbours = []
        for i in range(number_robots):
            number_neighbours.append(len(neighbours[i]))

        #Create edge list for the name of columns as well as robot name columns
        edges = []
        robot_col = ['Time']
        for i in range(number_robots):
            for j in neighbours[i]:
                if (i+1,j) not in edges and (j,i+1) not in edges:
                    edges.append((i+1,j))
            robot_col.append("Robot_x"+str(i+1))
            robot_col.append("Robot_y"+str(i+1))

        edges_col = ['Time']
        for i in range(len(edges)):
            edges_col.append("Edge"+str(edges[i]))

        #Open files for writing the data
        global cbf_cm_filename
        cbf_cm_filename  = rospy.get_param('~cbf_cm_filename')
        global cbf_oa_filename
        cbf_oa_filename = rospy.get_param('~cbf_oa_filename')
        global controller_filename
        controller_filename = rospy.get_param('~controller_filename')
        global nom_controller_filename
        nom_controller_filename = rospy.get_param('~nom_controller_filename')

        #Create dataframes to pickle the data
        global df_cbf_cm
        df_cbf_cm = pd.DataFrame(columns=edges_col)
        global df_cbf_oa
        df_cbf_oa = pd.DataFrame(columns=edges_col)
        global df_controller
        df_controller = pd.DataFrame(columns=robot_col)
        global df_nom_controller
        df_nom_controller = pd.DataFrame(columns=robot_col)  

        #Setup controller output norm subscriber and init output
        self.controller = [0.]
        self.nom_controller = [0.]
        for i in range(number_robots):
            rospy.Subscriber("/controller"+str(i+1), geometry_msgs.msg.Vector3, self.controller_callback, i)
            rospy.Subscriber("/nom_controller"+str(i+1), geometry_msgs.msg.Vector3, self.nom_controller_callback, i)
            self.controller.append(0.)
            self.controller.append(0.)
            self.nom_controller.append(0.)
            self.nom_controller.append(0.)

        #Setup cbf functions subscriber and init output
        self.cbf_cm = [0.]
        self.cbf_oa = [0.]
        for i in range(len(edges)):
            rospy.Subscriber("/cbf_function_cm"+"/".join(map(str,edges[i])), Float64, self.cbf_cm_callback, i)
            rospy.Subscriber("/cbf_function_oa"+"/".join(map(str,edges[i])), Float64, self.cbf_oa_callback, i)
            self.cbf_cm.append(0.)
            self.cbf_oa.append(0.)

        #Setup HuIL controller subscriber
        rospy.Subscriber("/HuIL/key_vel", geometry_msgs.msg.Twist, self.huil_callback)

        #Init HuIL controller velocity
        self.vel_huil = geometry_msgs.msg.Twist()

        if show_motion:
        #-----------------------------------
        # Create plot for controller motion
        #-----------------------------------
            plt.ion()

            fig = plt.figure(figsize=(6, 5))
            ax = fig.add_subplot(111)
            img = plt.imread("/home/viktornfa/catkin_ws/src/master_thesis_victor/first_task_formation_cbf/src/nexus.png")
            ax.imshow(img)
            ax.set_title("HuIL controller and CBF-QP controller output for Robot 5")
            ax.axis('off')

            arrow1 = plt.arrow(x=579, y=373, dx=500*0, dy=500*0, width=10, facecolor='red', edgecolor='none')
            arrow2 = plt.arrow(x=579, y=373, dx=500*0, dy=500*0, width=10, facecolor='blue', edgecolor='none')

            plt.legend([arrow1, arrow2,], ['HuIL', 'CBF-QP',])

            ax.plot(579, 373, '-ko')

        #-----------------------------------------------------------------
        # Loop at set frequency and publish position command if necessary
        #-----------------------------------------------------------------
        #Loop frequency in Hz
        loop_frequency = 50
        r = rospy.Rate(loop_frequency)

        initial_time = rospy.Time.now()

        rospy.loginfo("STARTING LOGGING!")

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            time = now.to_sec() - initial_time.to_sec()
            #------------------------
            # Save data in dataframe
            #------------------------
            #CBF functions
            self.cbf_cm[0] = time
            df2_cbf_cm = pd.DataFrame(np.array([self.cbf_cm]), columns=edges_col)
            df_cbf_cm = df_cbf_cm.append(df2_cbf_cm, ignore_index=True)
            self.cbf_oa[0] = time
            df2_cbf_oa = pd.DataFrame(np.array([self.cbf_oa]), columns=edges_col)
            df_cbf_oa = df_cbf_oa.append(df2_cbf_oa, ignore_index=True)
            
            #Final controller
            self.controller[0] = time
            df2_controller = pd.DataFrame(np.array([self.controller]), columns=robot_col)
            df_controller = df_controller.append(df2_controller, ignore_index=True)

            #Nominal controller
            self.nom_controller[0] = time
            df2_nom_controller = pd.DataFrame(np.array([self.nom_controller]), columns=robot_col)
            df_nom_controller = df_nom_controller.append(df2_nom_controller, ignore_index=True)

            if show_motion == 1:
                #------------------------
                # Save data in dataframe
                #------------------------
                arrow1.remove()
                arrow2.remove()
                ux = self.controller[-2]
                uy = self.controller[-1]
                uhuilx = self.vel_huil.linear.x
                uhuily = self.vel_huil.angular.z
                arrow1 = plt.arrow(x=579, y=373, dx=-300*uhuily, dy=-300*uhuilx, width=10, facecolor='red', edgecolor='none')
                arrow2 = plt.arrow(x=579, y=373, dx=-300*uy, dy=-300*ux, width=10, facecolor='blue', edgecolor='none')
                fig.canvas.draw()
                fig.canvas.flush_events()

            #---------------------------------
            # Sleep to respect loop frequency
            #---------------------------------
            r.sleep()

    #=====================================
    #          Callback function 
    #          for cbf functions
    #=====================================
    def cbf_cm_callback(self, float64_msg, i):
        #Save cbf for comunication maintenance 
        self.cbf_cm[i+1] = float64_msg.data

    def cbf_oa_callback(self, float64_msg, i):
        #Save cbf for obstacle avoidance
        self.cbf_oa[i+1] = float64_msg.data

    #=====================================
    #          Callback function 
    #     for controller output norm
    #=====================================
    def controller_callback(self, vector3_msg, i):
        #Save final controller in vector form 
        self.controller[2*i+1] = vector3_msg.x
        self.controller[2*i+2] = vector3_msg.y

    def nom_controller_callback(self, vector3_msg, i):
        #Save nominal controller in vector form 
        self.nom_controller[2*i+1] = vector3_msg.x
        self.nom_controller[2*i+2] = vector3_msg.y

    #=====================================
    #          Callback function 
    #         for huil controller
    #=====================================
    def huil_callback(self, twist_msg):
        #Save huil controller twist as class variable
        self.vel_huil = twist_msg

    #=====================================
    #     Function to close all files
    #=====================================
    def save_files(self):
        #Save data
        df_cbf_cm.to_csv(cbf_cm_filename, index=False)
        df_cbf_oa.to_csv(cbf_oa_filename, index=False)
        df_controller.to_csv(controller_filename, index=False)
        df_nom_controller.to_csv(nom_controller_filename, index=False)

        rospy.loginfo("FINISHED LOGGING!")

#=====================================
#               Main
#=====================================
if __name__ == "__main__":
    logger = CBFLogger()
    try:
        rospy.spin()
    except ValueError as e:
        rospy.logerr(e)
        sys.exit(0)
    except rospy.ROSInterruptException:
        print("Shutting down")
        sys.exit(0)