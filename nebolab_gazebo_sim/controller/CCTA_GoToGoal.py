#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose2D, Twist

import numpy as np
from functools import partial
import signal

from ..control_lib.cbf_single_integrator import cbf_si

# Variables
TB_L_SI2UNI = 0.06
ROBOT_COUNT = 1



ROS_NODE_NAME = 'CCTA_GoToInit'
class CCTAtoInit(Node):

    def __init__(self):
        super().__init__(ROS_NODE_NAME)

        # Default value, will be changed based on parameter
        self.USECBF_DYNAMICOBS = False
        self.gamma_dynamicObs = 1000
        self.ds_dyn = 0.4

        self.USECBF_STATICOBS = False
        self.gamma_staticObs = 1000
        self.obstacle = []

        # Allow changing scenarios from command line argument
        # By default scenario is 1
        self.declare_parameter('scene', 0)
        SCENARIO_MODE = self.get_parameter('scene')        
        self.evaluate_scenario(SCENARIO_MODE.value)

        # ROS
        self.ROS_RATE = 50

        # Handles robot-specific topics of one robot.
        self.poses_ahead = [None]*ROBOT_COUNT
        self.ros_pubs = []

        self.sensor_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        if self.USECBF_DYNAMICOBS: self.inNeigh = [None]*ROBOT_COUNT
        for robot_index in range(ROBOT_COUNT):
            tb_name = f'tb3_{robot_index}'

            # Create pose subscribers
            self.get_logger().info(f'Creating pos ahead subscriber /{tb_name}/pos')
            self.pos_sub = self.create_subscription(Pose2D,
                                    f'/{tb_name}/pos',
                                    partial(self.pos_ahead_callback, index=robot_index),
                                    qos_profile=qos_profile_sensor_data,
                                    callback_group=self.sensor_cb_group)

            # create cmd_vel publisher
            self.get_logger().info(f'Creating cmd_vel publisher: /cmd_vel')
            self.ros_pubs += [self.create_publisher(Twist, 'cmd_vel'.format(tb_name), 1)]
            
            if self.USECBF_DYNAMICOBS:
                # Assign all other robots as neighbour for collision avoidance
                neigh = []
                for j in range(ROBOT_COUNT):
                    if j != robot_index: neigh.append(j)
                self.inNeigh[robot_index] = np.array(neigh)

        # Initiate cbf
        self.cbf = [cbf_si() for _ in range(ROBOT_COUNT)]

        # Add handler if CTRL+C is pressed --> then save data to pickle
        signal.signal(signal.SIGINT, self.signal_handler)

        # Set timer for controller loop in each iteration
        self.controller_timer = self.create_timer(1./self.ROS_RATE, self.control_loop,
                                                callback_group=self.timer_cb_group)
        self.check_t = self.time()

    def time(self):
        """Returns the current time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def pos_ahead_callback(self, msg, index):
        self.poses_ahead[index] = msg

    def evaluate_scenario(self, scenario_mode):

        notice_txt = "Executing GoToInit "
        if scenario_mode == 1:
            notice_txt += "for CCTA Scenario: Formation Maintenace Avoiding Static Obstacle"
            # Scenario Formation Maintenace Avoiding Static Obstacle
            # ------------------------------------------------------
            cform_init = np.array([-1.8,-0.5, 0])
            form_w = 1 # width of rectangle formation
            form_h = 0.5 # height of rectangle formation
            # Set initial formation position --> Order: red, blue, green, orange
            self.goal_pos = np.array([   cform_init + np.array([  form_w/2, -form_h/2, 0]), 
                                    cform_init + np.array([  form_w/2,  form_h/2, 0]), 
                                    cform_init + np.array([ -form_w/2,  form_h/2, 0]), 
                                    cform_init + np.array([ -form_w/2, -form_h/2, 0]),  ])
            

            # Define Obstacle location
            self.obstacle = []
            self.obstacle += [{"pos": np.array([-0.4, 0.6, 0]), "r": 0.3}]
            self.obstacle += [{"pos": np.array([-0.5, -1., 0]), "r": 0.3}]
            self.gamma_staticObs = 1000

            self.USECBF_DYNAMICOBS = True
            self.USECBF_STATICOBS = True

        elif scenario_mode == 2:
            notice_txt += "for CCTA Scenario: Formation Maintenace Avoiding other Formation"
            # Scenario Formation Maintenace Avoiding other Formation
            # ------------------------------------------------------
            # Define Two formations
            cform1_init = np.array([-1., 1., 0])
            cform2_init = np.array([-1.02, -1, 0])
            #cform2_init = np.array([1.6, 0, 0])
            form_l = 0.5 # length between two robots in a formation
            # Set initial formation position --> Order: red, blue, green, orange
            self.goal_pos = np.array([   cform1_init + np.array([ 0,  form_l/2, 0]), 
                                    cform1_init + np.array([ 0, -form_l/2, 0]), 
                                    cform2_init + np.array([ 0,  form_l/2, 0]), 
                                    cform2_init + np.array([ 0, -form_l/2, 0]),  ])    

            self.USECBF_DYNAMICOBS = True
            self.USECBF_STATICOBS = False

        else: # other than 1 and 2
            notice_txt += "for testing"
            scale = 0.8
            centroid = np.array([-1.2, 0.1, 0])
            struct = np.array([[0.354, 0.354, 0], 
                               [-0.354, 0.354, 0], 
                               [-0.354, -0.354, 0], 
                               [ 0.354, -0.354, 0]]) * scale

            # self.goal_pos = np.array([[-x, y, 0], [x, y, 0], [x, -y, 0], [-x, -y, 0]])
            self.goal_pos = np.array([centroid + trans for trans in struct])

        self.get_logger().info(notice_txt)
        if self.USECBF_DYNAMICOBS: self.get_logger().info("using other robot's avoidance")
        if self.USECBF_STATICOBS: self.get_logger().info("using virtual obstacles's avoidance")

    @staticmethod
    def si_to_TBTwist(u, theta):
        # Inverse Look up ahead Mapping (u_z remain 0.)
        #   V = u_x cos(theta) + u_y sin(theta)
        #   omg = (- u_x sin(theta) + u_y cos(theta)) / l
        vel_lin = u[0]*np.cos(theta) + u[1]*np.sin(theta)
        vel_ang = (- u[0]*np.sin(theta) + u[1]*np.cos(theta))/TB_L_SI2UNI

        # TODO: do max (or saturation) if needed
        TBvel = Twist()
        TBvel.linear.x = vel_lin
        TBvel.linear.y = 0.0
        TBvel.linear.z = 0.0
        TBvel.angular.x = 0.0
        TBvel.angular.y = 0.0
        TBvel.angular.z = vel_ang

        return TBvel

    # Allow CTRL+C to stop the controller and dump the log into pickle
    def signal_handler(self, sig, frame):
        print('You pressed Ctrl+C. Turning off the controller.')

        # Stop all robots at the end
        for i in range(ROBOT_COUNT):
            self.ros_pubs[i].publish( self.si_to_TBTwist(np.array([0., 0.]), 0.) )

        exit()


    # MAIN LOOP CONTROLLER & VISUALIZATION
    def control_loop(self):

        now = self.time()
        diff = (now - self.check_t)
        if diff > (1.1/self.ROS_RATE): # Add 10% extra margin
            self.get_logger().info('Loop period: {:0.2f}'.format((now - self.check_t)*1000 ))     
        self.check_t = now


        if all (v is not None for v in self.poses_ahead):
            # Only run if all position values are initialized by localization systems
            
            for i in range(ROBOT_COUNT):
                # Collect States
                # ------------------------------------------------
                # TODO: checking if the pose is not None
                current_q = np.array([self.poses_ahead[i].x, self.poses_ahead[i].y, 0]) # Get position data only
                goal = self.goal_pos[i]

                if self.USECBF_DYNAMICOBS:
                    # Simulate get other robots position with laplacian
                    for j in self.inNeigh[i]:
                        j_pos = np.array([self.poses_ahead[j].x, self.poses_ahead[j].y, 0])
                        h = self.cbf[i].add_avoid_static_circle(current_q, j_pos, self.ds_dyn, 
                            gamma=self.gamma_dynamicObs, power=3)

                if self.USECBF_STATICOBS:            
                    for k in range(len(self.obstacle)):
                        h = self.cbf[i].add_avoid_static_circle(current_q, self.obstacle[k]["pos"], self.obstacle[k]["r"], 
                            gamma=self.gamma_staticObs, power=3)

                # Implementation of Control
                # ------------------------------------------------
                # Calculate nominal controller
                print(goal, current_q)
                u_nom = 0.8*(goal - current_q)

                # set speed limit
                speed_limit = 0.1
                norm = np.hypot(u_nom[0], u_nom[1])
                if norm > speed_limit: u_nom = speed_limit* u_nom / norm # max 

                self.mark_t = self.time()
                # Ensure safety
                u = self.cbf[i].compute_gtg(u_nom)

                diff = (self.time() - self.mark_t)*1000.
                if diff > 5:
                    self.get_logger().info('opt comp: {} - {:0.2f}'.format(i, diff))

                # Send command # TODO
                # ------------------------------------------------
                self.ros_pubs[i].publish( self.si_to_TBTwist(u, self.poses_ahead[i].theta) )



def main(args=None):
    rclpy.init(args=args)
    node = CCTAtoInit()
    executor = MultiThreadedExecutor()
    executor.add_node(node)    
    try:
        node.get_logger().info("Spinning " + ROS_NODE_NAME)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':   
    main()

