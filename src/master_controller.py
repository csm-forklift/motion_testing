#!/usr/bin/env python
'''
This node dictates which controller can be running at a given time. This is the
master controller that handles the highest level of logic.

List of Control Modes
0: no controllers running
1: forward path tracking (velocity_controller_forward)
2: reverse path tracking (velocity_controller_reverse)
3: approach control + clamp control pick + cylinder deteciont (approach_control, clamp_control, cylinder_detection)
4: cylinder detection only (cylinder_detection)
5: clamp control drop (clamp_control)
'''


import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from grasping.srv import OptimizeManeuver, OptimizeManeuverRequest, OptimizeManeuverResponse
from motion_testing.srv import SetTarget, SetTargetRequest, SetTargetResponse
from motion_testing.msg import PathWithGear
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int8, Float32, Float64
import tf
import dynamic_reconfigure.client
import math
import time
import copy


class DebugTest:
    def __init__(self, starting_mode=1, allowed_modes=[1]):
        self.starting_mode = starting_mode
        self.allowed_modes = allowed_modes

class MasterController:
    def __init__(self):
        # Read in ROS parameters
        rospy.init_node("master_controller")
        self.master_operation_mode = rospy.get_param("~master_operation_mode", None)
        self.base_to_clamp = rospy.get_param("/forklift/body/base_to_clamp", 1.4658)
        self.approach_offset = rospy.get_param("~approach_offset", 2*self.base_to_clamp)
        self.scale_grasp = rospy.get_param("~scale_grasp", 0.5) # speed signal for clamp open/close
        self.scale_movement_up = rospy.get_param("~scale_movement_up", 0.5) # speed signal for clamp raise
        self.scale_movement_down = rospy.get_param("~scale_movement_down", 0.5) # speed signal for clamp lower
        self.maneuver_velocity = rospy.get_param("~maneuver_velocity", 0.1) # max velocity for the two maneuver paths
        self.previous_max_velocity = rospy.get_param("/velocity_controller/maximum_linear_velocity", 0.25) # max velocity for other paths

        # Control deadman parameters
        self.manual_deadman_button = rospy.get_param("~manual_deadman", 4)
        self.manual_deadman_on = False # this must be switched to 'True' to publish a non-zero command signal
        self.autonomous_deadman_button = rospy.get_param("~autonomous_deadman", 5)
        self.autonomous_deadman_on = False
        self.timeout = rospy.get_param("~timeout", 1) # number of seconds allowed since the last setpoint message before sending a 0 command
        self.timeout_start = time.time()

        # Used to test only a small portion of the code. This allows for
        # splitting up the various path tracking operations and only do the
        # desired ones at a time.
        self.debug_test = rospy.get_param("~debug_test", "none")
        # These are the currently available tests that have been implemented.
        # Add new ones to the list as the need arises.
        self.available_debug_tests = {"none": DebugTest(starting_mode=1, allowed_modes=[0,1,2,3,4,5,6,7]), \
        "grasp": DebugTest(starting_mode=5, allowed_modes=[5,6,7,0]), \
        "obstacle_avoidance": DebugTest(starting_mode=2, allowed_modes=[2]), \
        "maneuver": DebugTest(starting_mode=1, allowed_modes=[1,3,4,5,6,7,0]), \
        "clamp": DebugTest(starting_mode=7, allowed_modes=[7,0]), \
        "man_to_grasp": DebugTest(starting_mode=3, allowed_modes=[3,4,5,6,7,0])}

        # Path indices
        self.obstacle_path = 0
        self.maneuver_path1 = 1
        self.maneuver_path2 = 2
        self.approach_path = 3

        self.current_path = 0 # path index, 0 = obstacle, 1 = maneuver1, 2 = maneuver2, 3 = approach
        self.paths = [None]*(max(self.obstacle_path, self.maneuver_path1, self.maneuver_path2, self.approach_path) + 1)
        self.gears = [0]*len(self.paths)
        self.gears[self.obstacle_path] = -1 # obstacle avoidance is always in reverse
        self.gears[self.approach_path] = 1 # roll approach is always forward

        # Operation state flow
        if self.debug_test in self.available_debug_tests.keys():
            self.operation_mode = self.available_debug_tests[self.debug_test].starting_mode
        else:
            self.debug_test = "none"
            self.operation_mode = 1 # start in the first mode, wait for service call
        # Set the operation mode to the provided parameter if it has been set
        # otherwise do not assign it
        if (self.master_operation_mode is not None):
            self.operation_mode = self.master_operation_mode

        print("[%s]: Using debug mode: %s" % (rospy.get_name(), self.debug_test))
        print("[master_controller]: Starting mode set to: %d" % self.operation_mode)

        self.grasp_finished = False # inidcates when grasp operation is fully complete
        self.grasp_status = False # indicates whether the roll has been grasped by the clamp
        self.drop_finished = False # inidcates when drop operation is fully complete
        self.drop_status = False # indicates whether the roll has been dropped by the clamp
        self.forklift_current_pose = PoseStamped()
        self.target_current_pose = PoseStamped()

        # FIXME: making True for testing purposes only, return to False when putting on the system
        self.switch_status_down = False
        self.switch_status_open = False
        self.obstacle_path_received = False
        self.maneuver_path1_received = False
        self.maneuver_path2_received = False
        self.approach_path_received = False

        # Make sure all controllers start OFF
        self.control_mode = Int8()
        self.control_mode.data = 0 # determines which controllers can be operating (see top of file for notes)

        # Messages for publishing
        self.gear_msg = Int8()
        self.clamp_movement_msg = Float32()
        self.clamp_grasp_msg = Float32()

        # Publishing rate
        self.rate = rospy.Rate(30)

        # Transform listener for acquiring robot pose
        self.listener = tf.TransformListener()

        # Dynamic reconfigure client
        self.client = dynamic_reconfigure.client.Client("velocity_controller")

        # ROS Publishers and Subscribers
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)
        self.gear_pub = rospy.Publisher("/velocity_node/gear", Int8, queue_size=3)
        self.velocity_setpoint_pub = rospy.Publisher("/velocity_node/velocity_setpoint", Float64, queue_size=3)
        # Control Mode Options
        # 0) No controllers running
        # 1) Forward velocity controller
        # 2) Reverse velocity controller
        # 3) Approach controller + Clamp controller + cylinder detection
        # 4) Cylinder detection only
        self.control_mode_pub = rospy.Publisher("/control_mode", Int8, queue_size=3, latch=True)
        self.clamp_movement_pub = rospy.Publisher("/clamp_switch_node/clamp_movement", Float32, queue_size=1)
        self.clamp_grasp_pub = rospy.Publisher("/clamp_switch_node/clamp_grasp", Float32, queue_size=1)
        self.roll_pose_pub = rospy.Publisher("/roll/pose", PoseStamped, queue_size=3, latch=True)
        self.forklift_approach_pub = rospy.Publisher("/forklift/approach_pose", PoseStamped, queue_size=3)

        self.obstacle_avoidance_sub = rospy.Subscriber("/obstacle_avoidance_path", Path, self.obstacleAvoidanceCallback, queue_size=1)
        self.maneuver_path1_sub = rospy.Subscriber("/maneuver_path1_gear", PathWithGear, self.maneuverPath1Callback, queue_size=1)
        self.maneuver_path2_sub = rospy.Subscriber("/maneuver_path2_gear", PathWithGear, self.maneuverPath2Callback, queue_size=1)
        self.approach_path_sub = rospy.Subscriber("/approach_path", Path, self.approachPathCallback, queue_size=1)
        self.switch_status_down_sub = rospy.Subscriber("/switch_status_down", Bool, self.switchDownCallback, queue_size=1)
        self.switch_status_open_sub = rospy.Subscriber("/switch_status_open", Bool, self.switchOpenCallback, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size=3)
        self.grasp_status_sub = rospy.Subscriber("/clamp_control/grasp_status", Bool, self.graspSuccessCallback, queue_size=3)
        self.grasp_finished_sub = rospy.Subscriber("/clamp_control/grasp_finished", Bool, self.graspFinishedCallback, queue_size=3)
        self.drop_status_sub = rospy.Subscriber("/clamp_control/drop_status", Bool, self.dropSuccessCallback, queue_size=3)
        self.drop_finished_sub = rospy.Subscriber("/clamp_control/drop_finished", Bool, self.dropFinishedCallback, queue_size=3)
        self.roll_position_sub = rospy.Subscriber("/cylinder_detection/point", PointStamped, self.rollPositionCallback, queue_size=3)
        self.joystick_sub = rospy.Subscriber("/joy", Joy, self.joystickCallback, queue_size=3)

        #=====#
        # Wait for optimization maneuver service because it is require
        # before the set pick target service can run
        #=====#
        # Set up OptimizeManeuver service, send any bool value to have it
        # perform the optimization then it returns a bool indicating whether the
        # optimization was successful
        print("="*30)
        print("Waiting for maneuver service")
        print("="*30)

        self.trigger_optimization = True
        rospy.wait_for_service("/maneuver_path/optimize_maneuver")
        try:
            self.optimizeManeuver = rospy.ServiceProxy("/maneuver_path/optimize_maneuver", OptimizeManeuver)
        except rospy.ServiceException, e:
            print("Optimization service call failed: %s" % e)

        # Set Target Services
        print("="*30)
        print("Creating service for pick")
        print("="*30)

        self.pick_target_srv = rospy.Service("~set_pick_target", SetTarget, self.pickTarget)
        self.drop_target_srv = rospy.Service("~set_drop_target", SetTarget, self.dropTarget)


    def spin(self):
        while not rospy.is_shutdown():
            if (self.operation_mode not in self.available_debug_tests[self.debug_test].allowed_modes):
                # Restart operation
                self.operation_mode = self.available_debug_tests[self.debug_test].starting_mode

                # # Stop operation
                # self.operation_mode = 0
            self.rate.sleep()

    def obstacleAvoidanceCallback(self, msg):
        self.obstacle_path_received = True
        self.paths[self.obstacle_path] = copy.deepcopy(msg)

    def maneuverPath1Callback(self, msg):
        self.maneuver_path1_received = True
        self.paths[self.maneuver_path1] = copy.deepcopy(msg.path)
        self.gears[self.maneuver_path1] = msg.gear

    def maneuverPath2Callback(self, msg):
        self.maneuver_path2_received = True
        self.paths[self.maneuver_path2] = copy.deepcopy(msg.path)
        self.gears[self.maneuver_path2] = msg.gear

    def approachPathCallback(self, msg):
        self.approach_path_received = True
        self.paths[self.approach_path] = copy.deepcopy(msg)

    def optimizationCallback(self, msg):
        self.optimization_successful = msg.data

    def switchDownCallback(self, msg):
        self.switch_status_down = msg.data

    def switchOpenCallback(self, msg):
        self.switch_status_open = msg.data

    def odomCallback(self, msg):
        self.forklift_current_pose.header = copy.deepcopy(msg.header)
        self.forklift_current_pose.pose = copy.deepcopy(msg.pose.pose)

    def acquireRobotPose(self):
        try:
            self.listener.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(2.0))
            (trans, rot) = self.listener.lookupTransform('odom', 'base_link', rospy.Time(0))

            self.forklift_current_pose.header.stamp = rospy.Time(0)
            self.forklift_current_pose.header.frame_id = 'odom'
            self.forklift_current_pose.pose.position.x = trans[0]
            self.forklift_current_pose.pose.position.y = trans[1]
            self.forklift_current_pose.pose.position.z = trans[2]
            self.forklift_current_pose.pose.orientation.x = rot[0]
            self.forklift_current_pose.pose.orientation.y = rot[1]
            self.forklift_current_pose.pose.orientation.z = rot[2]
            self.forklift_current_pose.pose.orientation.w = rot[3]
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("[%s]: Error looking up transform from 'odom' to 'base_link'" % rospy.get_name())
            return False


    def graspSuccessCallback(self, msg):
        self.grasp_status = msg.data

    def graspFinishedCallback(self, msg):
        self.grasp_finished = msg.data

    def dropSuccessCallback(self, msg):
        self.drop_status = msg.data

    def dropFinishedCallback(self, msg):
        self.drop_finished = msg.data

    def rollPositionCallback(self, msg):
        self.target_current_pose.pose.position = copy.deepcopy(msg.point)

    def joystickCallback(self, msg):
        # Update timeout time
        self.timeout_start = time.time()

        # One of these buttons must be on for this node to send a driving command
        if (msg.buttons[self.manual_deadman_button]):
            self.manual_deadman_on = True
        else:
            self.manual_deadman_on = False

        if (msg.buttons[self.autonomous_deadman_button]):
            self.autonomous_deadman_on = True
        else:
            self.autonomous_deadman_on = False

    def distanceFromTarget(self):
        '''
        Calculate the distance from the forklift's current position to the roll position
        '''
        # # DEBUG:
        # print("Calculating distance using forklift: (%0.4f, %0.4f)" % (self.forklift_current_pose.pose.position.x, self.forklift_current_pose.pose.position.y))

        distance = math.sqrt((self.forklift_current_pose.pose.position.x - self.target_current_pose.pose.position.x)**2 + (self.forklift_current_pose.pose.position.y - self.target_current_pose.pose.position.y)**2)

        return distance

    def pickTarget(self, req):
        '''
        Operation modes
        1: Perform the path generation optimization using the provided roll target point
        2: Obstacle avoidance path tracking controller for getting from the current position to the desired maneuver starting point
        3: First segment of the maneuver
        4: Second segment of the maneuver
        5: Open and lower the clamp to check for the roll and update the target position if seen
        6: Follow the approach path b-spline trajectory
        7: Approach roll using the relative position from the lidar data and grasp
        '''
        # Publish the new roll target pose
        self.target_current_pose = req.roll_pose
        self.roll_pose_pub.publish(self.target_current_pose)

        # Begin grasping sequence
        self.trigger_optimization = True

        message = "Grasp sequence finished. Waiting for drop target."

        #======================================================================#
        # Handle Debug Conditions Here
        #======================================================================#
        if (self.debug_test == "obstacle_avoidance"):
            rospy.set_param("/control_panel_node/goal_x", self.target_current_pose.pose.position.x)
            rospy.set_param("/control_panel_node/goal_y", self.target_current_pose.pose.position.y)

            # Wait for the path to update
            self.obstacle_path_received = False
            while not self.obstacle_path_received:
                # DEBUG: Check for obstacle path
                print("Mesaa waitin'")

                dur = rospy.Duration(1.0)
                rospy.sleep(dur)

            rospy.loginfo("Obstacle path received. Beginning path tracking test.")

        if (self.debug_test == "man_to_grasp"):
            # run optimization to generate maneuver path
            resp = self.optimizeManeuver(True)

            # DEBUG:
            rospy.loginfo("Optimization result: %d\nMessage: %s", resp.optimization_successful, resp.message)


        #======================================================================#
        # Main Control Loop
        #======================================================================#
        while (self.grasp_finished is not True):
            # Run optimization to obtain paths from current position to the maneuver to the roll
            if (self.operation_mode == 1):
                print(30*"=")
                print("Optimizing maneuver")
                print(30*"=")
                # Set operation mode parameter in case of restart
                rospy.set_param("~master_operation_mode", self.operation_mode)
                resp = self.optimizeManeuver(True)

                # DEBUG:
                rospy.loginfo("Optimization result: %d\nMessage: %s", resp.optimization_successful, resp.message)

                if (resp.optimization_successful):
                    if (self.debug_test == 'maneuver'):
                        print("[%s]: maneuver test, skipping obstacle path" % rospy.get_name())
                        self.operation_mode = 3
                    else:
                        rospy.wait_for_message("/obstacle_avoidance_path", Path)
                        # Publish path and gear and delay
                        self.path_pub.publish(self.paths[self.obstacle_path])
                        self.publishGear(self.gears[self.obstacle_path])
                        time.sleep(1)

                        print("[%s]: other test: %s" % (rospy.get_name(), self.debug_test))
                        self.operation_mode = 2

            # Avoid obstacles on the way to the maneuver path
            if (self.operation_mode == 2):
                print(30*"=")
                print("Obstacle avoidance path")
                print(30*"=")
                # Set operation mode parameter in case of restart
                rospy.set_param("~master_operation_mode", self.operation_mode)

                if (self.paths[self.obstacle_path] is not None):

                    self.control_mode.data = 2 # reverse controller
                    self.control_mode_pub.publish(self.control_mode)

                    while not rospy.get_param("/goal_bool", False):
                        self.path_pub.publish(self.paths[self.obstacle_path])
                        self.publishGear(self.gears[self.obstacle_path])
                        self.rate.sleep()

                    # Publish next path and delay
                    self.path_pub.publish(self.paths[self.maneuver_path1])
                    self.publishGear(self.gears[self.maneuver_path1])
                    time.sleep(1)

                    self.operation_mode = 3
                else:
                    message = "Error: obstacle path was not generated"
                    self.grasp_finished = False
                    break

            # Drive first segment of maneuver
            if (self.operation_mode == 3):
                print(30*"=")
                print("Maneuver part 1")
                print(30*"=")
                # Set operation mode parameter in case of restart
                rospy.set_param("~master_operation_mode", self.operation_mode)

                # Get the current maximum velocity for the velocity controller, look ahead segments, and cross track error gain
                self.previous_max_velocity = rospy.get_param("/velocity_controller/maximum_linear_velocity", self.maneuver_velocity)
                self.num_of_segments_ahead = rospy.get_param("/velocity_controller/num_of_segments_ahead", 10)
                self.previous_cte_gain = rospy.get_param("/velocity_controller/cte_gain", 0.2)

                # Set max velocity to the maneuver velocity and reduce the look ahead segments to 1 since these paths are smooth
                params = {"maximum_linear_velocity" : self.maneuver_velocity, "num_of_segments_ahead" : 1}
                config = self.client.update_configuration(params)

                if (self.paths[self.maneuver_path1] is not None):

                    if (self.gears[self.maneuver_path1] < 0):
                        self.control_mode.data = 2 # reverse controller
                    else:
                        self.control_mode.data = 1 # forward controller
                    self.control_mode_pub.publish(self.control_mode)

                    while not rospy.get_param("/goal_bool", False):
                        self.path_pub.publish(self.paths[self.maneuver_path1])
                        self.publishGear(self.gears[self.maneuver_path1])
                        self.rate.sleep()

                    # Publish next path and delay
                    self.path_pub.publish(self.paths[self.maneuver_path2])
                    self.publishGear(self.gears[self.maneuver_path2])

                    # Set velocity to 0 before switching modes
                    velocity_setpoint_msg = Float64()
                    velocity_setpoint_msg.data = 0.0
                    self.velocity_setpoint_pub.publish(velocity_setpoint_msg)
                    time.sleep(1)

                    self.operation_mode = 4
                else:
                    message = "Error: maneuver path 1 was not generated"
                    self.grasp_finished = False
                    break

            # Drive second segment of maneuver
            if (self.operation_mode == 4):
                print(30*"=")
                print("Maneuver part 2")
                print(30*"=")
                # Set operation mode parameter in case of restart
                rospy.set_param("~master_operation_mode", self.operation_mode)

                if (self.paths[self.maneuver_path2] is not None):

                    if (self.gears[self.maneuver_path2] < 0):
                        self.control_mode.data = 2 # reverse controller
                    else:
                        self.control_mode.data = 1 # forward controller
                    self.control_mode_pub.publish(self.control_mode)

                    while not rospy.get_param("/goal_bool", False):
                        self.path_pub.publish(self.paths[self.maneuver_path2])
                        self.publishGear(self.gears[self.maneuver_path2])
                        self.rate.sleep()

                    self.operation_mode = 5

                    # Set velocity to 0 before switching modes
                    velocity_setpoint_msg = Float64()
                    velocity_setpoint_msg.data = 0.0
                    self.velocity_setpoint_pub.publish(velocity_setpoint_msg)

                else:
                    message = "Error: maneuver path 2 was not generated"
                    self.grasp_finished = False
                    break

            # Move clamp down and open to try to see the roll with the lidar
            # if the roll is seen, update the target position
            if (self.operation_mode == 5):
                print(30*"=")
                print("Move clamp to see roll")
                print(30*"=")
                # Set operation mode parameter in case of restart
                rospy.set_param("~master_operation_mode", self.operation_mode)

                # Turn controllers off
                # FIXME: trying turning off the cylinder detection to avoid errors before actually approaching the roll
                self.control_mode.data = 0
                self.control_mode_pub.publish(self.control_mode)

                # Open the clamp
                while (self.switch_status_open == False):
                    self.publishClampGrasp(self.scale_grasp) # positive is open
                    self.rate.sleep()
                # Send the stop command
                self.publishClampGrasp(0)

                # Lower the clamp
                while (self.switch_status_down == False):
                    self.publishClampMovement(self.scale_movement_down) # positive is down
                    self.rate.sleep()
                # Send the stop command
                self.publishClampMovement(0)

                # Check if the roll pose has changed from the target specified by the service request. If so, publish the new roll and forklift poses to generate a new approach path
                if ((self.target_current_pose.pose.position.x != req.roll_pose.pose.position.x) or (self.target_current_pose.pose.position.y != req.roll_pose.pose.position.y)):
                    self.roll_pose_pub.publish(self.target_current_pose)

                # Update approach path using forklift's current position
                self.acquireRobotPose() # use this function only if no odometry message is being published
                self.forklift_approach_pub.publish(self.forklift_current_pose)
                time.sleep(1)

                # Publish next path and delay
                self.path_pub.publish(self.paths[self.approach_path])
                self.publishGear(self.gears[self.approach_path])

                self.operation_mode = 6

            # Follow the approach b-spline path to the roll until the vehicle is within the tolerance to switch from path tracking to relative position control
            if (self.operation_mode == 6):
                print(30*"=")
                print("Approach path")
                print(30*"=")
                # Set operation mode in case of restart
                rospy.set_param("~master_operation_mode", self.operation_mode)

                # Restore the previous maximum velocity for the velocity controller
                params = {"maximum_linear_velocity" : 0.15, "num_of_segments_ahead" : 5, "cte_gain" : 0.5}
                config = self.client.update_configuration(params)

                self.control_mode.data = 1 # forward controller
                self.control_mode_pub.publish(self.control_mode)

                if (self.paths[self.approach_path] is not None):
                    self.acquireRobotPose()
                    print("[%s]: Distance from target: %0.4f, Radius: %0.4f" % (rospy.get_name(), self.distanceFromTarget(), self.approach_offset))
                    while (self.distanceFromTarget() > self.approach_offset):
                        # Get forklift's current position
                        self.acquireRobotPose()

                        # DEBUG:
                        print("[%s]: Distance from target: %0.4f, Radius: %0.4f" % (rospy.get_name(), self.distanceFromTarget(), self.approach_offset))
                        self.path_pub.publish(self.paths[self.approach_path])
                        self.publishGear(self.gears[self.approach_path])
                        self.rate.sleep()

                    self.operation_mode = 7

                    # Set velocity to 0 before switching modes
                    velocity_setpoint_msg = Float64()
                    velocity_setpoint_msg.data = 0.0
                    self.velocity_setpoint_pub.publish(velocity_setpoint_msg)

                    # Restore the look ahead segments
                    params = {"num_of_segments_ahead" : self.num_of_segments_ahead}
                    config = self.client.update_configuration(params)
                else:
                    message = "Error: approach path was not generated"
                    self.grasp_finished = False
                    break

            # Approach roll using relative position
            if (self.operation_mode == 7):
                print(30*"=")
                print("Final grasp portion")
                print(30*"=")
                # Set operation mode in case of restart
                rospy.set_param("~master_operation_mode", self.operation_mode)

                self.control_mode.data = 3 # approach + clamp control
                self.control_mode_pub.publish(self.control_mode)

                # Make sure that the system stops while switching between the regular velocity controller and the clamp approach controller
                # Set velocity to 0 before switching modes
                velocity_setpoint_msg = Float64()
                velocity_setpoint_msg.data = 0.0
                self.velocity_setpoint_pub.publish(velocity_setpoint_msg)

                while (self.grasp_finished == False):
                    self.rate.sleep()

                self.operation_mode = 0
                message = "Pick grasp completed successfully"

                # Turn off controllers
                self.control_mode.data = 0 # no controllers
                self.control_mode_pub.publish(self.control_mode)

        resp = SetTargetResponse()
        resp.success = self.grasp_finished
        resp.message = message
        return resp

    def dropTarget(self, req):
        '''
        Operation Modes
        1: Travel to target position
        2: Drop roll
        '''
        # Initialize operation mode
        self.operation_mode = 1

        # Get target and set obstacle avoidance path controller
        self.target_current_pose = req.roll_pose
        rospy.set_param("/control_panel_node/goal_x", float(self.target_current_pose.pose.position.x))
        rospy.set_param("/control_panel_node/goal_y", float(self.target_current_pose.pose.position.y))

        # Wait for the path to update
        self.obstacle_path_received = False
        while not self.obstacle_path_received:
            # DEBUG: Check for obstacle path
            print("Mesaa waitin'")

            dur = rospy.Duration(1.0)
            rospy.sleep(dur)

        # Avoid obstacles on the way to the maneuver path
        if (self.operation_mode == 1):
            print(30*"=")
            print("Obstacle avoidance path")
            print(30*"=")
            # Set operation mode parameter in case of restart
            rospy.set_param("~master_operation_mode", self.operation_mode)

            if (self.paths[self.obstacle_path] is not None):

                self.control_mode.data = 2 # reverse controller
                self.control_mode_pub.publish(self.control_mode)

                while not rospy.get_param("/goal_bool", False):
                    self.path_pub.publish(self.paths[self.obstacle_path])
                    self.publishGear(self.gears[self.obstacle_path])
                    self.rate.sleep()

                # Delay
                time.sleep(1)

                self.operation_mode = 2
            else:
                message = "Error: obstacle path was not generated"
                self.drop_finished = False
                break

        if (self.operation_mode == 2):
            print(30*"=")
            print("Dropping roll")
            print(30*"=")
            # Set operation mode parameter in case of restart
            rospy.set_param("~master_operation_mode", self.operation_mode)

            self.control_mode.data = 5 # clamp_control drop
            self.control_mode_pub.publish(self.control_mode)

            # Make sure that the system stops while switching between the regular velocity controller and the clamp approach controller
            # Set velocity to 0 before switching modes
            velocity_setpoint_msg = Float64()
            velocity_setpoint_msg.data = 0.0
            self.velocity_setpoint_pub.publish(velocity_setpoint_msg)

            while (self.drop_finished == False):
                self.rate.sleep()

            self.operation_mode = 0
            message = "Drop completed successfully"

            # Turn off controllers
            self.control_mode.data = 0 # no controllers
            self.control_mode_pub.publish(self.control_mode)

        resp = SetTargetResponse()
        resp.success = self.drop_finished
        resp.message = message
        return resp

    def publishGear(self, gear):
        # Publish the gear value considering deadman buttons
        if (self.manual_deadman_on and ((time.time() - self.timeout_start) < self.timeout)):
            # Send no command
            pass
        elif (self.autonomous_deadman_on and ((time.time() - self.timeout_start) < self.timeout)):
            # Send desired gear
            self.gear_msg.data = gear
            self.gear_pub.publish(self.gear_msg)
        else:
            # Deadman not pressed or joystick timed out
            self.gear_msg.data = 0
            self.gear_pub.publish(self.gear_msg)

    def publishClampMovement(self, movement):
        # Publish the clamp raise/lower command considering deadman buttons
        if (self.manual_deadman_on and ((time.time() - self.timeout_start) < self.timeout)):
            # Send no command
            pass
        elif (self.autonomous_deadman_on and ((time.time() - self.timeout_start) < self.timeout)):
            # Send desired gear
            self.clamp_movement_msg.data = movement
            self.clamp_movement_pub.publish(self.clamp_movement_msg)
        else:
            # Deadman not pressed or joystick timed out
            self.clamp_movement_msg.data = 0
            self.clamp_movement_pub.publish(self.clamp_movement_msg)

    def publishClampGrasp(self, grasp):
        # Publish the clamp open/close command considering deadman buttons
        if (self.manual_deadman_on and ((time.time() - self.timeout_start) < self.timeout)):
            # Send no command
            pass
        elif (self.autonomous_deadman_on and ((time.time() - self.timeout_start) < self.timeout)):
            # Send desired gear
            self.clamp_grasp_msg.data = grasp
            self.clamp_grasp_pub.publish(self.clamp_grasp_msg)
        else:
            # Deadman not pressed or joystick timed out
            self.clamp_grasp_msg.data = 0
            self.clamp_grasp_pub.publish(self.clamp_grasp_msg)

if __name__ == "__main__":
    try:
        master_controller = MasterController()
        master_controller.spin()
    except rospy.ROSInterruptException:
        pass
