#!/usr/bin/env python
'''
This node dictates which controller can be running at a given time. This is the
master controller that handles the highest level of logic.

List of Control Modes
0: no controllers running
1: forward path tracking (velocity_controller_forward)
2: reverse path tracking (velocity_controller_reverse)
3: approach control + clamp control (approach_control, clamp_control)
'''


import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from grasping.srv import OptimizeManeuver, OptimizeManeuverRequest, OptimizeManeuverResponse
from motion_testing.srv import SetTarget, SetTargetRequest, SetTargetResponse
from motion_testing.msg import PathWithGear
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, Int8, Float32
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
        self.base_to_clamp = rospy.get_param("/forklift/body/base_to_clamp", 1.4658)
        self.roll_approach_radius = rospy.get_param("~roll_approach_radius", 3*self.base_to_clamp)
        self.scale_grasp = rospy.get_param("~scale_grasp", 0.5) # speed signal for clamp open/close
        self.scale_movement = rospy.get_param("~scale_movement", 0.5) # speed signal for clamp raise/lower
        # Used to test only a small portion of the code. This allows for
        # splitting up the various path tracking operations and only do the
        # desired ones at a time.
        self.debug_test = rospy.get_param("~debug_test", "none")
        # These are the currently available tests that have been implemented.
        # Add new ones to the list as the need arises.
        self.available_debug_tests = {"none": DebugTest(starting_mode=1, allowed_modes=[1,2,3,4,5,6,7]), "grasp": DebugTest(starting_mode=5, allowed_modes=[5,6,7])}

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
        print("Starting mode set to: %d" % self.operation_mode)

        self.grasp_finished = False # inidcates when grasp operation is fully complete
        self.grasp_success = False # indicates whether the roll has been grasped by the clamp
        self.forklift_current_pose = PoseStamped()
        self.target_current_pose = PoseStamped()

        # FIXME: making True for testing purposes only, return to False when putting on the system
        self.switch_status_down = True
        self.switch_status_open = True

        # Make sure all controllers start OFF
        self.control_mode = Int8()
        self.control_mode.data = 0 # determines which controllers can be operating (see top of file for notes)

        # Publishing rate
        self.rate = rospy.Rate(30)

        # ROS Publishers and Subscribers
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)
        self.gear_pub = rospy.Publisher("/velocity_node/gear", Int8, queue_size=3)
        self.control_mode_pub = rospy.Publisher("/control_mode", Int8, queue_size=3, latch=True)
        self.clamp_movement_pub = rospy.Publisher("/clamp_switch_node/clamp_movement", Float32, queue_size=3)
        self.clamp_grasp_pub = rospy.Publisher("/clamp_switch_node/clamp_grasp", Float32, queue_size=3)
        self.roll_pose_pub = rospy.Publisher("/roll/pose", PoseStamped, queue_size=3, latch=True)
        self.forklift_approach_pub = rospy.Publisher("/forklift/approach_pose", PoseStamped, queue_size=3)

        self.obstacle_avoidance_sub = rospy.Subscriber("/obstacle_avoidance_path", Path, self.obstacleAvoidanceCallback, queue_size=1)
        self.maneuver_path1_sub = rospy.Subscriber("/maneuver_path1", PathWithGear, self.maneuverPath1Callback, queue_size=1)
        self.maneuver_path2_sub = rospy.Subscriber("/maneuver_path2", PathWithGear, self.maneuverPath2Callback, queue_size=1)
        self.approach_path_sub = rospy.Subscriber("/approach_path", Path, self.approachPathCallback, queue_size=1)
        self.switch_status_down_sub = rospy.Subscriber("/switch_status_down", Bool, self.switchDownCallback, queue_size=1)
        self.switch_status_open_sub = rospy.Subscriber("/switch_status_open", Bool, self.switchOpenCallback, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size=3)
        self.grasp_success_sub = rospy.Subscriber("/clamp_control/grasp_success", Bool, self.graspSuccessCallback, queue_size=3)
        self.grasp_finished_sub = rospy.Subscriber("/clamp_control/grasp_finished", Bool, self.graspFinishedCallback, queue_size=3)
        self.roll_position_sub = rospy.Subscriber("/cylinder_detection/point", PointStamped, self.rollPositionCallback, queue_size=3)

        #=====#
        # Wait for optimization maneuver service because it is require
        # before the set pick target service can run
        #=====#
        # Set up OptimizeManeuver service, send any bool value to have it
        # perform the optimization then it returns a bool indicating whether the
        # optimization was successful
        print("="*30)
        print("Waiting for maneuver service")

        self.trigger_optimization = True
        rospy.wait_for_service("/maneuver_path/optimize_maneuver")
        try:
            self.optimizeManeuver = rospy.ServiceProxy("/maneuver_path/optimize_maneuver", OptimizeManeuver)
        except rospy.ServiceException, e:
            print("Optimization service call failed: %s" % e)

        # Set Target Services
        print("="*30)
        print("Creating service for pick")

        self.pick_target_srv = rospy.Service("~set_pick_target", SetTarget, self.pickTarget)
        self.drop_target_srv = rospy.Service("~set_drop_target", SetTarget, self.dropTarget)


    def spin(self):
        while not rospy.is_shutdown():
            if (self.operation_mode not in self.available_debug_tests[self.debug_test].allowed_modes):
                # # Restart operation
                # self.operation_mode = self.available_debug_tests[self.debug_test].starting_mode

                # Stop operation
                self.operation_mode = 0
            self.rate.sleep()

    def obstacleAvoidanceCallback(self, msg):
        self.paths[self.obstacle_path] = copy.deepcopy(msg)

    def maneuverPath1Callback(self, msg):
        self.paths[self.maneuver_path1] = copy.deepcopy(msg.path)
        self.gears[self.maneuver_path1] = msg.gear

    def maneuverPath2Callback(self, msg):
        self.paths[self.maneuver_path2] = copy.deepcopy(msg.path)
        self.gears[self.maneuver_path2] = msg.gear

    def approachPathCallback(self, msg):
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

    def graspSuccessCallback(self, msg):
        self.grasp_success = msg.data

    def graspFinishedCallback(self, msg):
        self.grasp_finished = msg.data

    def rollPositionCallback(self, msg):
        self.target_current_pose.pose.position = copy.deepcopy(msg.point)

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

        while (self.grasp_finished is not True):
            # Run optimization to obtain paths from current position to the maneuver to the roll
            if (self.operation_mode == 1):
                print(30*"=")
                print("Optimizing maneuver")
                print(30*"=")
                resp = self.optimizeManeuver(True)

                # DEBUG:
                rospy.loginfo("Optimization result: %d", resp.optimization_successful)

                if (resp.optimization_successful):
                    rospy.wait_for_message("/obstacle_avoidance_path", Path)
                    # Publish path and gear and delay
                    self.path_pub.publish(self.paths[self.obstacle_path])
                    gear = Int8()
                    gear.data = self.gears[self.obstacle_path]
                    self.gear_pub.publish(gear)
                    time.sleep(1)

                    self.operation_mode = 2


            # Avoid obstacles on the way to the maneuver path
            if (self.operation_mode == 2):
                print(30*"=")
                print("Obstacle avoidance path")
                print(30*"=")
                if (self.paths[self.obstacle_path] is not None):

                    self.control_mode.data = 2 # reverse controller
                    self.control_mode_pub.publish(self.control_mode)

                    while not rospy.get_param("/goal_bool", False):
                        self.path_pub.publish(self.paths[self.obstacle_path])
                        gear = Int8()
                        gear.data = self.gears[self.obstacle_path]
                        self.gear_pub.publish(gear)
                        self.rate.sleep()

                    # Publish next path and delay
                    self.path_pub.publish(self.paths[self.maneuver_path1])
                    gear = Int8()
                    gear.data = self.gears[self.maneuver_path1]
                    self.gear_pub.publish(gear)
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
                if (self.paths[self.maneuver_path1] is not None):

                    if (self.gears[self.maneuver_path1] < 0):
                        self.control_mode.data = 2 # reverse controller
                    else:
                        self.control_mode.data = 1 # forward controller
                    self.control_mode_pub.publish(self.control_mode)

                    while not rospy.get_param("/goal_bool", False):
                        self.path_pub.publish(self.paths[self.maneuver_path1])
                        gear = Int8()
                        gear.data = self.gears[self.maneuver_path1]
                        self.gear_pub.publish(gear)
                        self.rate.sleep()

                    # Publish next path and delay
                    self.path_pub.publish(self.paths[self.maneuver_path2])
                    gear = Int8()
                    gear.data = self.gears[self.maneuver_path2]
                    self.gear_pub.publish(gear)
                    self.rate.sleep()

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
                if (self.paths[self.maneuver_path2] is not None):

                    if (self.gears[self.maneuver_path2] < 0):
                        self.control_mode.data = 2 # reverse controller
                    else:
                        self.control_mode.data = 1 # forward controller
                    self.control_mode_pub.publish(self.control_mode)

                    while not rospy.get_param("/goal_bool", False):
                        self.path_pub.publish(self.paths[self.maneuver_path2])
                        gear = Int8()
                        gear.data = self.gears[self.maneuver_path2]
                        self.gear_pub.publish(gear)
                        self.rate.sleep()

                    self.operation_mode = 5
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
                # Turn controllers off
                self.control_mode.data = 4
                self.control_mode_pub.publish(self.control_mode)

                # Open the clamp
                while (self.switch_status_open == False):
                    clamp_grasp = Float32()
                    clamp_grasp.data = self.scale_grasp # positive is open
                    self.clamp_grasp_pub.publish(clamp_grasp)

                # Lower the clamp
                while (self.switch_status_down == False):
                    clamp_movement = Float32()
                    clamp_movement = self.scale_movement # positive is down
                    self.clamp_movement_pub.publish(clamp_movement)
                    self.rate.sleep()

                # Check if the roll pose has changed from the target specified by the service request. If so, publish the new roll and forklift poses to generate a new approach path
                if ((self.target_current_pose.pose.position.x != req.roll_pose.pose.position.x) or (self.target_current_pose.pose.position.y != req.roll_pose.pose.position.y)):
                    self.roll_pose_pub.publish(self.target_current_pose)

                # Update approach path using forklift's current position
                self.forklift_approach_pub.publish(self.forklift_current_pose)
                time.sleep(1)

                # Publish next path and delay
                self.path_pub.publish(self.paths[self.approach_path])
                gear = Int8()
                gear.data = self.gears[self.approach_path]
                self.gear_pub.publish(gear)

                self.operation_mode = 6

            # Follow the approach b-spline path to the roll until the vehicle is within the tolerance to switch from path tracking to relative position control
            if (self.operation_mode == 6):
                print(30*"=")
                print("Approach path")
                print(30*"=")

                self.control_mode.data = 1 # forward controller
                self.control_mode_pub.publish(self.control_mode)

                if (self.paths[self.approach_path] is not None):
                    while (self.distanceFromTarget() > self.roll_approach_radius):
                        # DEBUG:
                        print("Distance from target: %0.4f, Radius: %0.4f" % (self.distanceFromTarget(), self.roll_approach_radius))
                        self.path_pub.publish(self.paths[self.approach_path])
                        gear = Int8()
                        gear.data = self.gears[self.approach_path]
                        self.gear_pub.publish(gear)
                        self.rate.sleep()

                    self.operation_mode = 7
                else:
                    message = "Error: approach path was not generated"
                    self.grasp_finished = False
                    break

            # Approach roll using relative position
            if (self.operation_mode == 7):
                print(30*"=")
                print("Final grasp portion")
                print(30*"=")

                self.control_mode.data = 3 # approach + clamp control
                self.control_mode_pub.publish(self.control_mode)

                while (self.grasp_finished == False):
                    self.rate.sleep()
                self.operation_mode = 0
                message = "Pick grasp completed successfully"

        resp = SetTargetResponse()
        resp.success = self.grasp_finished
        resp.message = message
        return resp

    def dropTarget(self, req):
        '''
        Description of modes and stateflow
        '''
        resp = SetTargetResponse()
        resp.success = False
        resp.message = "This feature has not been implemented yet."
        return resp

if __name__ == "__main__":
    try:
        master_controller = MasterController()
        master_controller.spin()
    except rospy.ROSInterruptException:
        pass
