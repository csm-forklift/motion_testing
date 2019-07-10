#!/usr/bin/env python
'''
This node reads all of the paths and sends them one at a time to the velocity
controller with the appropriate gear.
'''


import rospy
from grasping.srv import OptimizeManeuver, OptimizeManeuverRequest, OptimizeManeuverResponse
from motion_testing.msg import PathWithGear
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Int8
import time


class PathHandler:
    def __init__(self):
        # Read in ROS parameters
        rospy.init_node("path_handler")
        self.obstacle_path = None
        self.maneuver_path1 = None
        self.maneuver_path2 = None
        self.approach_path = None

        self.rate = rospy.Rate(30)

        self.current_path = 0 # path index, 0 = obstacle, 1 = maneuver1, 2 = maneuver2, 3 = approach
        self.paths = [self.obstacle_path, self.maneuver_path1, self.maneuver_path2, self.approach_path]
        self.gears = [-1, -1, 1, 1] # first two paths are reverse driving, last two are forward driving

        self.trigger_optimization = True

        # ROS Publishers and Subscribers
        self.obstacle_avoidance_sub = rospy.Subscriber("/obstacle_avoidance_path", Path, self.obstacleAvoidanceCallback, queue_size=1)
        self.maneuver_path1_sub = rospy.Subscriber("/maneuver_path1", PathWithGear, self.maneuverPath1Callback, queue_size=1)
        self.maneuver_path2_sub = rospy.Subscriber("/maneuver_path2", PathWithGear, self.maneuverPath2Callback, queue_size=1)
        self.approach_path_sub = rospy.Subscriber("/approach_path", Path, self.approachPathCallback, queue_size=1)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)
        self.gear_pub = rospy.Publisher("/velocity_node/gear", Int8, queue_size=3)

        # Set up OptimizeManeuver service, send any bool value to have it perform the optimization then it returns a bool indicating whether the optimization was successful
        rospy.wait_for_service("/maneuver_path/optimize_maneuver")
        try:
            self.optimizeManeuver = rospy.ServiceProxy("/maneuver_path/optimize_maneuver", OptimizeManeuver)
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

    def spin(self):
        while not rospy.is_shutdown():
            if (self.current_path < 4):
                if (self.trigger_optimization):
                    resp = self.optimizeManeuver(True)
                    if (resp.optimization_successful):
                        self.trigger_optimization = False

                if (self.paths[self.current_path] is not None):
                    # DEBUG:
                    print("Publishing current path (%d) using gear (%d):" % (self.current_path, self.gears[self.current_path]))
                    print(self.paths[self.current_path])

                    while not rospy.get_param("/goal_bool", False):
                        self.path_pub.publish(self.paths[self.current_path])
                        gear = Int8()
                        gear.data = self.gears[self.current_path]
                        self.gear_pub.publish(gear)
                        self.rate.sleep()

                    # Update the path and gear and pause to let the gear switch
                    self.current_path += 1

                    if (self.current_path < 4):
                        # Update the gear and wait 1 second for it to shift
                        self.gear_pub.publish(self.gears[self.current_path])
                        time.sleep(1)
                        # Publish the new path to reset the goal
                        self.path_pub.publish(self.paths[self.current_path])
                    else:
                        # DEBUG:
                        rospy.loginfo("All paths complete.")

    def obstacleAvoidanceCallback(self, msg):
        self.paths[0] = msg

    def maneuverPath1Callback(self, msg):
        self.paths[1] = msg.path
        self.gears[1] = msg.gear

    def maneuverPath2Callback(self, msg):
        self.paths[2] = msg.path
        self.gears[2] = msg.gear

    def approachPathCallback(self, msg):
        self.paths[3] = msg

    def optimizationCallback(self, msg):
        self.optimization_successful = msg.data

if __name__ == "__main__":
    try:
        path_handler = PathHandler()
        path_handler.spin()
    except rospy.ROSInterruptException:
        pass
