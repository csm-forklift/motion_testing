#!/usr/bin/env python
'''
This node reads all of the paths and sends them one at a time to the velocity
controller with the appropriate gear.
'''


import rospy
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
        self.path_gears = [-1, -1, 1, 1] # first two paths are reverse driving, last two are forward driving

        self.trigger_optimization = True
        self.optimization_successful = False
        self.opt_msg = Bool()
        self.opt_msg.data = True

        # ROS Publishers and Subscribers
        self.obstacle_avoidance_sub = rospy.Subscriber("/obstacle_avoidance_path", Path, self.obstacleAvoidanceCallback, queue_size=1)
        self.maneuver_path1_sub = rospy.Subscriber("/maneuver_path1", Path, self.maneuverPath1Callback, queue_size=1)
        self.maneuver_path2_sub = rospy.Subscriber("/maneuver_path2", Path, self.maneuverPath2Callback, queue_size=1)
        self.approach_path_sub = rospy.Subscriber("/approach_path", Path, self.approachPathCallback, queue_size=1)
        self.optimization_success_sub = rospy.Subscriber("/maneuver_path/optimization_success", Bool, self.optimizationCallback, queue_size=3)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)
        self.perform_optimization_pub = rospy.Publisher("/maneuver_path/perform_optimization", Bool, queue_size=3)
        self.gear_pub = rospy.Publisher("/forklift/gear", Int8, queue_size=3)

    def spin(self):
        while not rospy.is_shutdown():
            if (self.optimization_successful):
                self.trigger_optimization = False

            if (self.trigger_optimization):
                self.perform_optimization_pub.publish(self.opt_msg)

            if (self.paths[self.current_path] is not None and self.current_path < 4):
                while not rospy.get_param("/goal_bool"):
                    self.path_pub.publish(self.paths[self.current_path])
                    self.gear_pub.publish(self.path_gears[self.current_path])

                # Update the path and gear and pause to let the gear switch
                self.current_path += 1
                if (self.current_path < 4):
                    self.gear_pub.publish(self.path_gears[self.current_path])
                    time.sleep(1)
                else:
                    # DEBUG:
                    rospy.log_info("All paths complete.")


    def obstacleAvoidanceCallback(self, msg):
        paths[0] = msg

    def maneuverPath1Callback(self, msg):
        paths[1] = msg

    def maneuverPath2Callback(self, msg):
        paths[2] = msg

    def approachPathCallback(self, msg):
        paths[3] = msg

    def optimizationCallback(self, msg):
        self.optimization_successful = msg.data

if __name__ == "__main__":
    try:
        path_handler = PathHandler()
        path_handler.spin()
    except rospy.ROSInterruptException:
        pass
