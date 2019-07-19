#!/usr/bin/env python
'''
Sends the service call to set the roll pose target for the 'pick' command.
'''

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from motion_testing.srv import SetTarget, SetTargetRequest, SetTargetResponse
from tf.transformations import quaternion_from_euler
import time

rospy.wait_for_service("/master_controller/set_pick_target")
try:
    set_target = rospy.ServiceProxy("/master_controller/set_pick_target", SetTarget)
except rospy.ServiceException, e:
    print("Service call failed: %s", e)

# Target point
target = PoseStamped()
target.header.frame_id = "odom"
# Full path point
# target.pose.position.x = 13
# target.pose.position.y = 5
# Short path point
target.pose.position.x = 3
target.pose.position.y = 5
quat_target = quaternion_from_euler(0,0,-0.78539816339)
target.pose.orientation = Quaternion(quat_target[0], quat_target[1], quat_target[2], quat_target[3])

time.sleep(1)

print("Calling 'SetTarget' service")
resp = set_target(target)
print("Service finished")
print("success: %d" % resp.success)
print("message: %s" % resp.message)
