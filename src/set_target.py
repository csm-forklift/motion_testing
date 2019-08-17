#!/usr/bin/env python
'''
This node calls the 'set_pick_target' service from 'master_controller'
'''


import rospy
from motion_testing.srv import SetTarget, SetTargetRequest, SetTargetResponse
import math

def main():
    # Create node
    rospy.init_node("set_target")

    # Determine the setpoint to use
    target_location = rospy.get_param("~location", "demo")

    # Run the service and return the response
    rospy.wait_for_service("/master_controller/set_pick_target", 2)
    try:
        set_target = rospy.ServiceProxy("/master_controller/set_pick_target", SetTarget)
    except rospy.ServiceException, e:
        print("[%s]: waiting for service failed: %s" % (rospy.get_name(), e))

    request = SetTargetRequest()
    request.roll_pose.header.frame_id = 'odom'

    if (target_location == "demo"):
        print("[%s]: using 'demo' target" % rospy.get_name())
        request.roll_pose.pose.position.x = 9.0
        request.roll_pose.pose.position.y = 2.0
        request.roll_pose.pose.position.z = 0.0
        request.roll_pose.pose.orientation.x = 0.0
        request.roll_pose.pose.orientation.y = 0.0
        request.roll_pose.pose.orientation.z = -math.sqrt(0.5)
        request.roll_pose.pose.orientation.w = math.sqrt(0.5)

    elif (target_location == "maneuver"):
        print("[%s]: using 'maneuver' target" % rospy.get_name())
        request.roll_pose.pose.position.x = -9.0
        request.roll_pose.pose.position.y = 7.0
        request.roll_pose.pose.position.z = 0.0
        request.roll_pose.pose.orientation.x = 0.0
        request.roll_pose.pose.orientation.y = 0.0
        request.roll_pose.pose.orientation.z = -0.3826834
        request.roll_pose.pose.orientation.w = 0.9238795

    elif (target_location == "grasp"):
        print("[%s]: using 'grasp' target" % rospy.get_name())
        request.roll_pose.pose.position.x = -6.0
        request.roll_pose.pose.position.y = 0.0
        request.roll_pose.pose.position.z = 0.0
        request.roll_pose.pose.orientation.x = 0.0
        request.roll_pose.pose.orientation.y = 0.0
        request.roll_pose.pose.orientation.z = 0.0
        request.roll_pose.pose.orientation.w = 1.0

    resp = set_target(request)
    print(resp)


try:
    main()
except rospy.ROSInterruptException:
    pass
