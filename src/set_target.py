#!/usr/bin/env python
'''
This node calls the 'set_pick_target' service from 'master_controller'
'''


import rospy
from motion_testing.srv import SetTarget, SetTargetRequest, SetTargetResponse
import math
import tf

def main():
    # Create node
    rospy.init_node("set_target")

    # Determine the setpoint to use
    target_location = rospy.get_param("~location", "demo1")

    # Run the service and return the response
    rospy.wait_for_service("/master_controller/set_pick_target", 2)
    try:
        set_target = rospy.ServiceProxy("/master_controller/set_pick_target", SetTarget)
    except rospy.ServiceException, e:
        print("[%s]: waiting for service failed: %s" % (rospy.get_name(), e))

    request = SetTargetRequest()
    request.roll_pose.header.frame_id = 'odom'

    if (target_location == "demo1"):
        print("[%s]: using 'demo1' target" % rospy.get_name())
        request.roll_pose.pose.position.x = 9.0
        request.roll_pose.pose.position.y = 3.0
        request.roll_pose.pose.position.z = 0.0
        yaw = -math.pi/2.0
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        request.roll_pose.pose.orientation.x = quat[0]
        request.roll_pose.pose.orientation.y = quat[1]
        request.roll_pose.pose.orientation.z = quat[2]
        request.roll_pose.pose.orientation.w = quat[3]
        # request.roll_pose.pose.orientation.x = 0.0
        # request.roll_pose.pose.orientation.y = 0.0
        # request.roll_pose.pose.orientation.z = -math.sqrt(0.5)
        # request.roll_pose.pose.orientation.w = math.sqrt(0.5)

    elif (target_location == "demo2"):
        print("[%s]: using 'demo2' target" % rospy.get_name())
        request.roll_pose.pose.position.x = 5.0
        request.roll_pose.pose.position.y = -12.0
        request.roll_pose.pose.position.z = 0.0
        yaw = math.pi/4.0
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        request.roll_pose.pose.orientation.x = quat[0]
        request.roll_pose.pose.orientation.y = quat[1]
        request.roll_pose.pose.orientation.z = quat[2]
        request.roll_pose.pose.orientation.w = quat[3]

    elif (target_location == "maneuver1"):
        print("[%s]: using 'maneuver1' target" % rospy.get_name())
        request.roll_pose.pose.position.x = -9.0
        request.roll_pose.pose.position.y = 7.0
        request.roll_pose.pose.position.z = 0.0
        request.roll_pose.pose.orientation.x = 0.0
        request.roll_pose.pose.orientation.y = 0.0
        request.roll_pose.pose.orientation.z = -0.3826834
        request.roll_pose.pose.orientation.w = 0.9238795

    elif (target_location == "grasp1"):
        print("[%s]: using 'grasp1' target" % rospy.get_name())
        request.roll_pose.pose.position.x = -6.0
        request.roll_pose.pose.position.y = 0.0
        request.roll_pose.pose.position.z = 0.0
        request.roll_pose.pose.orientation.x = 0.0
        request.roll_pose.pose.orientation.y = 0.0
        request.roll_pose.pose.orientation.z = 0.0
        request.roll_pose.pose.orientation.w = 1.0

    elif (target_location == "test"):
        print("[%s]: using 'test' target" % rospy.get_name())
        request.roll_pose.pose.position.x = -5.0
        request.roll_pose.pose.position.y = 2.0
        request.roll_pose.pose.position.z = 0.0
        yaw = -math.pi/4.0
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        request.roll_pose.pose.orientation.x = quat[0]
        request.roll_pose.pose.orientation.y = quat[1]
        request.roll_pose.pose.orientation.z = quat[2]
        request.roll_pose.pose.orientation.w = quat[3]

    else:
        print("Unknown target location set.")
        return

    resp = set_target(request)
    print(resp)


try:
    main()
except rospy.ROSInterruptException:
    pass
