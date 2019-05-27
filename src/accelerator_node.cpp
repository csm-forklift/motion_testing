/**
 * 'accelerator_node' is intended to read directly from the joystick node and
 * get the analog values from the left joystick. The "Up/Down" axis is the only
 * value of interest and directly controls the signal sent to the accelerator
 * pedal through the Arduino. The accelerator can only control the velocity in
 * a single direction (based on which gear the forklift is in, e.g. forward or
 * reverse).
 *
 * You should use the "X" mode for the controller as this code will be designed
 * for that button/axes configuration.
 *
 * This hsould be used with an Arduino loaded with "accelerator_write.ino"
 */

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>

class AcceleratorNode
{
private:
    // ROS Objects
    ros::NodeHandle nh; // global handle
    ros::NodeHandle nh_; // local handle
    ros::Subscriber joy_sub; // subscribes to raw joystick message
    ros::Publisher throttle_switch_pub; // publishes on/off signal for throttle
    ros::Publisher pedal_fraction_pub; // publishes fraction pedal is pressed

    // Parameters
    int deadman_button; // assigns which button to check in order to send motion commands
    double scale_linear; // scales accelerator command by this factor before sending

public:
    AcceleratorNode() : nh(""), nh_("~")
    {
        // Set up ROS objects
        joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &AcceleratorNode::joyCallback, this);
        throttle_switch_pub = nh_.advertise<std_msgs::Bool>("throttle_switch", 1);
        pedal_fraction_pub = nh_.advertise<std_msgs::Float32>("pedal_fraction", 1);

        // Initialize Parameters
        nh_.param<int>("deadman", deadman_button, 4);
        nh_.param<double>("scale_linear", scale_linear, 1.0);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        // Reads the joystick deadman button, which controls the relay for turning on the pedal control, and the left joystick "Up/Down" axis, which determines the pedal fraction to send to the Arduino to control the velocity.

        // Message data
        std_msgs::Bool throttle_switch;
        std_msgs::Float32 pedal_fraction;

        // Check deadman button
        // If it is pressed, it should cause the throttle switch relay to turn on allowing the forklift to receive velocity commands.
        if (msg->buttons[deadman_button] == 0) {
            throttle_switch.data = false;
            throttle_switch_pub.publish(throttle_switch);

            // Send a 0 velocity signal to be sure there is no motion
            pedal_fraction.data = 0.0;
            pedal_fraction_pub.publish(std_msgs::Float32(pedal_fraction));
        }
        else {
            throttle_switch.data = true;
            throttle_switch_pub.publish(throttle_switch);

            // Grab the left analog "Up/Down" value and scale before sending
            pedal_fraction.data = msg->axes[1];

            // Bound between 0 and 1
            pedal_fraction.data = std::max(0.0, static_cast<double>(pedal_fraction.data));
            pedal_fraction.data = std::min(1.0, static_cast<double>(pedal_fraction.data));

            // Scale
            pedal_fraction.data *= scale_linear;
            pedal_fraction_pub.publish(pedal_fraction);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "accelerator_node");

    AcceleratorNode accelerator_node;

    ros::spin();
    return 0;
}
