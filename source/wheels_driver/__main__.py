"""Execute a ROS node using the wheels_driver module.

This node reads messages from a custom topic (external), parses them into
the value that Gazebo expects, and then publishes these messages
to Gazebo (internal). For the wheels driver, these messages are Twist values
that represent the linear and angular velocity that should be applied to wheels.

The abstraction done by this node is necessary to be able to switch
between hardware and software; a similarly written hardware node could
listen to the same istructions topic and command actuators, instead of the sim.
"""
import std_msgs.msg
import rclpy

NODE = "wheels_driver"
WHEELS_EXTERNAL_TOPIC = "wheel_instructions"
WHEELS_INTERNAL_TOPIC = "diff_drive_controller/cmd_vel"
MAX_VELOCITY = 5

QUEUE_SIZE = 10

# Dictionary values set after publishers get created in main()
publishers = {}


def handle_wheel_movements(twist):

    diff_drive_twist = std_msgs.msg.Twist()

    # Although there are caps on the speed in the diff_drive, this helps the
    # rover not turn incredibly slow when making lefts, rights, donuts, etc.

    diff_drive_twist.linear.x = twist.linear.x * MAX_VELOCITY
    diff_drive_twist.linear.y = twist.linear.y
    diff_drive_twist.linear.z = twist.linear.z
    diff_drive_twist.angular.x = twist.angular.x
    diff_drive_twist.angular.y = twist.angular.y
    diff_drive_twist.angular.z = twist.angular.z * MAX_VELOCITY

    publishers[WHEELS_INTERNAL_TOPIC].publish(diff_drive_twist)


def main(passed_args=None):
    """Main entry point for the ROS node."""
    try:
        rclpy.init(args=passed_args)
        node = rclpy.create_node(NODE)

        # Create publishers to Gazebo diff_drive managers.
        publishers[WHEELS_INTERNAL_TOPIC] = node.create_publisher(
            std_msgs.msg.Float64, WHEELS_INTERNAL_TOPIC, QUEUE_SIZE
        )

        # Create subscriptions to listen for specific robot actions from users.
        node.create_subscription(
            std_msgs.msg.Float64,
            WHEELS_EXTERNAL_TOPIC,
            handle_wheel_movements,
            QUEUE_SIZE,
        )

        node.get_logger().info("wheels_driver node created successfully")

        # Spin!
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
