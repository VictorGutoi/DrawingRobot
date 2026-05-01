"""Smoke-test publisher: publishes a Twist on /cmd_vel at 10 Hz.

Used to isolate the ROS publish path from the simulator GUI when debugging
why a remote subscriber doesn't see messages.
"""

import time
import rclpy
from geometry_msgs.msg import Twist


def main():
    rclpy.init()
    node = rclpy.create_node('drawingrobot_smoke_pub')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    print(f"[smoke] node='drawingrobot_smoke_pub' topic='/cmd_vel'", flush=True)
    n = 0
    try:
        while rclpy.ok():
            msg = Twist()
            msg.linear.x = 0.1 if (n // 10) % 2 == 0 else -0.1
            msg.angular.z = 0.2 if (n // 10) % 2 == 0 else -0.2
            pub.publish(msg)
            n += 1
            if n % 10 == 0:
                print(f"[smoke #{n:>4}] v={msg.linear.x:+.3f}  ω={msg.angular.z:+.3f}",
                      flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
