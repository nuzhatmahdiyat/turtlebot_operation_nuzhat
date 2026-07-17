#!/usr/bin/env python3
"""
direction_autopilot_node.py

Drives the robot autonomously using LiDAR data, avoiding obstacles
with a forward -> turn -> reverse state machine. Also exposes a
/set_direction service so an operator can override the robot's
movement at any time (forward, reverse, left, right).
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from obstacle_direction_interfaces.srv import SetDirection


class DirectionAutopilotController(Node):
    """Drives the robot, avoids obstacles, and accepts direction overrides."""

    def __init__(self):
        super().__init__('direction_autopilot_controller')

        # Subscriber: reads LiDAR data from the robot's sensor.
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publisher: sends movement commands to the robot.
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Service: lets an operator override the current direction at any time.
        self.direction_service = self.create_service(
            SetDirection, '/set_direction', self.set_direction_callback)

        # Tunable parameters.
        self.obstacle_threshold = 0.50       # front distance that triggers a turn
        self.free_forward_threshold = 1.00   # front distance clear enough to go forward again
        self.turn_safety = 0.40              # side clearance needed to turn that way
        self.forward_velocity = 0.20
        self.reverse_velocity = -0.10
        self.angular_velocity = 0.50

        # State tracking.
        self.state = 'forward'       # forward -> turn -> reverse
        self.turning_direction = 0   # 1 = left, -1 = right, 0 = undecided

        self.get_logger().info('Direction autopilot started. State: FORWARD')

    def scan_callback(self, msg: LaserScan):
        """Runs every time new LiDAR data arrives. Measures distances in
        front, left, and right, then runs the control logic.
        """
        front = self._sector_distance(msg, 0.0, math.radians(30))
        left = self._sector_distance(msg, math.pi / 2, math.radians(30))
        right = self._sector_distance(msg, -math.pi / 2, math.radians(30))

        self.get_logger().info(
            f'F:{front:.2f}m | L:{left:.2f}m | R:{right:.2f}m | STATE:{self.state}')

        self._control_robot(front, left, right)

    def _normalize_angle(self, angle):
        """Keep an angle within -pi to pi."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _angle_to_index(self, angle, angle_min, angle_increment, size):
        """Convert an angle (radians) into an index in the LiDAR ranges list."""
        delta = self._normalize_angle(angle) - self._normalize_angle(angle_min)
        if delta < 0.0:
            delta += 2.0 * math.pi
        index = int(round(delta / angle_increment))
        return max(0, min(size - 1, index))

    def _sector_distance(self, msg, center_angle, width, max_distance=5.0):
        """Find the closest valid distance within a slice of the LiDAR
        scan, centered on center_angle, spanning the given width (radians).
        """
        ranges = msg.ranges
        n = len(ranges)
        half_width = width / 2.0

        start_idx = self._angle_to_index(center_angle - half_width, msg.angle_min, msg.angle_increment, n)
        end_idx = self._angle_to_index(center_angle + half_width, msg.angle_min, msg.angle_increment, n)

        if start_idx <= end_idx:
            sector = ranges[start_idx:end_idx + 1]
        else:
            # The sector wraps around the end of the list.
            sector = ranges[start_idx:] + ranges[:end_idx + 1]

        valid = [r for r in sector if 0.1 < r < max_distance]
        return min(valid) if valid else max_distance

    def set_direction_callback(self, request, response):
        """Handle /set_direction service calls. Publishes the requested
        direction immediately, once. Autonomous control resumes on the
        next LiDAR scan (scans arrive continuously, so this is brief).
        """
        direction = request.direction.lower().strip()
        valid_directions = ['forward', 'reverse', 'left', 'right']

        if direction not in valid_directions:
            response.success = False
            response.message = f"Invalid direction '{request.direction}'. Use forward, reverse, left, or right."
            return response

        self._publish_direction(direction)
        self.get_logger().warn(f'OVERRIDE: operator requested {direction}')

        response.success = True
        response.message = f'Direction override to {direction} accepted.'
        return response

    def _publish_direction(self, direction):
        """Build and publish a Twist message for a given direction command."""
        cmd = Twist()
        if direction == 'forward':
            cmd.linear.x = self.forward_velocity
        elif direction == 'reverse':
            cmd.linear.x = self.reverse_velocity
        elif direction == 'left':
            cmd.angular.z = self.angular_velocity
        elif direction == 'right':
            cmd.angular.z = -self.angular_velocity
        self.velocity_publisher.publish(cmd)

    def _control_robot(self, front, left, right):
        """The forward -> turn -> reverse state machine. Decides what
        command to publish based on the current state and the latest
        LiDAR distances.
        """
        cmd = Twist()
        can_turn_left = left > self.turn_safety
        can_turn_right = right > self.turn_safety

        if self.state == 'forward':
            if front <= self.obstacle_threshold:
                self.state = 'turn'
                self.turning_direction = 1 if left >= right else -1
                side = 'LEFT' if self.turning_direction > 0 else 'RIGHT'
                self.get_logger().warn(f'OBSTACLE: front {front:.2f}m. Switching to TURN ({side}).')
            else:
                cmd.linear.x = self.forward_velocity
                self.get_logger().info('ACTION: FORWARD')

        if self.state == 'turn':
            # If the side we're turning toward is now blocked, try the other side.
            if self.turning_direction > 0 and not can_turn_left and can_turn_right:
                self.turning_direction = -1
            elif self.turning_direction < 0 and not can_turn_right and can_turn_left:
                self.turning_direction = 1
            elif self.turning_direction > 0 and not can_turn_left:
                self.turning_direction = 0
            elif self.turning_direction < 0 and not can_turn_right:
                self.turning_direction = 0

            if self.turning_direction == 0:
                if can_turn_left or can_turn_right:
                    self.turning_direction = 1 if left >= right else -1
                else:
                    self.state = 'reverse'
                    self.get_logger().error(
                        f'TRAPPED! No safe turn (L:{left:.2f} R:{right:.2f}). Switching to REVERSE.')

            if self.state == 'turn':
                if front > self.free_forward_threshold:
                    self.state = 'forward'
                    self.turning_direction = 0
                    cmd.linear.x = self.forward_velocity
                    self.get_logger().info('PATH CLEAR. Switching back to FORWARD.')
                else:
                    cmd.angular.z = self.angular_velocity * self.turning_direction
                    side = 'LEFT' if self.turning_direction > 0 else 'RIGHT'
                    self.get_logger().warn(f'ROTATE {side} until front path is free')

        if self.state == 'reverse':
            cmd.linear.x = self.reverse_velocity
            cmd.angular.z = self.angular_velocity if left >= right else -self.angular_velocity
            self.get_logger().error(f'REVERSE and rotate toward safer side (L:{left:.2f} R:{right:.2f})')

            if front > self.free_forward_threshold and (can_turn_left or can_turn_right):
                self.state = 'forward'
                self.turning_direction = 0
                self.get_logger().info('RECOVERED. Switching back to FORWARD.')

        self.velocity_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DirectionAutopilotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
