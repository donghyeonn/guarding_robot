#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
import time
from std_msgs.msg import Float64MultiArray, Bool
import math

class NavigatorWithMapPose(Node):
    def __init__(self):
        super().__init__('navigator_with_map_pose')
        self.dock_navigator = TurtleBot4Navigator()
        self.nav_navigator = BasicNavigator(namespace='/robot2')
        self.nav_navigator.waitUntilNav2Active()

        self.current_goal = None
        self.last_goal_time = self.get_clock().now()
        self.goal_interval_sec = 1.0  # 최소 이동 주기 (초)
        self.stop_distance = 0.5
        self.robot_pose = None
        self.cctv_detected = False
        self.object_distance = float('inf')  # 초기화


        self.create_subscription(Bool, '/cctv1/detect', self.trigger_func, 10)
        self.create_subscription(PointStamped, '/robot2/transformed_point_map', self.point_callback, 10)
        self.create_subscription(Float64MultiArray, '/robot2/dist_info', self.update_distance, 10)

        self.get_logger().info('객체 추적 모드 시작')

    def trigger_func(self, msg: Bool):
        self.cctv_detected = msg.data

    def update_distance(self, msg: Float64MultiArray):
        distance = msg.data[2]
        if 0.05 < distance < 10.0:  # 거리 유효성 확인 (0보다 크고 너무 멀지 않은 경우)
            self.object_distance = distance
        else:
            self.get_logger().warn(f'무시된 거리 값: {distance:.2f} (유효 범위 초과)')
            self.object_distance = float('inf')

    def point_callback(self, msg: PointStamped):
        if self.cctv_detected:
            # 거리 유효성 체크
            if self.object_distance == float('inf') or self.object_distance < self.stop_distance:
                self.get_logger().info('거리가 너무 가깝거나 유효하지 않음. 이동하지 않음.')
                return

            now = self.get_clock().now()
            time_since_last_goal = (now - self.last_goal_time).nanoseconds / 1e9
            if time_since_last_goal < self.goal_interval_sec:
                return  # 주기 제한

            x = msg.point.x
            y = msg.point.y
            yaw_deg = 0.0

            # 이전 목표와 비교해 변화가 충분한지 확인
            if self.current_goal:
                dx = x - self.current_goal.pose.position.x
                dy = y - self.current_goal.pose.position.y
                if math.hypot(dx, dy) < 0.5:
                    return  # 변화가 작으면 무시

            # 이동 중이면 무시
            if not self.nav_navigator.isTaskComplete():
                self.get_logger().info('이동 중. 새 목표 무시.')
                return

            new_goal = self.create_pose(x, y, yaw_deg)
            self.current_goal = new_goal
            self.last_goal_time = now

            self.get_logger().info(f'새 목표: ({x:.2f}, {y:.2f}) → 이동 시작.')
            self.nav_navigator.goToPose(new_goal)
        else:
            self.get_logger().info('CCTV not detected.')

    def create_pose(self, x, y, yaw_deg):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav_navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        yaw_rad = math.radians(yaw_deg)
        q = quaternion_from_euler(0, 0, yaw_rad)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

def main():
    rclpy.init()
    navigator = NavigatorWithMapPose()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('종료합니다.')
    finally:
        navigator.dock_navigator.destroy_node()
        navigator.nav_navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
