#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
from std_msgs.msg import String, Bool
import time
import yaml
import os


def create_pose(x, y, yaw_deg, navigator):
    """x, y, yaw(도 단위) → PoseStamped 생성"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y

    yaw_rad = yaw_deg * 3.141592 / 180.0
    q = quaternion_from_euler(0, 0, yaw_rad)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

class Patrol(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self.dock_navigator = TurtleBot4Navigator(namespace='/robot2')
        self.navigator = BasicNavigator(namespace='/robot2')
        
    
        self.navigator.waitUntilNav2Active()
        
        self.locations = self.load_yaml_locations()

        self.sub_cmd = self.create_subscription(String, '/robot2/waypoint_index', self.cb_command, 10)
        self.active = self.create_publisher(Bool, '/robot2/waypoint_active',10)
        self.find = self.create_publisher(Bool, '/robot2/waypoint_find',10)

    def load_yaml_locations(self):
        yaml_path = os.path.expanduser('~/GuardingRobot/src/night_patrol/night_patrol/way2.yaml')
        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)['locations']

    def cb_command(self,msg):
        cmd = msg.data.lower()
        if cmd in self.locations:
            self.navigate_to(cmd)
        else:
            self.get_logger().warn(f'알 수 없는 명령: {cmd}')
    
    
    def navigate_to(self, location_name):
        x, y, yaw = self.locations[location_name]
        target_pose = create_pose(x, y, yaw, self.navigator)
        self.navigator.goToPose(target_pose)
        time.sleep(0.1)
        feedback = self.navigator.getFeedback()
        if feedback:
            self.active.publish(Bool(data=True))
            print(f'[진행 중] 현재 경로 진행률: {feedback.distance_remaining:.2f}m')
        else:
            self.active.publish(Bool(data=False))
            self.get_logger().info(f'[{location_name} 도착]')
        
            
            

        
def main():
    rclpy.init()
    node = Patrol()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

