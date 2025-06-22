import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from transforms3d.euler import euler2quat
import yaml
import os
import time
from collections import deque
from museum_interfaces.srv import VisitOrder
class Robot3Navigator(Node):
    def __init__(self):
        super().__init__('robot3_navigator')

        # --- Load YAML ---
        yaml_path = os.path.expanduser('~/rokey_ws/routes/points.yaml')
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        self.points = data['points']

        self.cli = self.create_client(VisitOrder, 'get_visit_order')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('VisitOrder service not available, waiting...')

        # 명시적 순서 지정
        self.waypoint_keys = ['lobby', 'entrance']

        # --- Init NAV2 Navigator ---
        self.navigator = BasicNavigator(namespace='/robot2')
        self.navigator.waitUntilNav2Active()

        # QR 통신 관련
        self.qr_finish_history = deque(maxlen=5)
        self.qr_done = True  # QR 완료 여부
        self.qr_trigger = self.create_publisher(Bool , '/robot2/qr_trigger', 10)
        self.qr_finish = self.create_subscription(Bool , '/robot2/qr_complete', self.cb_qr_finish, 10)
        self.time_out = 15.0
        # 시작
        self.run_navigation()

    def to_pose_stamped(self, point_key):
        point = self.points[point_key][0]  # 리스트 안의 첫 dict
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point['x']
        pose.pose.position.y = point['y']
        q = euler2quat(0, 0, point['yaw'])
        pose.pose.orientation.x = q[1]
        pose.pose.orientation.y = q[2]
        pose.pose.orientation.z = q[3]
        pose.pose.orientation.w = q[0]
        return pose

    def run_navigation(self):
        time.sleep(2)  # Nav2 안정화 대기

        i = 0
        while i < len(self.waypoint_keys):
            key = self.waypoint_keys[i]
            pose = self.to_pose_stamped(key)
            self.get_logger().info(f'[Nav] Moving to waypoint [{i+1}/{len(self.waypoint_keys)}]: {key}')
            self.navigator.goToPose(pose)

            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'[Nav] 도착완료 "{key}" successfully')
            else:
                self.get_logger().warn(f'[Nav] Failed to reach "{key}"')

            # ---- QR 처리 ----
            if 'room' in key:
                self.qr_done = False
                self.qr_finish_history.clear()
                msg = Bool(data=True)
                self.start_time = time.time()
                self.get_logger().info('[QR] Triggering QR process...')
                while not self.qr_done and (time.time()-self.start_time) <= self.time_out:
                    self.qr_trigger.publish(msg)
                    rclpy.spin_once(self, timeout_sec=0.2)
                self.get_logger().info('[QR] QR 완료됨. 다음 위치로 진행.')

            # ---- entrance 도착 후 visit order 요청 ----
            if key == 'entrance':
                self.get_logger().info('[Order] Calling VisitOrder service...')
                req = VisitOrder.Request()
                future = self.cli.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                if future.result() is not None:
                    orderlist = future.result().orderlist
                    self.get_logger().info(f'[Order] Received visit order: {orderlist}')
                    self.waypoint_keys += orderlist  # 현재까지 온 위치 이후로 append
                else:
                    self.get_logger().warn('[Order] Failed to receive visit order')
                

            i += 1

        self.get_logger().info('[Done] All waypoints complete')

    def cb_qr_finish(self, msg):
        self.qr_finish_history.append(msg.data)
        true_count = sum(self.qr_finish_history)
        if true_count >= 3:
            self.qr_done = True


def main(args=None):
    rclpy.init(args=args)
    node = Robot3Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
