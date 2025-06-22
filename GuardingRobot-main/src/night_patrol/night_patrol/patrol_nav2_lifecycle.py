
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
from std_msgs.msg import String, Bool
import yaml
import os

from ament_index_python.packages import get_package_share_directory

def create_pose(x, y, yaw_deg, navigator):
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

class Patrol(LifecycleNode):
    def __init__(self):
        super().__init__('patrol_nav2_lifecycle')

        # 노드 변수
        self.navigator = None
        self.dock_navigator = None
        self.locations = None
        self.sub_cmd = None
        self.active = None
        self.find = None

    # ---------- 라이프사이클 콜백들 ----------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # 네비게이터 인스턴스
        self.dock_navigator = TurtleBot4Navigator(namespace='/robot2')
        self.navigator = BasicNavigator(namespace='/robot2')

        # Nav2 활성화 대기
        self.navigator.waitUntilNav2Active()
        # yaml 파일 경로를 패키지에서 가져옴
        package_dir = get_package_share_directory('night_patrol')
        yaml_path = os.path.join(package_dir, 'night_patrol', 'way2.yaml')
        self.locations = self.load_yaml_locations(yaml_path)

        # pub/sub 생성
        self.sub_cmd = self.create_subscription(String, '/robot2/waypoint_index', self.cb_command, 10)
        self.active = self.create_publisher(Bool, '/robot2/waypoint_active', 10)
        self.find = self.create_publisher(Bool, '/robot2/waypoint_find', 10)

        self.get_logger().info('Patrol 노드가 configure 완료됨.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Patrol 노드가 activate됨.')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Patrol 노드가 deactivate됨.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.navigator = None
        self.dock_navigator = None
        self.locations = None
        self.sub_cmd = None
        self.active = None
        self.find = None
        self.get_logger().info('Patrol 노드가 cleanup됨.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Patrol 노드가 shutdown됨.')
        return TransitionCallbackReturn.SUCCESS

    # ---------- 기존 동작 ----------
    def load_yaml_locations(self, yaml_path):
        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)['locations']

    def cb_command(self, msg):
        cmd = msg.data.lower()
        if cmd in self.locations:
            self.navigate_to(cmd)
        else:
            self.get_logger().warn(f'알 수 없는 명령: {cmd}')
    
    def navigate_to(self, location_name):
        self.active.publish(Bool(data=True))
        x, y, yaw = self.locations[location_name]
        target_pose = create_pose(x, y, yaw, self.navigator)
        self.get_logger().info(f'[{location_name} 위치로 이동]')
        self.navigator.goToPose(target_pose)
        while not self.navigator.isTaskComplete():
            pass
        self.get_logger().info(f'[{location_name} 도착]')
        self.active.publish(Bool(data=False))

def main():
    rclpy.init()
    node = Patrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()