import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Float64MultiArray, Bool
import math

class NavigatorWithMapPose(Node):
    def __init__(self):
        super().__init__('navigator_with_map_pose') 
        
        self.create_subscription(Bool, '/cctv1/detect', self.trigger_func_for_r3, 10)
        
        self.nav_navigator = BasicNavigator(namespace='/robot3')
        self.nav_navigator.waitUntilNav2Active()

        self.current_goal = None
        self.robot_pose = None
    
    def trigger_func_for_r3(self, msg: Bool):
        # self.cctv_detected = msg.data
        self.cctv_detected = True

    def point_callback(self):
        # 거리 유효성 체크
        if self.cctv_detected:
            self.get_logger().info('객체 추적 시작')
            x = -3.66
            y = 1.76
            yaw_deg = 0.99

            self.create_pose(x, y, yaw_deg)
            self.get_logger().info('객체 추적 완료')


                

    def create_pose(self, x, y, yaw_deg):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
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
        navigator.nav_navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()