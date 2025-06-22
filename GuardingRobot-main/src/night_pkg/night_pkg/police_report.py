import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

class PoliceReport(Node):
    def __init__(self):
        super().__init__('police_report_node')

        # 도둑 탐지 데이터 로봇2, 로봇3 구독자(x, y, depth)
        self.thief_detected_robot2 = self.create_subscription(
            Float64MultiArray,
            '/robot2/dist_info',
            self.thief_detected_callback,
            5)
        self.thief_detected_robot3 = self.create_subscription(
            Float64MultiArray,
            '/robot3/dist_info',
            self.thief_detected_callback,
            5)

        # 도둑이 발견되면, 경찰에 신고하는 로봇2, 로봇3 퍼블리셔
        self.police_report_pub_robot2 = self.create_publisher(Bool, '/robot2/police_report', 5)
        self.police_report_pub_robot3 = self.create_publisher(Bool, '/robot3/police_report', 5)

    def thief_detected_callback(self, msg):
        if msg.data:
            self.get_logger().info('도둑 발견! 경찰에 신고 중...')
            self.report_to_police(True)
        else:
            self.get_logger().info('도둑 없음')
            self.report_to_police(False)

    def report_to_police(self, is_thief_detected):
        police_report_msg = Bool()
        police_report_msg.data = is_thief_detected
        # True/False
        self.police_report_pub_robot2.publish(police_report_msg)
        self.police_report_pub_robot3.publish(police_report_msg)
        if is_thief_detected:
            self.get_logger().info('경찰 신고 완료')

def main(args=None):
    rclpy.init(args=args)
    node = PoliceReport()
    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
