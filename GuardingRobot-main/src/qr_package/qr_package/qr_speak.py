import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import os
class QRDescriptionSpeaker(Node):
    def __init__(self):
        super().__init__('qr_description_speaker')
        self.subscription = self.create_subscription(
            String,
            'qr_db_description',
            self.speak_description,
            10
        )
        self.db_check_publisher = self.create_publisher(Bool, '/robot2/qr_done', 10)
        self.prev_description = None  # 최근 읽은 설명 저장
        self.timer = None  # 타이머 참조 저장
    def speak_description(self, msg):
        description = msg.data.strip()
        if not description:
            self.get_logger().warn("Empty description.")
            return
        # 이전 설명과 같으면 무시
        if description == self.prev_description:
            self.get_logger().info(f"Same description received again. Skipping: {description}")
            return
        self.prev_description = description  # 새 설명 저장
        self.get_logger().info(f"Speaking: {description}")
        os.system(f'espeak -v en-us -s 150 -p 50 "{description}"')
        # 기존 타이머 있으면 취소하고 새 타이머 설정
        if self.timer:
            self.timer.cancel()
        self.timer = self.create_timer(2.0, self.publish_qr_done)
    def publish_qr_done(self):
        done_msg = Bool()
        done_msg.data = True
        self.db_check_publisher.publish(done_msg)
        self.get_logger().info("Published /robot2/qr_done=True")
        # 타이머는 1회성이라 자동 제거되지만 참조도 초기화
        self.timer = None
def main(args=None):
    rclpy.init(args=args)
    node = QRDescriptionSpeaker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()