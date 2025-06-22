import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
import time

class Alarmsystem(Node):
    def __init__(self):
        super().__init__('alarm_system_node')

        time.sleep(1)

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

        # 도둑이 발견되면, 경보음을 내는 로봇2, 로봇3 퍼블리셔
        self.alarm_pub_robot2 = self.create_publisher(AudioNoteVector, '/robot2/cmd_audio', 5)
        self.alarm_pub_robot3 = self.create_publisher(AudioNoteVector, '/robot3/cmd_audio', 5)

        self.timer = None
        
        self.detect_result = None
    
    def thief_detected_callback(self, msg):
        # 도둑이 발견되었을 경우
        self.detect_result = msg.data
        if msg.data:
            if self.timer is None:
                self.timer = self.create_timer(1.5, self.publish_alarm)
                self.publish_alarm()  # 최초 알람 발송

    def publish_alarm(self):
        msg = AudioNoteVector()
        msg.notes = [
            AudioNote(frequency=int(880), max_runtime=rclpy.duration.Duration(seconds=0.3).to_msg()),  # 삐
            AudioNote(frequency=int(440), max_runtime=rclpy.duration.Duration(seconds=0.3).to_msg()),  # 뽀
            AudioNote(frequency=int(880), max_runtime=rclpy.duration.Duration(seconds=0.3).to_msg()),  # 삐
            AudioNote(frequency=int(440), max_runtime=rclpy.duration.Duration(seconds=0.3).to_msg()),  # 뽀
        ]

        self.alarm_pub_robot2.publish(msg)
        self.alarm_pub_robot3.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Alarmsystem()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()