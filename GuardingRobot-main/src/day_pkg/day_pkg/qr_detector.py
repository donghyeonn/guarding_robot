import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json
import mysql.connector
import os
import time
from collections import deque

class QRTaskNode(Node):
    def __init__(self):
        super().__init__('qr_task_node')

        self.robot_ns = '/robot2'
        self.bridge = CvBridge()
        self.image_topic = f'{self.robot_ns}/oakd/rgb/preview/image_raw'
        self.image_sub = None

        # 내부 상태
        self._active = False
        self._trigger_history = deque(maxlen=5)
        self._detected_id = None
        self._start_time = None
        self._timeout_sec = 13.0

        # QR 상태 발행 관련
        self.qr_done_state = True  # True: 대기 가능, False: 작업 중
        self.qr_status_pub = self.create_publisher(Bool, f'{self.robot_ns}/qr_complete', 10)
        self.status_timer = self.create_timer(0.5, self.publish_status)  # 상태 주기적 발행

        # QR 트리거 구독
        self.qr_trigger_sub = self.create_subscription(
            Bool, f'{self.robot_ns}/qr_trigger', self.trigger_cb, 10)

        # 이미지 처리 타이머
        self.timer = self.create_timer(0.2, self.timer_cb)

        # DB 연결
        self.conn = mysql.connector.connect(
            host="localhost",
            user="root",
            password="rokey1234",
            database="qr_database"
        )
        self.cursor = self.conn.cursor()

        self.get_logger().info(f"[{self.robot_ns}] QRTaskNode 준비 완료")

    def publish_status(self):
        msg = Bool()
        msg.data = self.qr_done_state
        self.qr_status_pub.publish(msg)

    def trigger_cb(self, msg):
        self._trigger_history.append(msg.data)
        if not self._active and sum(self._trigger_history) >= 3:
            self.start_qr_task()

    def start_qr_task(self):
        self._active = True
        self._detected_id = None
        self._start_time = time.time()
        self.qr_done_state = False  # 작업 중 표시
        self.get_logger().info('[QR] QR 태스크 시작됨')
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

    def timer_cb(self):
        if not self._active:
            return
        if (time.time() - self._start_time) > self._timeout_sec:
            self.get_logger().warn('[QR] 타임아웃: QR 인식 실패')
            self.finish_task()

    def image_callback(self, msg):
        if not self._active:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
            self.finish_task()
            return

        qr_id = self.detect_qr(frame)
        if qr_id:
            self.get_logger().info(f"[QR] QR 감지됨: {qr_id}")
            self._detected_id = qr_id
            self.query_db_and_speak(qr_id)
            self.finish_task()

    def detect_qr(self, frame):
        qr_detector = cv2.QRCodeDetector()
        retval, decoded_infos, points, _ = qr_detector.detectAndDecodeMulti(frame)
        if retval and decoded_infos:
            for d in decoded_infos:
                if d:
                    try:
                        data_dict = json.loads(d)
                        qr_id = str(data_dict.get("id", "")).strip()
                        if qr_id:
                            return qr_id
                    except:
                        self.get_logger().warn("QR 데이터 파싱 실패")
        return None

    def query_db_and_speak(self, qr_id):
        try:
            self.cursor.execute("SELECT description FROM marker_info WHERE id = %s", (qr_id,))
            result = self.cursor.fetchone()
            if result:
                description = result[0]
                self.get_logger().info(f"[DB] 설명: {description}")
                os.system(f'espeak -v en-us -s 150 -p 50 "{description}"')
        except Exception as e:
            self.get_logger().error(f"[DB] 조회 실패: {e}")

    def finish_task(self):
        if not self._active:
            return
        self._active = False
        self._trigger_history.clear()
        self.qr_done_state = True  # 다시 대기 가능 상태로 전환
        self.get_logger().info("[QR] QR 작업 완료 - 상태 True 전환")
    #     self.cleanup_timer = self.create_timer(10.0, self.cleanup_after_qr)

    # def cleanup_after_qr(self):
    #     self.get_logger().info("[QR] 10초 후 cleanup 수행")
    #     # 타이머 한 번만 실행되도록 해제
    #     self.cleanup_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = QRTaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()











