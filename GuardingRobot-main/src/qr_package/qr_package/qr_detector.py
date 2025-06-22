import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import json
def detect_qr_codes(image):
    qr_detector = cv2.QRCodeDetector()
    detect_data = []
    retval, decoded_infos, points, _ = qr_detector.detectAndDecodeMulti(image)
    if retval and points is not None:
        for d, p in zip(decoded_infos, points):
            if d:
                corners = np.int32(p).reshape(-1, 2)
                cv2.polylines(image, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
                cv2.putText(image, d, tuple(corners[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                detect_data.append(d)
    return image, detect_data
class QRCodeDetector(Node):
    def __init__(self):
        super().__init__('qr_code_detector')
        self.active = False
        self.bridge = CvBridge()
        self.trigger_subscriber = self.create_subscription(
            Bool,
            '/robot2/point_arrive',
            self.trigger_callback,
            10
        )
        self.image_subscriber = self.create_subscription(
            Image,
            '/robot2/oakd/rgb/preview/image_raw',
            self.listener_callback,
            10
        )
        self.qr_publisher = self.create_publisher(String, 'detected_qr_id', 10)
    def trigger_callback(self, msg):
        if msg.data:
            self.active = True
            self.get_logger().info(" QR Detection Triggered")
    def listener_callback(self, msg):
        if not self.active:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image convert error: {e}")
            return
        frame, detect_data = detect_qr_codes(frame)
        if not detect_data:
            return
        for qr_content in detect_data:
            self.get_logger().info(f"QR Raw: {qr_content}")
            try:
                data_dict = json.loads(qr_content)
                id_value = data_dict.get("id", "")
                if id_value:
                    id_msg = String()
                    id_msg.data = id_value
                    self.qr_publisher.publish(id_msg)
                    self.get_logger().info(f" Published ID: {id_value}")
                    self.active = False
                    break
            except json.JSONDecodeError:
                self.get_logger().warn("QR data is not JSON.")
def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()