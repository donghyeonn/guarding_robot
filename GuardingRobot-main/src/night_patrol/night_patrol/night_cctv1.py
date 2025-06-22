import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import time



class cctv1(Node):
    def __init__(self):
        super().__init__('night_cctv1')

        # YOLO 모델 로드 (경로 수정 가능)
        self.model = YOLO('yolo11n.pt')

        # 웹캠 연결 (0, 1, 2 중에 상황 맞게 선택)
        self.cap = cv2.VideoCapture(4)

        if not self.cap.isOpened():
            self.get_logger().error("웹캠을 열 수 없습니다.")
            raise RuntimeError("웹캠 실패")

        # 타이머 설정 (약 30fps)
        self.timer = self.create_timer(0.03, self.timer_callback)

        # 상태 초기화
        self.last_infer_time = 0
        self.people_count = 0
        self.annotated_frame = None

        # 퍼블리셔
        self.detect_people = self.create_publisher(Bool, '/cctv1/detect', 10)
        # ✅ 이미지 퍼블리시를 위한 브릿지와 퍼블리셔
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/cctv1/image_raw', 10)
        self.get_logger().info("YOLO 추론 노드 시작 완료")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("프레임 캡처 실패")
            return

        current_time = time.time()

        if current_time - self.last_infer_time > 0.3:
            results = self.model.predict(frame)
            boxes = results[0].boxes
            self.people_count = sum(int(cls) == 0 for cls in boxes.cls)
            self.annotated_frame = results[0].plot()
            self.last_infer_time = current_time

        elif self.annotated_frame is None:
            self.annotated_frame = frame.copy()

        resized = cv2.resize(self.annotated_frame, (640, 480))
        cv2.putText(resized, f"cc2 People: {self.people_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        state_text = "warnning" if self.people_count >= 1 else ""
        cv2.putText(resized, f"{state_text}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255) if state_text == "warnning" else (255, 255, 0), 2)
        # ✅ 이미지 퍼블리시
        try:
            ros_image = self.bridge.cv2_to_imgmsg(resized, encoding="bgr8")
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"이미지 퍼블리시 실패: {e}")


        # ✅ Bool 메시지 퍼블리시
        msg = Bool()
        msg.data = self.people_count >= 1
        # msg.data = True
        self.detect_people.publish(msg)

        cv2.imshow("YOLO Crowd Tracker", resized)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("종료 요청 감지됨")
            self.destroy_node()
            self.cap.release()
            cv2.destroyAllWindows()



def main(args=None):
    rclpy.init(args=args)
    try:
        node = cctv1()
        rclpy.spin(node)
    except Exception as e:
        print(f"[에러] 노드 실행 중 예외 발생: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
