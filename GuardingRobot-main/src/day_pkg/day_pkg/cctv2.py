import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

from ultralytics import YOLO
import cv2
import time


class cc2(Node):
    def __init__(self):
        super().__init__('cc2')

        # YOLO 모델 로드 (경로 수정 가능)
        self.model = YOLO('yolo11n.pt')

        # 웹캠 연결 (0, 1, 2 중에 상황 맞게 선택)
        self.cap = cv2.VideoCapture(2)

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
        self.count_pub = self.create_publisher(Int32, 'cc2_count', 10)

        self.get_logger().info("YOLO 추론 노드 시작 완료")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("프레임 캡처 실패")
            return

        current_time = time.time()

        # YOLO 추론은 0.3초 간격으로 제한
        if current_time - self.last_infer_time > 0.3:
            results = self.model.track(frame, persist=True, imgsz=320, show=False)
            boxes = results[0].boxes

            # 클래스 0은 사람
            self.people_count = sum(int(cls) == 0 for cls in boxes.cls)
            self.annotated_frame = results[0].plot()
            self.last_infer_time = current_time

            # 퍼블리시: 사람 수
            count_msg = Int32()
            count_msg.data = self.people_count
            self.count_pub.publish(count_msg)

        elif self.annotated_frame is None:
            self.annotated_frame = frame.copy()

        # OpenCV 창에 시각화
        resized = cv2.resize(self.annotated_frame, (640, 480))
        cv2.putText(resized, f"cc2 People: {self.people_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 혼잡/여유 상태 표시
        state_text = "Congested" if self.people_count >= 4 else "Relaxed"
        cv2.putText(resized, f"state: {state_text}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255) if state_text == "Congested" else (255, 255, 0), 2)

        cv2.imshow("YOLO Crowd Tracker", resized)


        # 'q' 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("종료 요청 감지됨")
            self.destroy_node()
            self.cap.release()
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = cc2()
        rclpy.spin(node)
    except Exception as e:
        print(f"[에러] 노드 실행 중 예외 발생: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
