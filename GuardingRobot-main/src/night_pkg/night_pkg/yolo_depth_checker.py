import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
import threading
import os
import sys
from std_msgs.msg import Float64MultiArray, MultiArrayDimension


# ========================
# 상수 정의
# ========================
# YOLO_MODEL_PATH = '/home/mi/rokey_ws/model/yolov8n.pt'
YOLO_MODEL_PATH = '/home/rokey/GuardingRobot/models/best.pt'  # YOLO 모델 경로
RGB_TOPIC = '/robot2/oakd/rgb/preview/image_raw'
DEPTH_TOPIC = '/robot2/oakd/stereo/image_raw'
CAMERA_INFO_TOPIC = '/robot2/oakd/stereo/camera_info'

TARGET_CLASS_ID = [0, 1]  # 예: car
# ========================
BOX_CENTER_PIXEL_NUM = 7 # 1, 9, 25, 15, 45, 7, 3

class YoloDepthDistance(Node):
    def __init__(self):
        super().__init__('yolo_depth_distance')
        self.get_logger().info("YOLO + Depth 거리 출력 노드 시작")

        # YOLO 모델 로드
        if not os.path.exists(YOLO_MODEL_PATH):
            self.get_logger().error(f"YOLO 모델이 존재하지 않습니다: {YOLO_MODEL_PATH}")
            sys.exit(1)
        self.model = YOLO(YOLO_MODEL_PATH)
        self.class_names = getattr(self.model, 'names', [])

        self.bridge = CvBridge()
        self.K = None
        self.rgb_image = None
        self.depth_image = None
        self.lock = threading.Lock()

        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 1)
        self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, 1)
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 1)

        self.dist_info_publisher = self.create_publisher(Float64MultiArray, '/robot2/dist_info', 5)

        # YOLO + 거리 출력 루프 실행
        threading.Thread(target=self.processing_loop, daemon=True).start()

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("CameraInfo 수신 완료")

    def rgb_callback(self, msg):
        with self.lock:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_callback(self, msg):
        with self.lock:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def processing_loop(self):
        cv2.namedWindow("YOLO Distance View", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            with self.lock:
                if self.rgb_image is None or self.depth_image is None or self.K is None:
                    continue
                rgb = self.rgb_image.copy()
                depth = self.depth_image.copy()
            results = self.model(rgb, stream=True)

            for r in results:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    if cls not in TARGET_CLASS_ID:
                        continue

                    # 중심 좌표
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    u, v = (x1 + x2) // 2, (y1 + y2) // 2

                    if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
                        continue

                    # 거리 계산 (mm → m)
                    if BOX_CENTER_PIXEL_NUM == 1: # 기본
                        val = depth[v, u]

                    elif BOX_CENTER_PIXEL_NUM == 9: # 3x3
                        val = np.mean(depth[v-1:v+2, u-1:u+2][(depth[v-1:v+2, u-1:u+2]!=0)|(depth[v-1:v+2, u-1:u+2]!=np.inf)])
                    
                    elif BOX_CENTER_PIXEL_NUM == 25: # 5x5
                        val = np.mean(depth[v-2:v+3, u-2:u+3][(depth[v-2:v+3, u-2:u+3]!=0)|(depth[v-2:v+3, u-2:u+3]!=np.inf)])
                    
                    elif BOX_CENTER_PIXEL_NUM == 15: # 3x5
                        val = np.mean(depth[v-1:v+2, u-2:u+3][(depth[v-1:v+2, u-2:u+3]!=0)|(depth[v-1:v+2, u-2:u+3]!=np.inf)])

                    elif BOX_CENTER_PIXEL_NUM == 45: # 5x9
                        val = np.mean(depth[v-2:v+3, u-4:u+5][(depth[v-2:v+3, u-4:u+5]!=0)|(depth[v-2:v+3, u-4:u+5]!=np.inf)])
                    
                    elif BOX_CENTER_PIXEL_NUM == 7: # 가로 다섯 세로 셋
                        w , h = abs(x2 - x1) , abs(y2 - y1)
                        patch = [
                            depth[v, u-int(2*w/10)],
                            depth[v, u-int(1*w/10)],
                            depth[v, u],
                            depth[v, u+int(1*w/10)],
                            depth[v, u+int(2*w/10)],
                            depth[v+int(1*h/10), u],
                            depth[v-int(1*h/10), u]
                            ]
                        val = np.mean(patch[(patch!=0)|(patch!=np.inf)])
                    
                    elif BOX_CENTER_PIXEL_NUM == 3:
                        w , h = abs(x2 - x1) , abs(y2 - y1)
                        patch = [
                            depth[v-int(1*h/4), u-int(1*w/4)],
                            depth[v, u],
                            depth[v-int(1*h/4), u+int(1*w/4)],
                            ]

                    if depth.dtype == np.uint16:
                        distance_m = val / 1000.0
                    else:
                        distance_m = float(val)
                    
                    msgs = Float64MultiArray()
                    msgs.layout.dim.append(MultiArrayDimension(label='dist_info', size=3, stride=3))
                    msgs.data = [float(u), float(v), float(distance_m)]
                    self.dist_info_publisher.publish(msgs)

                    label = self.class_names[cls] if cls < len(self.class_names) else f'class_{cls}'
                    self.get_logger().info(f"Using {BOX_CENTER_PIXEL_NUM} Pixels")
                    self.get_logger().info(f"{label} at ({u},{v}) → {distance_m:.2f}m")

                    # RGB 이미지 위 시각화
                    cv2.rectangle(rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(rgb, (u, v), 4, (0, 0, 255), -1)
                    cv2.putText(rgb, f"{distance_m:.2f}m", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            cv2.imshow("YOLO Distance View", rgb)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

# ========================
# 메인 함수
# ========================
def main():
    rclpy.init()
    node = YoloDepthDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()