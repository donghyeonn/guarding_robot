import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import CameraInfo

class TfPointTransform(Node):
    def __init__(self):
        super().__init__('tf_point_transform')

        # TF Buffer와 Listener 준비
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 카메라 정보 및 거리 정보 구독
        self.create_subscription(CameraInfo, '/robot3/oakd/rgb/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Float64MultiArray, "/robot3/dist_info", self.dist_callback, 10)

        # 맵 좌표를 퍼블리시할 Publisher 생성
        self.map_pub = self.create_publisher(PointStamped, '/robot3/transformed_point_map', 10)

        # 변수 초기화
        self.fx = 1.
        self.fy = 1.
        self.cx = 1.
        self.cy = 1.
        self.get_logger().info(f"nyaya")
        

    def dist_callback(self, msg: Float64MultiArray):
        try:
            x, y, dist = msg.data
            X = (x - self.cx) * dist / self.fx
            Y = (y - self.cy) * dist / self.fy
            print(f"x, y, dist:{x, y, dist}")
            print(f"X, Y, self.fx, self.fy, self.cx, self.cy:{X, Y, self.fx, self.fy, self.cx, self.cy}")
            
            point_base = PointStamped()
            point_base.header.stamp = self.get_clock().now().to_msg()
            point_base.header.frame_id = 'oakd_rgb_camera_optical_frame'
            point_base.point.x = X
            point_base.point.y = Y
            point_base.point.z = 0.0

            try:
                point_map = self.tf_buffer.transform(
                    point_base,
                    'map',
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )

                self.get_logger().info(f"[Camera] ({point_base.point.x:.2f}, {point_base.point.y:.2f}, {point_base.point.z:.2f})")
                self.get_logger().info(f"[Map]    ({point_map.point.x:.2f}, {point_map.point.y:.2f}, {point_map.point.z:.2f})")

                self.map_pub.publish(point_map)

            except Exception as e:
                self.get_logger().warn(f"TF transform to map failed: {e}")

        except Exception as e:
            self.get_logger().warn(f"dist_callback error: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        data = msg.k
        self.fx = data[0]
        self.fy = data[4]
        self.cx = data[2]
        self.cy = data[5]

def main():
    rclpy.init()
    node = TfPointTransform()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
