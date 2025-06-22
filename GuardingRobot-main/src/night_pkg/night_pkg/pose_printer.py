# tf_point_transform.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs  # 꼭 필요
import math
from std_msgs.msg import Float64MultiArray
import time
from visualization_msgs.msg import Marker

class TfPointTransform(Node):
    def __init__(self):
        super().__init__('tf_point_transform')
        self.dist = None
        # TF Buffer와 Listener 준비
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 5초 후에 변환 시작
        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        self.create_subscription(Float64MultiArray, "/robot2/dist_info", self.transform_callback, 10)
        time.sleep(5)
        # self.start_timer = self.create_timer(5.0, self.start_transform)

        self.point_map_publisher = self.create_publisher(PointStamped, "/robot2/transformed_point_map", 5)

        self.marker_pub = self.create_publisher(Marker, 'detected_objects_marker', 10)
        self.marker_id = 0

    def update_dist(self, msg):
        self.dist = msg.data[-1]
        # print(f"updated to {self.dist}")  

    def start_transform(self):
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")

        # 주기적 변환 타이머 등록
        self.transform_timer = self.create_timer(2.0, self.timer_callback)

        # 시작 타이머 중지 (한 번만 실행)
        self.start_timer.cancel()

    def transform_callback(self, msg):
        distance = msg.data[-1]
        distance *= 0.8

        try:
            # base_link 기준 포인트 생성
            point_base = PointStamped()
            point_base.header.stamp = rclpy.time.Time().to_msg()
            point_base.header.frame_id = 'base_link'
            point_base.point.x = distance
            point_base.point.y = 0.0
            point_base.point.z = 0.0

            # base_link → map 변환 (회전 포함)
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            # 회전 정보 (quaternion → yaw)
            q = transform.transform.rotation
            yaw = self.quaternion_to_yaw(q)

            # 위치 변환 (point 기준)
            point_map = self.tf_buffer.transform(

                point_base,
                'map',
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            self.get_logger().info(f"[Base_link] ({point_base.point.x:.2f}, {point_base.point.y:.2f}, {point_base.point.z:.2f}), yaw: {yaw:.3f}")
            self.get_logger().info(f"[Map]       ({point_map.point.x:.2f}, {point_map.point.y:.2f}, {point_map.point.z:.2f}), yaw: {yaw:.3f}")
            self.point_map_publisher.publish(point_map)
            self.publish_marker(point_map.point.x, point_map.point.y, point_map.point.z)
        except Exception as e:
            self.get_logger().warn(f"TF 변환 실패: {e}")


    def timer_callback(self):
        try:
            # base_link 기준 포인트 생성
            point_base = PointStamped()
            point_base.header.stamp = rclpy.time.Time().to_msg()
            point_base.header.frame_id = 'base_link'
            point_base.point.x = self.dist
            point_base.point.y = 0.0
            point_base.point.z = 0.0

            # base_link → map 변환 (회전 포함)
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            # 회전 정보 (quaternion → yaw)
            q = transform.transform.rotation
            yaw = self.quaternion_to_yaw(q)

            # 위치 변환 (point 기준)
            point_map = self.tf_buffer.transform(
                point_base,
                'map',
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            self.get_logger().info(f"[Base_link] ({point_base.point.x:.2f}, {point_base.point.y:.2f}, {point_base.point.z:.2f}), yaw: {yaw:.3f}")
            self.get_logger().info(f"[Map]       ({point_map.point.x:.2f}, {point_map.point.y:.2f}, {point_map.point.z:.2f}), yaw: {yaw:.3f}")
            self.point_map_publisher.publish(point_map)
        except Exception as e:
            self.get_logger().warn(f"TF 변환 실패: {e}")

    def publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'detected_objects'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 3
        self.marker_pub.publish(marker)

    def quaternion_to_yaw(self, q):
        # Quaternion → Yaw 변환 함수
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

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
