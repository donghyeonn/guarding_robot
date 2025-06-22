import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import mysql.connector

class QRDBQueryNode(Node):
    def __init__(self):
        super().__init__('qr_db_query_node')
        self.subscription = self.create_subscription(
            String,
            'detected_qr_id',
            self.qr_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, 'qr_db_description', 10)

        self.conn = mysql.connector.connect(
            host="localhost",
            user="root",
            password="rokey1234",
            database="qr_database"
        )
        self.cursor = self.conn.cursor()

    def qr_callback(self, msg):
        qr_id = msg.data.strip()
        if not qr_id:
            return

        try:
            qr_id = int(qr_id)
        except ValueError:
            self.get_logger().error(f"Invalid QR ID: {qr_id}")
            return

        query = "SELECT description FROM marker_info WHERE id = %s"
        try:
            self.cursor.execute(query, (qr_id,))
            result = self.cursor.fetchone()
            if result:
                description = result[0]
                self.get_logger().info(f"DB Desc: {description}")
                msg_out = String()
                msg_out.data = description
                self.publisher_.publish(msg_out)
            else:
                self.get_logger().warn(f"No data for ID {qr_id}")
        except mysql.connector.Error as err:
            self.get_logger().error(f"MySQL error: {err}")

def main(args=None):
    rclpy.init(args=args)
    node = QRDBQueryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
