from museum_interfaces.srv import VisitOrder
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class VisitOrderService(Node):
    def __init__(self):
        super().__init__('visit_order_service')
        self.srv = self.create_service(VisitOrder, 'get_visit_order', self.handle_visit_order)

        self.cctv1 = self.create_subscription(Int32,'cc1_count', self.cb_cctv_1, 10)
        self.cctv2 = self.create_subscription(Int32,'cc2_count', self.cb_cctv_2, 10)


        self.cctv1_count = 0
        self.cctv2_count = 1
        self.room2_status = 'clear'
        self.room3_status = 'clear'

    def cb_cctv_1(self, msg):
        self.cctv1_count = msg.data
    def cb_cctv_2(self, msg):
        self.cctv2_count = msg.data

    def handle_visit_order(self, request, response):
        order = self.make_visit_order()
        response.orderlist = order
        return response

    def make_visit_order(self):
        room_points = {
            1: 'room1',
            2: 'room2',
            3: 'room3',
            4: 'room4'
        }
        order = []
        
        if self.cctv1_count >= 2:
            self.room2_status = 'crowded'
        else:
            self.room2_status = 'clear'

        if self.cctv2_count >= 2:
            self.room3_status = 'crowded'
        else:
            self.room3_status = 'clear'

        
        
        if self.room2_status == 'clear' and self.room3_status == 'clear':
            order += room_points[1]
            order += room_points[2]
            order += room_points[3]
            self.get_logger().info('1번 전시실 : 원활 , 2번 전시실 : 원활, 3번 전시실 : 원활')
            self.get_logger().info('관람 순서 : 1번 -> 2번 -> 3번 -> 4번')
        elif self.room2_status =='clear' and self.room3_status == 'crowded':
            order += room_points[2]
            order += room_points[1]
            order += room_points[3]
            self.get_logger().info('1번 전시실 : 혼잡 , 2번 전시실 : 원활, 3번 전시실 : 혼잡')
            self.get_logger().info('관람 순서 : 2번 -> 1번 -> 3번 -> 4번')

        elif self.room2_status == 'crowded' and self.room3_status == 'crowded':
            order += room_points[1]
            order += room_points[3]
            order += room_points[2]
            self.get_logger().info('1번 전시실 : 원활 , 2번 전시실 : 혼잡, 3번 전시실 : 혼잡')
            self.get_logger().info('관람 순서 : 1번 -> 3번 -> 2번 -> 4번')

        else:
            order += room_points[1]
            order += room_points[3]
            order += room_points[2]
            self.get_logger().info('1번 전시실 : 원활 , 2번 전시실 : 혼잡, 3번 전시실 : 원활')
            self.get_logger().info('관람 순서 : 1번 -> 3번 -> 2번 -> 4번')


        order += room_points[4]

        return order

def main():
    rclpy.init()
    node = VisitOrderService()
    rclpy.spin(node)
    rclpy.shutdown()
