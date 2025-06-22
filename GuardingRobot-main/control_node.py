import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64MultiArray

WAYPOINT_LIST = {0: 'entrance', 1: 'room1_front', 2: 'room2', 3: 'room3_front',4: 'room4_front' ,5: 'lobby_robot2', 99: 'room3_cctv'}
#WAYPOINT_LIST = {0: 'entrance', 1: 'room1_front', 2: 'room2', 3: 'room3_front',4: 'room4_front' , 99: 'room3_cctv'}
MAX_INDEX = 6  # 순환 범위 (0~3, 99는 예외용)

class WaypointIndexDictMain(Node):
    def __init__(self):
        super().__init__('waypoint_index_dict_main')
        self.index2 = 0
        self.index3 = 0
        self.waypoints = WAYPOINT_LIST
        self.max_index = MAX_INDEX
        self.normal_pub2 = self.create_publisher(String, '/robot2/waypoint_index', 10)
        self.normal_pub3 = self.create_publisher(String, '/robot3/waypoint_index', 10)
        self.active_sub2 = self.create_subscription(Bool, '/robot2/waypoint_active', self.active2_callback, 10)
        self.active_sub3 = self.create_subscription(Bool, 'robot3/waypoint_active', self.active3_callback, 10)
        self.cctv_sub = self.create_subscription(Bool, '/cctv1/detect', self.cctv_callback, 10)
        self.thief_find2 = self.create_subscription(Float64MultiArray, 'robot2/dist_info',self.depth_callback, 10)
        self.thief_find3 = self.create_subscription(Float64MultiArray, 'robot3/dist_info',self.depth_callback, 10)

        self.find = False
        self.thief = None
        # 1초마다 timer_callback을 실행
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if self.thief == None:
            if self.index2 != 5:
                self.publish_index2(self.index2)
            if self.index3 != 5:
                self.publish_index3(self.index3)
        else :
            return
        
    def active2_callback(self, msg):
        
        if self.find == False:
            if not msg.data:
                if self.index2 != 5:
                    self.index2 += 1
                else:
                    self.index2 == 0
                if self.index3 == 4:
                    self.index2 == 0
                # 퍼블리시는 timer에서 수행하므로 여기는 호출 안해도 됨
        else:
            if msg.data and self.index2 != 0:
                self.index2 = 99
                # 퍼블리시는 timer에서 수행하므로 여기는 호출 안해도 됨
    
    def active3_callback(self, msg):
        
        if self.find == False:
            if not msg.data:
                if self.index3 != 5:
                    self.index3 += 1
                else:
                    self.index3 == 0
                if self.index3 == 4:
                    self.index2 == 0
                # 퍼블리시는 timer에서 수행하므로 여기는 호출 안해도 됨
        else:
            if msg.data and self.index2 != 0:
                self.index2 = 99
                # 퍼블리시는 timer에서 수행하므로 여기는 호출 안해도 됨

    def cctv_callback(self, msg):
        self.find = True
        # self.find = msg.data

    def depth_callback(self, msg):
        self.thief = msg.data

    def publish_index2(self, idx):
        msg = String()
        msg.data = self.waypoints[idx]
        self.normal_pub2.publish(msg)
        self.get_logger().info(f'Published waypoint: {msg.data} (index: {self.index})')

    def publish_index3(self, idx):
        msg = String()
        msg.data = self.waypoints[idx]
        self.normal_pub3.publish(msg)
        self.get_logger().info(f'Published waypoint: {msg.data} (index: {self.index})')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointIndexDictMain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
