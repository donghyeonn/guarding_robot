import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64MultiArray
import time
#WAYPOINT_LIST = {0: 'room1_front', 1: 'room2', 2: 'room3_front',3: 'room4_front' ,4: 'lobby_robot2', 99: 'room3_cctv'}
WAYPOINT_LIST = { 0: 'room1_front', 1: 'room2', 2: 'room3_front',3: 'room4_front', 4: 'entrance', 99: 'room3_cctv'}
MAX_INDEX = 5  # 순환 범위 (0~3, 99는 예외용)

class WaypointIndexDictMain(Node):
    def __init__(self):
        super().__init__('waypoint_index_dict_main')
        self.index = 4
        self.index2 = 0
        self.index_msg = "room1_front"
        self.find = False
        self.theif = None
        self.waypoints = WAYPOINT_LIST
        self.max_index = MAX_INDEX
        self.normal_pub = self.create_publisher(String, '/robot3/waypoint_index', 10)
        self.active_sub = self.create_subscription(Bool, '/robot3/waypoint_active', self.active_callback, 10)
        self.robot2_index = self.create_subscription(String, '/robot2/waypoint_index', self.index_callback, 10)
        self.error_sub = self.create_subscription(Bool, '/cctv1/detect', self.error_callback, 10)
        self.thief_find3 = self.create_subscription(Float64MultiArray, 'robot3/dist_info',self.depth_callback, 10)        
        # 1초마다 timer_callback을 실행
        self.timer = self.create_timer(2.0, self.timer_callback)
        
    def index_callback(self,msg):
        if msg.data == 'room4_front':
            if self.index_msg == 'entrance':
                self.index_msg = 'room1_front'
    def depth_callback(self,msg):
        self.theif = msg.data

    def timer_callback(self):
        if self.theif == None:
            if self.find == False:

                self.publish_index(self.index_msg)

    def active_callback(self, msg):
        time.sleep(0.5)
        if self.find == False:
            if not msg.data:
                if self.index_msg =='room1_front':
                    self.index_msg = 'room2'
                elif self.index_msg =='room2':
                    self.index_msg = 'room3_front'
                elif self.index_msg =='room3_front':
                    self.index_msg = 'room4_front'
                elif self.index_msg =='room4_front':
                    self.index_msg = 'entrance'
                elif self.index_msg =='entrance':
                    return

                # 퍼블리시는 timer에서 수행하므로 여기는 호출 안해도 됨
        else:
            return
                # 퍼블리시는 timer에서 수행하므로 여기는 호출 안해도 됨

    def error_callback(self, msg):
        self.find = msg.data
        
        if self.index_msg != "entrance":
            self.index_msg = 'room3_cctv'

    def publish_index(self, idx):
        msg = String()
        # msg.data = self.waypoints[self.index]
        msg.data = idx
        self.normal_pub.publish(msg)
        self.get_logger().info(f'Published waypoint: {msg.data} (index: {self.index_msg})')

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
