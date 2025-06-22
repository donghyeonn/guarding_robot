import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import Bool, String, Float64MultiArray

WAYPOINT_LIST = {0: 'entrance', 1: 'room1_front', 2: 'room2', 3: 'room3_front',4: 'room4_front' ,5: 'lobby_robot2', 99: 'room3_cctv'}
MAX_INDEX = 6  # 순환 범위 (0~3, 99는 예외용)

class WaypointIndexDictMain(LifecycleNode):
    def __init__(self):
        super().__init__('control_node_lifecycle')
        self.index2 = 0
        self.index3 = 0
        self.waypoints = WAYPOINT_LIST
        self.max_index = MAX_INDEX
        self.find = False
        self.thief = None

        # pub/sub/timer 객체 미리 선언 (activate에서 생성)
        self.normal_pub2 = None
        self.normal_pub3 = None
        self.active_sub2 = None
        self.active_sub3 = None
        self.cctv_sub = None
        self.thief_find2 = None
        self.thief_find3 = None
        self.timer = None

    # ---------- 라이프사이클 콜백들 ----------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # 퍼블리셔/서브스크립션 설정 (타이머는 activate에서)
        self.normal_pub2 = self.create_publisher(String, '/robot2/waypoint_index', 10)
        self.normal_pub3 = self.create_publisher(String, '/robot3/waypoint_index', 10)
        self.active_sub2 = self.create_subscription(Bool, '/robot2/waypoint_active', self.active2_callback, 10)
        self.active_sub3 = self.create_subscription(Bool, 'robot3/waypoint_active', self.active3_callback, 10)
        self.cctv_sub = self.create_subscription(Bool, '/cctv1/detect', self.cctv_callback, 10)
        self.thief_find2 = self.create_subscription(Float64MultiArray, 'robot2/dist_info',self.depth_callback, 10)
        self.thief_find3 = self.create_subscription(Float64MultiArray, 'robot3/dist_info',self.depth_callback, 10)
        self.get_logger().info('on_configure called')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # 타이머 활성화
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('on_activate called, timer started')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # 타이머 정지
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        self.get_logger().info('on_deactivate called, timer stopped')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        # 퍼블리셔/서브스크립션 해제
        self.normal_pub2 = None
        self.normal_pub3 = None
        self.active_sub2 = None
        self.active_sub3 = None
        self.cctv_sub = None
        self.thief_find2 = None
        self.thief_find3 = None
        self.get_logger().info('on_cleanup called')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown called')
        return TransitionCallbackReturn.SUCCESS

    # ---------- 기존 기능들 ----------
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
                    self.index2 = 0
                if self.index3 == 4:
                    self.index2 = 0
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
                    self.index3 = 0
                if self.index3 == 4:
                    self.index2 = 0
        else:
            if msg.data and self.index2 != 0:
                self.index2 = 99

    def cctv_callback(self, msg):
        self.find = True

    def depth_callback(self, msg):
        self.thief = msg.data

    def publish_index2(self, idx):
        msg = String()
        msg.data = self.waypoints[idx]
        self.normal_pub2.publish(msg)
        self.get_logger().info(f'Published waypoint: {msg.data} (index2: {self.index2})')

    def publish_index3(self, idx):
        msg = String()
        msg.data = self.waypoints[idx]
        self.normal_pub3.publish(msg)
        self.get_logger().info(f'Published waypoint: {msg.data} (index3: {self.index3})')


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