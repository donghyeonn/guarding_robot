import sys
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from std_srvs.srv import SetBool
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout
from PyQt5.QtCore import QTimer

class TurtleBotModeGUI(Node):
    def __init__(self):
        super().__init__('turtlebot_mode_gui')

        # 라이프사이클 관리할 노드명 리스트
        self.day_nodes = []
        self.night_nodes = ['/control_node_lifecycle', '/patrol_nav2_lifecycle']
        self.is_day = True

        # ChangeState 서비스 클라이언트 생성 (노드별)
        self.day_clients = [
            self.create_client(ChangeState, f'{name}/change_state') for name in self.day_nodes
        ]
        self.night_clients = [
            self.create_client(ChangeState, f'{name}/change_state') for name in self.night_nodes
        ]

        self.close_btn = self.create_client(SetBool, 'alarm_btn')


        # Qt 앱 및 윈도우 생성
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle('TurtleBot Mode GUI')

        # 모드 표시 라벨
        self.mode_label = QLabel(self.mode_text())
        self.mode_label.setStyleSheet("font-size: 20px; font-weight: bold;")

        # 종료(Alarm Stop) 버튼 추가
        self.alarm_button = QPushButton("종료(Alarm Stop)")
        self.alarm_button.setStyleSheet("font-size: 16px; color: red;")
        self.alarm_button.clicked.connect(self.call_alarm_stop)


        # 모드 토글 버튼
        self.toggle_button = QPushButton(self.toggle_text())
        self.toggle_button.setStyleSheet("font-size: 18px;")
        self.toggle_button.clicked.connect(self.toggle_mode)

        # 레이아웃 구성
        layout = QVBoxLayout()
        layout.addWidget(self.mode_label)
        layout.addWidget(self.toggle_button)
        layout.addWidget(self.alarm_button)
        self.window.setLayout(layout)
        self.window.setFixedSize(320, 120)
        self.window.show()

        # GUI 주기적 업데이트(ROS spin)
        self.qtimer = QTimer()
        self.qtimer.timeout.connect(self.spin_once)
        self.qtimer.start(50)

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.1)

    def mode_text(self):
        return f'현재 모드: {"주간" if self.is_day else "야간"}'

    def toggle_text(self):
        return "야간 모드로 전환" if self.is_day else "주간 모드로 전환"

    def call_lifecycle_manager(self, client, transition_id):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('lifecycle service 준비 안됨')
            return
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = client.call_async(req)
        self.get_logger().info(f'라이프사이클에 transition {transition_id} 요청')

    def toggle_mode(self):
        if self.is_day:
            # 주간 → 야간 전환: 주간 deactivate, 야간 configure+activate
            for client in self.day_clients:
                self.call_lifecycle_manager(client, Transition.TRANSITION_DEACTIVATE)
                self.call_lifecycle_manager(client, Transition.TRANSITION_CLEANUP)
            for client in self.night_clients:
                self.call_lifecycle_manager(client, Transition.TRANSITION_CONFIGURE)
                self.call_lifecycle_manager(client, Transition.TRANSITION_ACTIVATE)
            self.mode_label.setText('현재 모드: 야간')
            self.toggle_button.setText('주간 모드로 전환')
            self.is_day = False
        else:
            # 야간 → 주간 전환: 야간 deactivate, 주간 configure+activate
            for client in self.night_clients:
                self.call_lifecycle_manager(client, Transition.TRANSITION_DEACTIVATE)
                self.call_lifecycle_manager(client, Transition.TRANSITION_CLEANUP)

            for client in self.day_clients:
                self.call_lifecycle_manager(client, Transition.TRANSITION_CONFIGURE)
                self.call_lifecycle_manager(client, Transition.TRANSITION_ACTIVATE)
            self.mode_label.setText('현재 모드: 주간')
            self.toggle_button.setText('야간 모드로 전환')
            self.is_day = True

    def run(self):
        sys.exit(self.app.exec_())

    def call_alarm_stop(self):
        # SetBool 서비스 호출 (data=False)
        if not self.close_btn.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('alarm_btn 서비스 준비 안됨')
            return
        req = SetBool.Request()
        req.data = False  # 종료 신호
        future = self.close_btn.call_async(req)
        future.add_done_callback(self.handle_alarm_response)
        self.get_logger().info('Alarm 종료 요청 전송!')

    def handle_alarm_response(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f"Alarm 종료 성공: {result.message}")
            else:
                self.get_logger().warn(f"Alarm 종료 실패: {result.message}")
        except Exception as e:
            self.get_logger().error(f"Alarm 종료 서비스 응답 오류: {e}")

    def run(self):
        sys.exit(self.app.exec_())


def main():
    rclpy.init()
    gui = TurtleBotModeGUI()
    gui.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()