import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

class WaypointNavigator(Node):
    """
    NavigateToPose 액션 클라이언트를 클래스 기반으로 구현한 노드.
    지정된 웨이포인트로 로봇을 이동시키는 기능을 수행합니다.
    """
    def __init__(self):
        """노드 초기화 및 액션 클라이언트 생성"""
        super().__init__('nav2_waypoint_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.hana_nav2_state = "None"
        self.retry_count = 0
        self.max_retries = 3
        self.current_waypoint = None
        self.retry_timer = None

    def send_goal(self,waypoint_array):
        """
        목표 지점을 전송합니다.

        :param waypoint_array: [x, y, w] 형태의 목표 좌표
        """
        # 기존 재시도 타이머 취소
        if self.retry_timer:
            self.retry_timer.cancel()
            self.retry_timer = None
        
        # 새로운 목표가 오면 재시도 카운트 리셋
        if self.current_waypoint != waypoint_array:
            self.retry_count = 0
            self.current_waypoint = waypoint_array.copy()
        
        # 실제 목표 전송 실행
        self._send_goal_internal(waypoint_array)

    def feedback_callback(self, feedback_msg):
        """액션 피드백 콜백 함수"""
        remain_dist = feedback_msg.feedback.distance_remaining
        # self.get_logger().info(f"피드백: 남은 거리 {remain_dist:.2f} m")

    def goal_response_callback(self, future):
        """목표 수락/거부 응답 콜백 함수"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('목표가 거부되었습니다.')
            # rclpy.shutdown() # GUI 앱이 계속 실행되도록 종료 코드를 제거합니다.
            return
        
        self.get_logger().info('목표가 수락되었습니다.')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """최종 결과 콜백 함수"""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('목표 지점에 성공적으로 도착했습니다!')
            self.hana_nav2_state = "Done"
            self.retry_count = 0  # 성공 시 재시도 카운트 리셋
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('목표 지점 이동이 중단되었습니다.')
            self._handle_failure()
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('목표 지점 이동이 취소되었습니다.')
            self._handle_failure()
        else:
            self.get_logger().info(f'알 수 없는 결과 상태: {status}')
            self._handle_failure()

        # rclpy.shutdown() # GUI 앱이 계속 실행되도록 종료 코드를 제거합니다.
    
    def _handle_failure(self):
        """이동 실패 처리"""
        if self.retry_count < self.max_retries and self.current_waypoint is not None:
            self.retry_count += 1
            self.get_logger().warn(f'재시도 {self.retry_count}/{self.max_retries}: 2초 후 다시 시도합니다.')
            
            # 기존 타이머 취소
            if self.retry_timer:
                self.retry_timer.cancel()
            
            # 2초 후 재시도를 위한 타이머 생성 (일회성)
            self.retry_timer = self.create_timer(2.0, self._retry_goal)
        else:
            self.get_logger().error(f'최대 재시도 횟수({self.max_retries})에 도달했습니다. 이동을 포기합니다.')
            self.hana_nav2_state = "Failed"
            self.retry_count = 0
    
    def _retry_goal(self):
        """재시도를 위한 타이머 콜백"""
        # 타이머 취소 (일회성 실행)
        if self.retry_timer:
            self.retry_timer.cancel()
            self.retry_timer = None
        
        if self.current_waypoint is not None:
            self.get_logger().info(f'목표 지점 재시도 중: ({self.current_waypoint[0]}, {self.current_waypoint[1]})')
            # 재시도 시에는 같은 좌표이므로 직접 실행
            self._send_goal_internal(self.current_waypoint)
    
    def _send_goal_internal(self, waypoint_array):
        """내부 목표 전송 (재시도 카운트 리셋 없이)"""
        self.get_logger().info("액션 서버를 기다리는 중...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("액션 서버를 사용할 수 없습니다. Nav2가 실행 중인지 확인하세요.")
            self.hana_nav2_state = "Failed"
            return

        goal_msg = NavigateToPose.Goal()
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = waypoint_array[0]
        pose.pose.position.y = waypoint_array[1]
        pose.pose.orientation.w = waypoint_array[2]
        goal_msg.pose = pose

        self.get_logger().info(f"목표 지점 전송 중: ({waypoint_array[0]}, {waypoint_array[1]})")
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)
    
    # WaypointNavigator 클래스의 인스턴스 생성
    navigator = WaypointNavigator()
    
    # 웨이포인트 (x, y, w)를 설정하여 목표 전송
    navigator.send_goal(x=0.0, y=-0.0, w=0.0)
    
    # rclpy.spin()으로 노드를 실행하고 콜백을 기다립니다.
    # get_result_callback에서 rclpy.shutdown()이 호출되면 종료됩니다.
    rclpy.spin(navigator)


if __name__ == '__main__':
    main()