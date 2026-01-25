import threading
import traceback

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import DR_init

# =========================================================
# 로봇 기본 설정
# =========================================================
ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL  = "Tool Weight"
ROBOT_TCP   = "GripperDA_v1"

VEL = 60
ACC = 60

DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# =========================================================
# 유틸리티 및 동작 함수
# =========================================================
def initialize_robot():
    """로봇 툴 및 TCP 초기화"""
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, ROBOT_MODE_AUTONOMOUS
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

def perform_cold_wash():
    """냉수 세척 시퀀스 실행"""
    from DSR_ROBOT2 import (
        movel, movejx, posx, wait, move_periodic, 
        get_current_posx, DR_BASE
    )

    # print("시퀀스 시작: 홈 포지션 이동")
    # home_x = posx([367.28, 8.93, 422.97, 4.37, 179.98, 3.97])
    # movejx(home_x, vel=VEL, acc=ACC)
    # wait(0.5)

    # 1. 위치 정의 및 진입
    water_top = posx([275.22, 526.54, 458.35, 1.11, -177.15, -177.39])
    movel(water_top, vel=VEL, acc=ACC)
    
    # 2. 방향 전환 (J6축 +90도)
    cp = get_current_posx()[0]
    change_dir = posx([cp[0], cp[1], cp[2], cp[3], cp[4], cp[5] + 90])
    movel(change_dir, vel=VEL, acc=ACC)
    wait(0.2)

    # 3. 세척통 하강 (Z축 -281mm)
    cold_soak = posx([cp[0], cp[1], cp[2] - 281, cp[3], cp[4], cp[5] + 90])
    movel(cold_soak, vel=VEL, acc=ACC)
    wait(0.2)

    # 4. 세척 동작 (Periodic Move)
    print("냉수 세척 동작 중...")
    move_periodic(
        amp=[60, 60, 0, 0, 0, 0], 
        period=[3.2, 1.6, 0, 0, 0, 0], 
        atime=3.1, 
        repeat=4, 
        ref=DR_BASE
    )
    wait(0.2)

    # 5. 복귀
    print("냉수 세척 완료")
    movel(cp, vel=VEL, acc=ACC) # 원래 높이로
    wait(0.2)
    # movel(home_x, vel=VEL, acc=ACC)
    # wait(0.5)

# =========================================================
# ROS2 SERVICE NODE CLASS
# =========================================================
class ColdWashService(Node):
    def __init__(self):
        super().__init__("cold_wash_srv", namespace=ROBOT_ID)
        
        DR_init.__dsr__node = self

        self._lock = threading.Lock()
        self._running = False
        
        # 병렬 처리를 위한 콜백 그룹
        self.callback_group = ReentrantCallbackGroup()

        # 서비스 생성
        self._srv = self.create_service(
            Trigger, 
            "cold_wash", 
            self._on_trigger,
            callback_group=self.callback_group
        )
        self.get_logger().info(f"[Service Ready] /{ROBOT_ID}/cold_wash")

    def _on_trigger(self, request, response):
        with self._lock:
            if self._running:
                response.success = False
                response.message = "로봇이 이미 동작 중입니다 (Cold Wash)."
                return response
            self._running = True
            
            threading.Thread(target=self._run_job, daemon=True).start()

        response.success = True
        response.message = "냉수 세척 공정을 시작합니다."
        return response
    
    def _run_job(self):
        try:
            self.get_logger().info("Cold Wash 기능 시작!")
            DR_init.__dsr__node = self
            initialize_robot()
            perform_cold_wash()
            self.get_logger().info("냉수 세척 완료")
        except Exception as e:
            self.get_logger().error(f"오류 발생: {e}")
            traceback.print_exc()
        finally:
            with self._lock:
                self._running = False

# =========================================================
# MAIN
# =========================================================
def main(args=None):
    rclpy.init(args=args)
    node = ColdWashService()
    DR_init.__dsr__node = node
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("키보드 중단 신호 감지")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()