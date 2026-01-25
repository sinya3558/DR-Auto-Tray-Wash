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

DO_CLOSE = 1
DO_OPEN  = 2
DO_WASH  = 3
DO_BRUSH = 4

DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# =========================================================
# 유틸리티 함수 (그리퍼 조작)
# =========================================================
def gripper_open():
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(DO_OPEN, 1)
    set_digital_output(DO_CLOSE, 0)
    set_digital_output(DO_WASH, 0)
    set_digital_output(DO_BRUSH, 0)
    wait(0.3)

def gripper_close():
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(DO_OPEN, 0)
    set_digital_output(DO_CLOSE, 1)
    set_digital_output(DO_WASH, 0)
    set_digital_output(DO_BRUSH, 0)
    wait(0.3)

# =========================================================
# 식판 집기 시퀀스 (동작부)
# =========================================================
def perform_pick_tray():
    from DSR_ROBOT2 import (
        movej, movejx, movel, posx,
        task_compliance_ctrl, release_compliance_ctrl,
        set_desired_force, get_current_posx, wait
    )

    print("시퀀스 시작: 홈 포지션 이동")
    home_j = [0, 0, 90, 0, 90, 0]
    home_x = posx([367.28, 8.93, 422.97, 4.37, 179.98, 3.97])
    movej(home_j, vel=VEL, acc=ACC)
    movejx(home_x, vel=VEL, acc=ACC)
    wait(0.5)

    print("식판 포지션으로 접근")
    tray_approach = posx([-289.24, -141, 544.47, 168.93, 179.38, -9.83])
    movejx(tray_approach, vel=VEL, acc=ACC, time=5.0)
    wait(0.5)

    gripper_open()

    print("힘 제어 기반 기준면 확보 중...")
    task_compliance_ctrl()
    set_desired_force([5, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0])

    approach_x = posx([-289.24, -139.26, 544.47, 168.93, 179.38, -9.83])
    movel(approach_x, vel=VEL, acc=VEL)
    
    release_compliance_ctrl()

    print("수직 하강 및 파지")
    cp = get_current_posx()[0]
    tray_down = posx([cp[0], cp[1], cp[2] - 290, cp[3], cp[4], cp[5]])
    movel(tray_down, vel=VEL, acc=ACC)

    gripper_close()

    print("식판 들어올림")
    movel(cp, vel=VEL, acc=ACC)

# =========================
# ROS2 SERVICE NODE CLASS
# =========================
class PickTrayService(Node):
    def __init__(self):
        super().__init__("pick_tray_srv", namespace=ROBOT_ID)
        
        # DR_init 연결
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        DR_init.__dsr__node = self
        print("a")
        self._lock = threading.Lock()
        self._running = False
        
        # 병렬 처리를 위한 콜백 그룹
        self.callback_group = ReentrantCallbackGroup()

        # 서비스 생성 (Trigger 타입 사용)
        self._srv = self.create_service(
            Trigger, 
            "pick_tray", 
            self._on_trigger,
            callback_group=self.callback_group
        )
        self.get_logger().info(f"[Service Ready] /{ROBOT_ID}/pick_tray")

    def _on_trigger(self, request, response):
        print("b")
        with self._lock:
            if self._running:
                response.success = False
                response.message = "로봇이 이미 동작 중입니다 (Pick Tray)."
                return response
            self._running = True
            
            # 별도 스레드에서 동작 실행
            threading.Thread(target=self._run_job, daemon=True).start()

        response.success = True
        response.message = "식판 집기 공정을 시작합니다."
        return response
    
    def _run_job(self):
        print("c")
        try:
            from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS, set_tool, set_tcp
            
            self.get_logger().info("Pick Tray 기능 시작!")
            
            # 로봇 초기화
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
            set_tool(ROBOT_TOOL)
            set_tcp(ROBOT_TCP)
            
            # 동작 실행
            perform_pick_tray()
            
            self.get_logger().info("식판 집기 완료")
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
    node = PickTrayService()

    DR_init.__dsr__node = node
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("keyboard interrupted by user (STOP)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()