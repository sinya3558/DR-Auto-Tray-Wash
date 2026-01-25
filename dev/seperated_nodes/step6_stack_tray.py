import threading
import traceback

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import DR_init

# =========================================================
# 로봇 및 IO 설정
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
# 유틸리티 및 그리퍼 제어 함수
# =========================================================
def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, ROBOT_MODE_AUTONOMOUS
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

def gripper_wash():
    """그리퍼 세척 모드 (DO_WASH만 ON)"""
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(DO_OPEN, 0)
    set_digital_output(DO_CLOSE, 0)
    set_digital_output(DO_WASH, 1)
    set_digital_output(DO_BRUSH, 0)
    wait(0.3)

# =========================================================
# 동작 시퀀스 (Stack)
# =========================================================
def perform_stack():
    from DSR_ROBOT2 import movel, posx, wait

    # 좌표 정의
    on_my_way_mid = posx([-74.630, 429.650, 474.090, 175.36, 177.4, -93.9])
    tray_up2      = posx([-269.990, 132.5, 540.540, 1.5,-178.04, 2.64])
    tray_down2    = posx([-280.75, 133.40, 260.380, 8.29, -178.4, 9.34])
    home_x        = posx([367.28, 8.93, 422.97, 4.37, 179.98, 3.97])

    print("적재 경로로 이동 (중간지점)...")
    movel(on_my_way_mid, vel=VEL, acc=ACC)
    wait(0.2)

    print("적재 위치 포인트 도착")
    movel(tray_up2, vel=VEL, acc=ACC)
    wait(0.2)

    print("하강")
    movel(tray_down2, vel=VEL, acc=ACC)
    wait(0.2)

    print("그리퍼 오픈")
    gripper_wash()
    wait(0.2)

    print("수직 상승")
    movel(tray_up2, vel=VEL, acc=ACC)
    wait(0.2)
    movel(home_x, vel=VEL, acc=ACC)
    print("적재 끝!")

# =========================================================
# ROS2 SERVICE NODE CLASS
# =========================================================
class StackService(Node):
    def __init__(self):
        super().__init__("stack_srv", namespace=ROBOT_ID)
        DR_init.__dsr__node = self

        self._lock = threading.Lock()
        self._running = False
        self.callback_group = ReentrantCallbackGroup()

        self._srv = self.create_service(
            Trigger, 
            "stack_tray", # 서비스 명: stack_tray
            self._on_trigger,
            callback_group=self.callback_group
        )
        self.get_logger().info(f"[Service Ready] /{ROBOT_ID}/stack_tray")

    def _on_trigger(self, request, response):
        with self._lock:
            if self._running:
                response.success = False
                response.message = "로봇이 이미 적재 동작 중입니다."
                return response
            self._running = True
            threading.Thread(target=self._run_job, daemon=True).start()

        response.success = True
        response.message = "적재 공정을 시작합니다."
        return response
    
    def _run_job(self):
        try:
            self.get_logger().info("Stacking 시작!")
            DR_init.__dsr__node = self
            initialize_robot()
            perform_stack()
            self.get_logger().info("Stacking 완료!")
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
    node = StackService()
    DR_init.__dsr__node = node
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()