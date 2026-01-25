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
FASTER_VEL = 90
FASTER_ACC = 90

DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# =========================================================
# 유틸리티 및 동작 함수
# =========================================================
def initialize_robot():
    """로봇 모드 및 툴 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, ROBOT_MODE_AUTONOMOUS
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

def hot_water_wash():
    """온수 세척 시퀀스"""
    from DSR_ROBOT2 import movel, posx, move_periodic, wait

    # 좌표 설정
    hot_top = posx([297.06, -437.00, 540.00, 40.38, -177.85, -47.41])
    hot_down = posx([291.20, -444.58, 308.34, 35.15, -177.60, -52.77])

    print("온수 세척 위치로 이동")
    movel(hot_top, vel=FASTER_VEL, acc=FASTER_ACC)
    movel(hot_down, vel=VEL, acc=ACC)

    print("세척 시작")
    move_periodic(
        amp=[0, 60, 0, 0, 0, 0],
        period=2.0,
        atime=0.5,
        repeat=3,
        ref=0
    )
    move_periodic(
        amp=[0, 0, 30, 0, 0, 0],
        period=2.0,
        atime=0.5,
        repeat=3,
        ref=0
    )

    movel(hot_top, vel=VEL, acc=ACC)
    wait(0.5)

# =========================================================
# ROS2 SERVICE NODE CLASS
# =========================================================
class HotWashService(Node):
    def __init__(self):
        super().__init__("hot_wash_srv", namespace=ROBOT_ID)
        
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        DR_init.__dsr__node = self

        self._lock = threading.Lock()
        self._running = False
        
        self.callback_group = ReentrantCallbackGroup()

        self._srv = self.create_service(
            Trigger, 
            "hot_wash", 
            self._on_trigger,
            callback_group=self.callback_group
        )
        self.get_logger().info(f"[Service Ready] /{ROBOT_ID}/hot_wash")

    def _on_trigger(self, request, response):
        with self._lock:
            if self._running:
                response.success = False
                response.message = "로봇이 이미 세척 동작 중입니다."
                return response
            self._running = True
            
            # 실제 동작은 별도 스레드에서 수행 (서비스 응답 지연 방지)
            threading.Thread(target=self._run_job, daemon=True).start()

        response.success = True
        response.message = "온수 세척 공정을 시작합니다."
        return response
    
    def _run_job(self):
        try:
            self.get_logger().info("온수 세척 시작!")

            DR_init.__dsr__node = self
            
            initialize_robot()

            hot_water_wash()

            self.get_logger().info("온수 세척 완료")

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
    node = HotWashService()
    DR_init.__dsr__node = node
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 중단되었습니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()