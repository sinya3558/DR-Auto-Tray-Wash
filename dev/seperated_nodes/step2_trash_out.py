#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
# 유틸리티 함수
# =========================================================
def _to_list(posj_val):
    if isinstance(posj_val, (list, tuple)):
        return list(posj_val)
    try:
        return list(posj_val)
    except Exception:
        raise RuntimeError(f"Unsupported posj type: {type(posj_val)}")

# =========================================================
# 잔반 처리 시퀀스 (동작부)
# =========================================================
def perform_trash_out():
    from DSR_ROBOT2 import (
        posx, movej, movel, wait, get_current_posj
    )

    # 포지션 정의
    P_PICKING_UP2 = posx(-289.24, -139.26, 544.47, 168.93, 179.38, -9.83)
    P_TRASH_UP    = posx(-19.01, -374.20, 540.00, 80.62, -174.96, -7.70)
    P_TRASH_DOWN  = posx(-54.45, -364.25, 252.28, 87.89, -149.56, -0.96)
    P_TRASH_DOWN_SAFE = posx(-54.45, -364.25, 540.00, 87.89, -149.56, -0.96)

    print("1) move: picking_up2")
    movel(P_PICKING_UP2, vel=VEL, acc=ACC)
    wait(0.5)

    print("2) move: trash_up")
    movel(P_TRASH_UP, vel=VEL, acc=ACC)
    wait(0.5)

    print("3) move: trash_down (via safe)")
    movel(P_TRASH_DOWN_SAFE, vel=VEL, acc=ACC)
    movel(P_TRASH_DOWN,      vel=VEL, acc=ACC)
    wait(0.5)

    print("4) shake: J5 up/down x5")
    base_j = _to_list(get_current_posj())
    j5_center = base_j[4]       
    J5_DELTA_DEG = 5.0

    for i in range(5):
        j_up = base_j.copy()
        j_dn = base_j.copy()
        j_up[4] = j5_center + J5_DELTA_DEG
        j_dn[4] = j5_center - J5_DELTA_DEG
        movej(j_up, vel=VEL, acc=ACC)
        movej(j_dn, vel=VEL, acc=ACC)

    wait(0.5)

    print("5) return: trash_up (via safe)")
    movel(P_TRASH_DOWN_SAFE, vel=VEL, acc=ACC)
    movel(P_TRASH_UP,        vel=VEL, acc=ACC)
    print("DONE")

# =========================================================
# ROS2 SERVICE NODE CLASS
# =========================================================
class TrashOutService(Node):
    def __init__(self):
        super().__init__("trash_out_service", namespace=ROBOT_ID)
        
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        DR_init.__dsr__node = self

        self._lock = threading.Lock()
        self._running = False
        
        self.callback_group = ReentrantCallbackGroup()

        # 서비스명: /dsr01/trash_out
        self._srv = self.create_service(
            Trigger, 
            "trash_out", 
            self._on_trigger,
            callback_group=self.callback_group
        )
        self.get_logger().info(f"[Service Ready] /{ROBOT_ID}/trash_out")

    def _on_trigger(self, request, response):
        with self._lock:
            if self._running:
                response.success = False
                response.message = "로봇이 이미 동작 중입니다 (Trash Out)."
                return response
            self._running = True
            
            threading.Thread(target=self._run_job, daemon=True).start()

        response.success = True
        response.message = "잔반 처리 공정을 시작합니다."
        return response
    
    def _run_job(self):
        try:
            from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS, set_tool, set_tcp
            
            self.get_logger().info("Trash Out 시퀀스 시작")
            
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
            set_tool(ROBOT_TOOL)
            set_tcp(ROBOT_TCP)
            
            perform_trash_out()
            
            self.get_logger().info("잔반 처리 완료")
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
    node = TrashOutService()

    DR_init.__dsr__node = node
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("서비스 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()