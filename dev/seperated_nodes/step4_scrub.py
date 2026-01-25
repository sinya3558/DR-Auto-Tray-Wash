import threading
import traceback

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import DR_init

# =========================================================
# GLOBAL SETTINGS (로봇 및 속도 설정)
# =========================================================
ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL  = "Tool Weight"
ROBOT_TCP   = "GripperDA_v1"

VEL = 60
ACC = 60
FASTER_VEL = 90
FASTER_ACC = 90

DO_CLOSE = 1
DO_OPEN  = 2
DO_WASH  = 3
DO_BRUSH = 4


DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
# =========================================================
# 유틸리티 함수 (그리퍼 및 좌표 계산)
# =========================================================
# =========================================================
# 그리퍼 세팅 및 그외 필요 함수
# =========================================================
def _to_list(posj_val):
    if isinstance(posj_val, (list, tuple)): return list(posj_val)
    return list(posj_val)

def _add_z(p6, dz_mm):
    """posx용 6D 리스트에서 z만 +dz 적용"""
    return [p6[0], p6[1], p6[2] + dz_mm, p6[3], p6[4], p6[5]]

def gripper_open():
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(DO_OPEN, 1); set_digital_output(DO_CLOSE, 0)
    set_digital_output(DO_WASH, 0); set_digital_output(DO_BRUSH, 0)
    wait(0.3)

def gripper_close():
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(DO_OPEN, 0); set_digital_output(DO_CLOSE, 1)
    set_digital_output(DO_WASH, 0); set_digital_output(DO_BRUSH, 0)
    wait(0.3)

def gripper_wash():  # 이름은 wash 인데 그리퍼 집을때 확 여는거
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(DO_WASH, 1); set_digital_output(DO_OPEN, 0); set_digital_output(DO_CLOSE, 0)
    wait(0.3)

def gripper_width_1mm():
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(1, 1); set_digital_output(2, 0); set_digital_output(3, 0); wait(0.8)

def gripper_width_37mm():
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(1, 0); set_digital_output(2, 1); set_digital_output(3, 0); wait(0.8)

def gripper_width_55mm():
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(1, 0); set_digital_output(2, 0); set_digital_output(3, 1); wait(0.8)

def rotate_j6_to(target_deg, jvel=60, jacc=120):
    from DSR_ROBOT2 import get_current_posj, movej, wait
    j = _to_list(get_current_posj())
    j[5] = float(target_deg)
    movej(j, vel=jvel, acc=jacc)
    wait(0.2)


# =========================================================
# 메인 동작 시퀀스 (Scrub Wash)
# =========================================================
def perform_scrub_wash():
    from DSR_ROBOT2 import (
        movel, movejx, movej, posx, wait, move_periodic,
        get_current_posx, get_current_posj, DR_BASE, set_ref_coord,
        task_compliance_ctrl, release_compliance_ctrl, set_desired_force,
        DR_MV_MOD_ABS, DR_MV_MOD_REL, set_singularity_handling, DR_AVOID
    )
    set_ref_coord(0)  # base 기준

    # ---------- ✅ movel 호환 wrapper (mod 인자 미지원 환경 대비) ----------
    def movel_call(p, ref=None, mod=None, radius=0.0):
    # mod/ref가 None이면 아예 인자를 넘기지 않도록 구성함
        kwargs = dict(vel=VEL, acc=ACC, radius=radius)

        if ref is not None:
            kwargs["ref"] = ref
        if mod is not None:
            kwargs["mod"] = mod

        return movel(p, **kwargs)


    def movel_rel(ref, dx=0.0, dy=0.0, dz=0.0, drz=0.0):
        dp = posx(dx, dy, dz, 0.0, 0.0, drz)
        return movel_call(dp, ref=ref, mod=DR_MV_MOD_REL, radius=0.0)

    # [STEP 0] 홈 포지션
    home_x = posx([367.28, 8.93, 422.97, 4.37, 179.98, 3.97])
    movejx(home_x, vel=VEL, acc=ACC)
    wait(0.5)

    
    # =====================================================
    # [STEP 4] 스크럽 세척 (Scrub Wash) ✅ FIXED
    # - plate_release -> release_up(안전) -> brush_up -> brush_down -> grab -> brush_up(안전)
    # - 사용자좌표 상대이동은 movel_rel wrapper로 끊김 방지
    # =====================================================
    set_singularity_handling(DR_AVOID)

    set_ref_coord(DR_BASE)

    plate_release_6d = [649.520, -12.250, 428.380, 151.75, -176.51, -28.70]
    plate_release    = posx(plate_release_6d)
    # 
    brush_up   = posx([585.890, 146.840, 434.800, 163.90, -178.49, 167.17])
    brush_down = posx([591.340, 147.690, 366.960, 164.44, -178.38, 167.61])
    # 중간 경로로 이동 후
    plate_up1_6d = posx([594.610, -11.050, 528.980, 168.26, -171.74, -11.87])
    movel_call(plate_up1_6d , ref=DR_BASE)
    wait(0.2)
    movel_call(plate_release, ref=DR_BASE)
    gripper_width_55mm()
    wait(0.2)

    plate_release_up = posx(_add_z(plate_release_6d, 100.0))
    movel_call(plate_release_up, ref=DR_BASE)
    wait(0.2)

    # 4-2) 브러시 접근은 반드시 UP 경유
    movel_call(brush_up, ref=DR_BASE)
    wait(0.2)
    movel_call(brush_down, ref=DR_BASE)
    wait(0.2)

    # 4-3) 브러시 파지 후 반드시 위로 빼기(안전)
    gripper_width_1mm()
    wait(0.2)

    movel_call(brush_up, ref=DR_BASE)
    wait(0.2)

    # 필요 시 자세 정리(원하면 주석 해제)
    rotate_j6_to(-82.0, jvel=60, jacc=120)

    # 4-4) 지그재그 패턴(예시: 5열)
    REF_USER = 101
    set_ref_coord(REF_USER)

    HOME = posx(367.28, 8.93, 422.97, 4.37, 179.98, 3.97)

    STEP_X = 15.0
    DOWN_Y = 25.0
    REPEAT_N = 6
    def movel_abs(p, ref):
        return movel_call(p, ref=ref, mod=DR_MV_MOD_ABS, radius=0.0)
    
    movel_abs(HOME, ref=DR_BASE)
    PLATELEFT = posx(478.840, 76.220, 314.870, 178.3, -136.46, -178.26)
    movel_abs(PLATELEFT, ref=DR_BASE)

    def col_pattern_x(x_sign, repeat_n=REPEAT_N):
        dx = STEP_X * x_sign
        for _ in range(repeat_n):
            movel_rel(REF_USER, dx=dx, drz=+30.0)
            movel_rel(REF_USER, dx=dx, drz=-30.0)

    # 1~5열
    col_pattern_x(+1); movel_rel(REF_USER, dy=-DOWN_Y)
    col_pattern_x(-1); movel_rel(REF_USER, dy=-DOWN_Y)
    col_pattern_x(+1); movel_rel(REF_USER, dy=-DOWN_Y)
    col_pattern_x(-1); movel_rel(REF_USER, dy=-DOWN_Y)
    col_pattern_x(+1)


    # 4-5) 종료: base 복귀
    set_ref_coord(DR_BASE)
    movel_call(home_x, ref=DR_BASE)


    JReady = [0, 0, 90, 0, 90, 0]

    HOME = posx(367.28, 8.93, 422.97, 4.37, 179.98, 3.97)

    brush_up   = posx(585.890, 146.840, 434.800, 163.90, -178.49, 167.17)
    brush_down = posx(591.340, 147.690, 366.960, 164.44, -178.38, 167.61)

    plate_grab_down = posx(694.94, -13.72, 346.44, 125.86, -177.59, 127.44)
    plate_grab_up   = posx(698.13, -14.26, 360.97, 161.29, -172.72, 162.84)
    plate_pull      = posx(549.59, 2.19, 574.29, 167.00, -164.91, 168.13)
    plate_pull_up   = posx(246.73, 3.25, 526.63, 164.10, -170.55, 169.11)

    # ===== 시퀀스 =====
    print("Move: JReady")
    movej(JReady, vel= VEL, acc=ACC)

    print("Move: HOME")
    movel(HOME, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE)

    print("Move: brush_up")
    movel(brush_up, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE)

    print("Move: brush_down")
    movel(brush_down, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE)

    print("Gripper: open 37mm (DO2 ON)")
    gripper_width_37mm()
    wait(0.2)

    print("Move: brush_up")
    movel(brush_up, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE)

    print("Move: plate_grab_up (approach)")
    movel(plate_grab_up, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE)

    print("Move: plate_grab_down")
    movel(plate_grab_down, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE)

    print("Gripper: close 1mm (DO1 ON)")
    gripper_width_1mm()
    wait(0.5)

    print("Move: plate_grab_up")
    movel(plate_grab_up, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE)
    wait(0.3)

    print("Move: plate_pull")
    movel(plate_pull, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE)

    print("Move: plate_pull_up")
    movel(plate_pull_up, vel= VEL, acc=ACC, radius=0.0, ref=DR_BASE)

    print("Sequence done")



# =========================================================
# ROS2 SERVICE NODE CLASS
# =========================================================
class ScrubWashService(Node):
    def __init__(self):
        super().__init__("scrub_wash_srv", namespace=ROBOT_ID)
        DR_init.__dsr__node = self

        self._lock = threading.Lock()
        self._running = False
        self.callback_group = ReentrantCallbackGroup()

        self._srv = self.create_service(
            Trigger, 
            "scrub_wash", 
            self._on_trigger,
            callback_group=self.callback_group
        )
        self.get_logger().info(f"[Service Ready] /{ROBOT_ID}/scrub_wash")

    def _on_trigger(self, request, response):
        with self._lock:
            if self._running:
                response.success = False
                response.message = "로봇이 이미 스크럽 세척 동작 중입니다."
                return response
            self._running = True
            threading.Thread(target=self._run_job, daemon=True).start()

        response.success = True
        response.message = "스크럽 세척 시퀀스를 시작합니다."
        return response
    
    def _run_job(self):
        try:
            from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS, set_tool, set_tcp

            self.get_logger().info("Scrub Wash 시작!")
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
            set_tool(ROBOT_TOOL)
            set_tcp(ROBOT_TCP)
            perform_scrub_wash()
            self.get_logger().info("Scrub Wash 완료")
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
    node = ScrubWashService()
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