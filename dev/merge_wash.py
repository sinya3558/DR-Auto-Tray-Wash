import rclpy
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

DO_CLOSE = 1
DO_OPEN  = 2
DO_WASH = 3
DO_BRUSH = 4

DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# =========================================================
# 초기화
# =========================================================
def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)


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

def gripper_wash():
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(DO_OPEN, 0)
    set_digital_output(DO_CLOSE, 0)
    set_digital_output(DO_WASH, 1)
    set_digital_output(DO_BRUSH, 0)
    wait(0.3)

## trash_out()
def _to_list(posj_val):
    """get_current_posj() 반환 타입이 list/posj 등일 수 있어 list로 통일함"""
    if isinstance(posj_val, (list, tuple)):
        return list(posj_val)
    try:
        return list(posj_val)
    except Exception:
        raise RuntimeError(f"Unsupported posj type: {type(posj_val)}")
    
## after hot wash >  -- > brush

def _add_z(p6, dz_mm):
    """posx용 6D 리스트에서 z만 +dz 적용함"""
    return [p6[0], p6[1], p6[2] + dz_mm, p6[3], p6[4], p6[5]]


def gripper_width_1mm():
    """그리퍼 폭 1mm(닫기): DO1=1, DO2=0, DO3=0"""
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(1, 1)
    set_digital_output(2, 0)
    set_digital_output(3, 0)
    wait(0.8)


def gripper_width_37mm():
    """그리퍼 폭 37mm: DO1=0, DO2=1, DO3=0"""
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    set_digital_output(3, 0)
    wait(0.8)


def gripper_width_55mm():
    """그리퍼 폭 55mm(열기): DO1=0, DO2=0, DO3=1"""
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(1, 0)
    set_digital_output(2, 0)
    set_digital_output(3, 1)
    wait(0.8)



# ==============
# 1. 식판 집기
# ==============
def pick_tray():
    from DSR_ROBOT2 import (
        movej, movejx, movel, posx,
        task_compliance_ctrl, release_compliance_ctrl,
        set_desired_force, get_current_posx, wait
    )

    # 홈
    home_j = [0, 0, 90, 0, 90, 0]
    home_x = posx([367.28, 8.93, 422.97, 4.37, 179.98, 3.97])
    
    movej(home_j, vel=VEL, acc=ACC)
    movejx(home_x, vel=VEL, acc=ACC)
    wait(0.5)
    print("home position")

    # 식판 상부 접근
    '''
    - 식판 1 : [-349.16, -143.11, 526.74, 133.51, 179.79, -45.27]
    - 식판 2 : [-289.24, -139.26, 544.47, 168.93, 179.38, -9.83]
    '''
    tray_approach = posx([-289.24, -141, 544.47, 168.93, 179.38, -9.83])
    movejx(tray_approach, vel=VEL, acc=ACC, time= 5)
    wait(0.5)
    print("식판 포지션으로 접근")

    gripper_open()

    # 힘 제어 기반 기준면 확보 (x- 방향)
    task_compliance_ctrl()
    set_desired_force([5, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0])

    approach_x = posx([-289.24, -139.26, 544.47, 168.93, 179.38, -9.83])
    movel(approach_x, vel=VEL, acc=VEL)
    print("go to approach x")

    release_compliance_ctrl()

    # 수직 하강 후 파지
    '''
    - 식판 2 down = [-295.8, -146.07, 256.53, , , ]
    -    =oritin = [-293.85, -139.27, 253.17, 167.80, 179.43, -10.95]
    - z 차이 : 291.3
    '''
    cp = get_current_posx()[0]
    print("현재 좌표 get")
    tray_down = posx([cp[0], cp[1], cp[2] - 290, cp[3], cp[4], cp[5]])
    movel(tray_down, vel=VEL, acc=ACC)

    gripper_close()

    # 들어올림
    movel(cp, vel=VEL, acc=ACC)

# ===========
# 1.5 trash
# ===========

def trash_out():
    """요청 시퀀스 수행함"""
    from DSR_ROBOT2 import (
        posx,
        movej,
        movel,
        wait,
        get_current_posj,
    )

    # ====== 포지션 정의(요청 좌표 그대로) ======
    P_PICKING_UP2 = posx(-289.24, -139.26, 544.47, 168.93, 179.38,  -9.83)
    P_TRASH_UP    = posx( -19.01, -374.20, 540.00,  80.62,-174.96,  -7.70)
    P_TRASH_DOWN  = posx( -54.45, -364.25, 252.28,  87.89,-149.56,  -0.96)

    # 안전고도 경유점(다운 접근 안전용) - down의 x,y/rpy + up의 z로 구성함
    P_TRASH_DOWN_SAFE = posx(-54.45, -364.25, 540.00, 87.89, -149.56, -0.96)

    # ====== 시퀀스 시작 ======
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
    j5_center = base_j[4]       # J5 = index 4
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

# ============
# 2. 온수 세척
# ============
def hot_water_wash():
    from DSR_ROBOT2 import movel, posx, move_periodic, wait

    hot_top = posx([297.06, -437.00, 540.00, 40.38, -177.85, -47.41])
    hot_down = posx([291.20, -444.58, 308.34, 35.15, -177.60, -52.77])

    movel(hot_top, vel=FASTER_VEL, acc=FASTER_ACC)
    movel(hot_down, vel=VEL, acc=ACC)

    move_periodic(
        amp=[0, 60, 0, 0, 0, 0],
        period= 2.0,
        atime= 0.5,
        repeat= 3,
        ref=0
    )

    move_periodic(
        amp=[0, 0, 60, 0, 0, 0],
        period=2.0,
        atime=0.5,
        repeat=3,
        ref=0
    )

    

    movel(hot_top, vel=VEL, acc=ACC)
    wait(0.5)



# ========================
# 4. 스크럽 세척 
# ========================

def scrub_wash():
    from DSR_ROBOT2 import (
        posx, movej, movel, wait, set_ref_coord, set_singularity_handling, 
        DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_AVOID
    )

    ## PART.1 어제 받은 코드
    set_ref_coord(0)  # base 기준

    # ===== 좌표(사용자 제공 티칭값) =====
    hot_up_6d       = [297.06, -437.00, 540.00, 40.38, -177.85, -47.41]
    plate_up1_6d    = [594.610, -11.050, 528.980, 168.26, -171.74, -11.87]
    plate_release_6d= [649.520, -12.250, 428.380, 151.75, -176.51, -28.70]
    brush_up_6d     = [585.890, 146.840, 434.800, 163.90, -178.49, 167.17]
    brush_down_6d   = [591.340, 147.690, 366.960, 164.44, -178.38, 167.61]
    # home_6d         = [367.28, 8.93, 422.97, 113.03, -179.91, 110.31]
    # home_1 = posx([367.28, 8.93, 422.97, 113.03, -179.91, 110.31])

    hot_up        = posx(hot_up_6d)
    plate_up1     = posx(plate_up1_6d)
    plate_release = posx(plate_release_6d)
    brush_up      = posx(brush_up_6d)
    brush_down    = posx(brush_down_6d)
    # HOME          = posx(home_6d)
    HOME2 = posx([367.32, 8.840, 422.880, 98.34, -180, 97.9])

    # ===== 시퀀스 =====
    print("Move: hot_up")
    movel(hot_up, vel=VEL, acc=ACC, radius=0.0, ref=0)
    wait(0.2)

    print("Move: plate_up1")
    movel(plate_up1, vel=VEL, acc=ACC, radius=0.0, ref=0)
    wait(0.2)

    print("Move: plate_release")
    movel(plate_release, vel=VEL, acc=ACC, radius=0.0, ref=0)
    wait(0.2)

    print("Gripper: open to 55mm (DO3 ON)")
    gripper_width_55mm()
    wait(0.2)

    print("Move: plate_release + 100mm Z up (10cm)")
    plate_release_up = posx(_add_z(plate_release_6d, 100.0))
    movel(plate_release_up, vel=VEL, acc=ACC, radius=0.0, ref=0)
    wait(0.2)

    print("Move: brush_up")
    movel(brush_up, vel=VEL, acc=ACC, radius=0.0, ref=0)
    wait(0.2)

    print("Move: brush_down")
    movel(brush_down, vel=VEL, acc=ACC, radius=0.0, ref=0)
    wait(0.2)

    print("Gripper: close to 1mm (DO1 ON)")
    gripper_width_1mm()
    wait(0.2)

    print("Move: brush_up")
    movel(brush_up, vel=VEL, acc=ACC, radius=0.0, ref=0)
    wait(0.2)

    brush_up_6d_j6m82 = [585.890, 146.840, 434.800, 163.90, -178.49, -82.00]
    print("Rotate: brush_up (J6 -> -82)")
    movel(brush_up_6d_j6m82, vel=VEL, acc=ACC, radius=0.0, ref=0)
    wait(0.2)

    print("Move: HOME2")
    movel(HOME2, vel=VEL, acc=ACC, radius=0.0, ref=0)
    wait(0.2)

    print("Task done")

    ## PART.2. BRUSH

    REF_BASE = DR_BASE
    REF_USER = 101  # User Coordinates2

    HOME = posx(367.28, 8.93, 422.97, 4.37, 179.98, 3.97)
    

    PLATELEFT = posx(478.840, 76.220, 314.870, 178.3, -136.46, -178.26)

    SAFE_Z_UP_MM = 50.0
    DOWN_Y_MM    = 25.0   # -Y 2cm
    STEP_X_MM    = 15.0   # 1.5cm
    REPEAT_N     = 6
    AFTER_Y_MM   = 50.0  

    def movel_call(p, ref=None, mod=None, radius=0.0):
        try:
            return movel(p, vel=VEL, acc=ACC, radius=radius, ref=ref, mod=mod)
        except TypeError:
            pass
        try:
            return movel(p, vel=VEL, acc=ACC, radius=radius, ref=ref)
        except TypeError:
            pass
        return movel(p, vel=VEL, acc=ACC)

    def movel_abs(p, ref):
        return movel_call(p, ref=ref, mod=DR_MV_MOD_ABS, radius=0.0)

    def movel_rel(ref, dx_mm=0.0, dy_mm=0.0, dz_mm=0.0, drz_deg=0.0):
        dp = posx(dx_mm, dy_mm, dz_mm, 0.0, 0.0, drz_deg)
        return movel_call(dp, ref=ref, mod=DR_MV_MOD_REL, radius=0.0)

    def col_pattern_x(x_sign, repeat_n=REPEAT_N):
        """한 열 패턴 수행함 (+X 또는 -X)"""
        dx = STEP_X_MM * x_sign
        for _ in range(repeat_n):
            movel_rel(REF_USER, dx_mm=dx, drz_deg=+30.0)
            movel_rel(REF_USER, dx_mm=dx, drz_deg=-30.0)

    def down_y():
        """열간 -Y 이동 수행함"""
        movel_rel(REF_USER, dy_mm=-DOWN_Y_MM)

    # 1) HOME -> PLATELEFT (base abs)
    set_ref_coord(REF_BASE)
    movel_abs(HOME, ref=REF_BASE)
    movel_abs(PLATELEFT, ref=REF_BASE)

    # 2) 사용자좌표계 패턴
    set_ref_coord(REF_USER)

    # 1열(+X) 후 -Y
    col_pattern_x(+1)
    down_y()

    # 2열(-X) 후 -Y
    col_pattern_x(-1)
    down_y()

    # 3열(+X) 후 -Y
    col_pattern_x(+1)
    down_y()

    # 4열(-X) 후 -Y  ✅ 여기서 반드시 내려가고
    col_pattern_x(-1)
    down_y()

    # 5열(+X) = 1열과 동일 ✅ 반드시 수행됨
    col_pattern_x(+1)
    movel_rel(REF_USER, dy_mm=+AFTER_Y_MM)

    # 3) 안전상승 후 HOME 복귀 (base)
    set_ref_coord(REF_BASE)
    movel_rel(REF_BASE, dz_mm=SAFE_Z_UP_MM)
    movel_abs(HOME, ref=REF_BASE)


    ## PART.2 오늘받은 코드
    REF_BASE = 0
    set_ref_coord(REF_BASE)
    set_singularity_handling(DR_AVOID)

    # ===== 포즈 정의 =====
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
    movel(HOME, vel= VEL, acc=ACC, radius=0.0, ref=REF_BASE)

    print("Move: brush_up")
    movel(brush_up, vel= VEL, acc=ACC, radius=0.0, ref=REF_BASE)

    print("Move: brush_down")
    movel(brush_down, vel= VEL, acc=ACC, radius=0.0, ref=REF_BASE)

    print("Gripper: open 37mm (DO2 ON)")
    gripper_width_37mm()
    wait(0.2)

    print("Move: brush_up")
    movel(brush_up, vel= VEL, acc=ACC, radius=0.0, ref=REF_BASE)

    print("Move: plate_grab_up (approach)")
    movel(plate_grab_up, vel= VEL, acc=ACC, radius=0.0, ref=REF_BASE)

    print("Move: plate_grab_down")
    movel(plate_grab_down, vel= VEL, acc=ACC, radius=0.0, ref=REF_BASE)

    print("Gripper: close 1mm (DO1 ON)")
    gripper_width_1mm()
    wait(0.5)

    print("Move: plate_grab_up")
    movel(plate_grab_up, vel= VEL, acc=ACC, radius=0.0, ref=REF_BASE)
    wait(0.3)

    print("Move: plate_pull")
    movel(plate_pull, vel= VEL, acc=ACC, radius=0.0, ref=REF_BASE)

    print("Move: plate_pull_up")
    movel(plate_pull_up, vel= VEL, acc=ACC, radius=0.0, ref=REF_BASE)

    print("Sequence done")



# =====================
# 5, 6. 물 세척 + 적재
# =====================


def cold_wash():
    from DSR_ROBOT2 import movel, movejx, movej, posx, wait, move_periodic, get_current_posx, release_compliance_ctrl, DR_BASE
    # 0. HOME 포지션에서 시작 
    # cold_j = [0, 0, 90, 0, 90, 90]
    home_x = posx([367.28, 8.93, 422.97, 4.37, 179.98, 3.97])
    
    movejx(home_x, vel=VEL, acc=ACC)
    wait(0.5)
    print("home position")

    # 1. 위치 정의
    water_top = posx([275.22, 526.54, 458.35, 1.11, -177.15, -177.39])
    # water_down = posx([262.86, 526.12, 176.83, 5.17, -177.20, -173.22])

    # # 2. 세척 위치로 진입
    # print("[STEP 5] Move to cold washing position")
    movel(water_top, vel=VEL, acc=ACC)
    cp = get_current_posx()[0]
    print("현재 좌표 get(STEP 5)")
    change_dir= posx([cp[0], cp[1], cp[2], cp[3], cp[4], cp[5] + 90])
    movel(change_dir, vel=VEL, acc=ACC)
    wait(0.5)

    # ------- current position 사용해보기
    cold_soak = posx([cp[0], cp[1], cp[2] - 281, cp[3], cp[4], cp[5]+90])

    # 내려
    movel(cold_soak, vel=VEL, acc=ACC)
    wait(0.5)

    move_periodic(amp= [60, 60, 0, 0, 0, 0], period=[3.2, 1.6, 0, 0, 0, 0], atime= 3.1, repeat=4, ref= DR_BASE)
    wait(0.5)

    # 들어올려
    movel(cp, vel=VEL, acc=ACC)
    wait(0.5)

    # movel(home_x, vel=VEL, acc=ACC)
    # wait(0.5)

    
def stack():
    from DSR_ROBOT2 import movel, posx, wait, get_current_posx, DR_BASE
    on_my_way_mid = posx([-74.630, 429.650, 474.090, 175.36, 177.4, -93.9])
    tray_up2 = posx([-269.990, 132.5, 540.540, 1.5,-178.04, 2.64])
    home_x = posx([367.28, 8.93, 422.97, 4.37, 179.98, 3.97])
    tray_down2 = posx([-280.75, 133.40, 260.380, 8.29, -178.4, 9.34])

    movel(on_my_way_mid, vel= VEL, acc= ACC)
    wait(0.5)
    movel(tray_up2, vel=VEL, acc= ACC)
    wait(0.5)
    print("Move back to stack position")

    # cp = get_current_posx()[0]
    # print("현재 좌표 get")
    # tray_down = posx([cp[0]+10, cp[1], cp[2] - 275, cp[3], cp[4], cp[5]])
    movel(tray_down2, vel=VEL, acc=ACC)
    wait(0.5)

    gripper_wash()
    wait(0.5)

    # 들어올림
    movel(tray_up2, vel=VEL, acc=ACC)
    wait(0.5)
    movel(on_my_way_mid, vel= VEL, acc= ACC)
    movel(home_x, vel=VEL, acc=ACC)
    print("마지막이야, 집 또 가")
    wait(0.5)

# ==========
# 메인 루프
# ==========
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("project_tray_wash", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        pick_tray()
        trash_out()
        hot_water_wash()
        # put_tray()
        scrub_wash()
        cold_wash()
        stack()

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
