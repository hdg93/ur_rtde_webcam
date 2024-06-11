import cv2
import numpy as np
import threading
import rtde_control
import rtde_receive
import onRobot.gripper as gripper
import time

global centers
ROBOT_HOST = '192.168.213.74'
ROBOT_PORT = 30004

    # RTDE 인터페이스 초기화
rtde_c = rtde_control.RTDEControlInterface(ROBOT_HOST)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_HOST)
# 그리퍼 세팅
rg_id = 0
rg_gripper = gripper.RG2(rg_id)

rg_width = rg_gripper.get_rg_width()
target_force = 40.00
close_width = 27.66
open_width = 55.2
#포스세팅
task_frame = [0, 0, 0, 0, 0, 0]
selection_vector = [0, 0, 1, 0, 0, 0]
wrench_down = [0, 0, -10, 0, 0, 0]
wrench_up = [0, 0, 10, 0, 0, 0]
force_type = 2
limits = [2, 2, 1.5, 1, 1, 1]
dt = 1.0/500  # 2ms
# 관절값과 TCP 값을 설정합니다
init_q = rtde_r.getActualQ()
#초기위치이동
init_waypoint = [0.02433,-0.31972,0.07402, 0.21408961102624507, -3.118485667801178, 0.0024653322433895682]
rtde_c.moveL(init_waypoint, 0.2, 0.2, False)
time.sleep(0.8)
#그리퍼 벌리기
if rg_width != open_width:
    rg_gripper.rg_grip(open_width, target_force)
    time.sleep(0.8)
   
#roi영역이동
roi_h_waypoint = [-0.21202,-0.16066, 0.07402559377458653, 0.21408961102624507, -3.118485667801178, 0.0024653322433895682]
rtde_c.moveL(roi_h_waypoint, 0.2, 0.2, False)
time.sleep(0.8)
#목적지위
Y=103
X=134
purpose_h_waypoint = [-Y * 8.793675889328063e-4-0.21202, -0.000868708 * X-0.16066, 0.07402559377458653, 0.21408961102624507, -3.118485667801178, 0.0024653322433895682]
rtde_c.moveL(purpose_h_waypoint, 0.2, 0.2, False) #목적지위로가기
time.sleep(0.8)
# print(rtde_r.getActualQ())
Z=74
tcp0 = rtde_r.getActualQ() #movej로 회전을 위함
tcp0[5] = tcp0[5] - (90-Z)/100
rtde_c.moveJ(tcp0, 0.2, 0.2, False) #tcp회전
time.sleep(0.8)
purpose_d_waypoint = rtde_r.getActualTCPPose()
purpose_d_waypoint[2] = purpose_d_waypoint[2] -0.05525
rtde_c.moveL(purpose_d_waypoint, 0.2, 0.2, False) #내려가기
#그리퍼 잡기
if rg_width != close_width:
    rg_gripper.rg_grip(close_width, target_force)
    time.sleep(0.8)
#위로 올라가기
purpose_h_waypoint = [-Y * 8.793675889328063e-4-0.21202, -0.000868708 * X-0.16066, 0.07402559377458653, 0.21408961102624507, -3.118485667801178, 0.0024653322433895682]
rtde_c.moveL(purpose_h_waypoint, 0.2, 0.2, False) #목적지위로가기
time.sleep(0.8)
#쌓을곳
stack_point=[-0.3193, -0.0423, 0.11505, 0.21408961102624507, -3.118485667801178, 0.0024653322433895682]
rtde_c.moveL(stack_point, 0.2, 0.2, False)
time.sleep(1.0)
#포스로 쌓기
rtde_c.forceMode(task_frame, selection_vector, wrench_down, force_type, limits)
# 실시간으로 힘을 모니터링하며 동작
start_time = time.time()
max_force_threshold = 10  # 힘 임계값

while True:
    
    actual_TCP_force = rtde_r.getActualTCPForce()  # 현재 TCP 힘을 읽어옴
    print(actual_TCP_force)
    if abs(actual_TCP_force[2]) > max_force_threshold:  # Z축 힘이 임계값을 초과하면 멈춤
        break

    rtde_c.waitPeriod(rtde_c.initPeriod())  # 주기 대기

rtde_c.forceModeStop()  # 힘 모드 정지
time.sleep(0.8)
# 물체를 놓기 위해 Z축을 약간 올리기
lift_position = rtde_r.getActualTCPPose()
lift_position[2] += 0.007  
rtde_c.moveL(lift_position)
time.sleep(0.8)
#그리퍼 벌리기
if rg_width != open_width:
    rg_gripper.rg_grip(open_width, target_force)
    time.sleep(0.8)
# 벗어 위해 Z축을 약간 올리기
lift_point= rtde_r.getActualTCPPose()
lift_point[2] += 0.05  
rtde_c.moveL(lift_point)
time.sleep(0.8)    
# 스크립트 정지
rtde_c.stopScript()

