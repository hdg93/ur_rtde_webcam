import rtde_control
import rtde_receive
import time

ROBOT_HOST = '192.168.213.74'
ROBOT_PORT = 30004

# RTDE 인터페이스 초기화
rtde_c = rtde_control.RTDEControlInterface(ROBOT_HOST)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_HOST)

task_frame = [0, 0, 0, 0, 0, 0]
selection_vector = [0, 0, 1, 0, 0, 0]
wrench_down = [0, 0, -10, 0, 0, 0]
wrench_up = [0, 0, 10, 0, 0, 0]
force_type = 2
limits = [2, 2, 1.5, 1, 1, 1]
dt = 1.0/500  # 2ms
# 웨이포인트
init_q = rtde_r.getActualQ()
# print(init_q)
stack_point=[-0.3193, -0.0423, 0.11505, 0.21408961102624507, -3.118485667801178, 0.0024653322433895682]
rtde_c.moveL(stack_point, 0.2, 0.2, False)
time.sleep(1.0)


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
lift_position[2] += 0.01  # Z축으로 10cm 올림
rtde_c.moveL(lift_position)
time.sleep(2)
# 스크립트 정지
rtde_c.stopScript()

#관절값
# init_waypoint = [-1.0605781714068812, -1.5472376619330426, -1.683311104774475, -1.4865282487920304, 1.555149793624878, 0.375861257314682]
# #tcp값
# roi_h_waypoint =[-0.23047311621166153, -0.31973025227049445, 0.07402559377458653, 0.21408961102624507, -3.118485667801178, 0.0024653322433895682]
# purpose_h_waypoint =[-y*0.12,-x*0.13,0.07402559377458653, 0.21408961102624507, -3.118485667801178, 0.0024653322433895682]
# # 현재 TCP 위치 가져오기
# tcp_pose = rtde_r.getActualTCPPose()
# print(tcp_pose) 
# # # Move to a specific Cartesian coordinate (e.g., [0.13, 0.2, 0.3, 0.1, 0.2, 0.0])
# specific_target = [-0.12364, -0.20897, 0.0256, 1.29, 2.864, 0.0]
# ###동작###
# # Move asynchronously in Cartesian space to specific_target, we specify asynchronous behavior by setting the async parameter to 'True'.
# rtde_c.moveL(specific_target, 0.1, 0.1, True)  # Adjust velocity and acceleration as needed

# # Wait sufficient time for the robot to move to the target position
# time.sleep(5.0)  # Increase sleep time to allow the robot to move to the target position

# # Optionally stop the movement if you need to interrupt it before the specified time
# # rtde_c.stopL(0.5)

# # Move back to initial joint configuration
# rtde_c.moveJ(init_q)

# # Wait sufficient time for the robot to return to the initial position
# time.sleep(5.0)  # Increase sleep time to allow the robot to return to the initial position

# # Stop the RTDE control script
# rtde_c.stopScript()
