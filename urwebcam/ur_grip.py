import onRobot.gripper as gripper
import rtde_control
import rtde_receive
import time

ROBOT_HOST = '192.168.213.74'
ROBOT_PORT = 30004
# Default id is zero, if you have multiple grippers, 
# see logs in UR Teach Pendant to know which is which :)
# RTDE 인터페이스 초기화
rtde_c = rtde_control.RTDEControlInterface(ROBOT_HOST)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_HOST)

# 초기 관절 위치 저장
init_q = rtde_r.getActualQ()
tcp_pose = rtde_r.getActualTCPPose()
print(init_q)
print(tcp_pose)
#rtde_c.moveL([-0.35364,-0.1634,0.0956,1.29,2.864,0.0024653322433895682], 0.1, 0.1, True)

rg_id = 0
rg_gripper = gripper.RG2(rg_id)

rg_width = rg_gripper.get_rg_width()


# force and width units described in onRobot RG2 Manual
target_width = 27.66
target_force = 40.00

open_width = 55.2
target_force = 40.00
if rg_width != target_width:
    rg_gripper.rg_grip(target_width, target_force)
    time.sleep(300)
