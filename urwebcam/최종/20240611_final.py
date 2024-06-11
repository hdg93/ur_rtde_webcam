import cv2
import numpy as np
import time
import threading
import rtde_control
import rtde_receive
import onRobot.gripper as gripper

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
close_width = 25.66
open_width = 55.2
#ur포스 세팅
task_frame = [0, 0, 0, 0, 0, 0]
selection_vector = [0, 0, 1, 0, 0, 0]
wrench_down = [0, 0, -10, 0, 0, 0]
wrench_up = [0, 0, 10, 0, 0, 0]
force_type = 2
limits = [2, 2, 1.5, 1, 1, 1]
dt = 1.0/500  # 2ms
# 색상 범위 설정
color_ranges = {
    'red': ([0, 120, 70], [10, 255, 255]),
    'yellow': ([20, 100, 100], [40, 255, 255]),
    'green': ([40, 40, 40], [80, 255, 255]),
    'blue': ([100, 150, 0], [140, 255, 255])
}

color_codes = {'red': 1, 'yellow': 2, 'green': 3, 'blue': 4}
min_size = 20  # 상자의 최소 크기 (너비와 높이의 최솟값)

# 카메라 보정
mtx = np.array([[3.35363409e+03, 0.00000000e+00, 3.20398039e+02],
                [0.00000000e+00, 3.27143741e+03, 2.40404283e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist = np.array([[4.97362943e-01, -6.05541137e+01, -6.56743039e-03, 4.39409986e-03,
                  -2.77528622e-01]])


def detect_boxes(hsv, color_ranges, min_size):
    boxes = {}
    for color, (lower, upper) in color_ranges.items():
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes[color] = []
        for contour in contours:
            rect = cv2.minAreaRect(contour)
            width, height = rect[1]
            if width >= min_size and height >= min_size:
                box = cv2.boxPoints(rect)
                box = np.intp(box)  # np.int0를 np.intp로 변경
                boxes[color].append((rect, box))
    return boxes


def sort_boxes(boxes):
    # 좌측 하단부터 우측 상단으로 정렬
    sorted_boxes = {color: sorted(box_list, key=lambda b: (b[0][0][1], b[0][0][0])) for color, box_list in boxes.items()}
    return sorted_boxes


def mark_boxes(image, sorted_boxes, roi_offset_x, roi_offset_y):
    labels = {}
    centers = []
    for color, boxes in sorted_boxes.items():
        color_code = color_codes[color]
        for i, (rect, box) in enumerate(boxes):
            label = f"{color_code}" if i == 0 else f"{color_code}_{i}"
            center = (int(rect[0][0]) , int(rect[0][1]) )  #roi 이미지 좌표로 변환
            centers.append((color, center, rect[2]))  # 색상, 중심 좌표 및 각도 추가
            labels[center] = label
            cv2.drawContours(image, [box + [roi_offset_x, roi_offset_y]], 0, (0, 255, 0), 2)
            x, y = rect[0]
            cv2.putText(image, label, (int(x) + roi_offset_x, int(y) + roi_offset_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return labels, centers


# 웹캠 초기화
cap = cv2.VideoCapture(1)  # 카메라 장치 번호 변경 가능
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

start_time = time.time()
detected_info = None
centers = []
selected_color = None
exit_flag = False

def input_thread():
    global selected_color, exit_flag
     #초기위치이동
    init_waypoint = [0.02433,-0.31972,0.07402, 0.21408961102624507, -3.118485667801178, 0.0024653322433895682]
    rtde_c.moveL(init_waypoint, 0.2, 0.2, False)
    time.sleep(0.8)
#그리퍼 벌리기
    if rg_width != open_width:
        rg_gripper.rg_grip(open_width, target_force)
        time.sleep(0.8)
    while True:
        user_input = input("Enter color code (1: red, 2: yellow, 3: green, 4: blue) or 'q' to quit: ")
        if user_input == 'q':
            exit_flag = True
            # 스크립트 정지
            rtde_c.stopScript()
            break
        elif user_input in ['1', '2', '3', '4']:
            color_dict = {'1': 'red', '2': 'yellow', '3': 'green', '4': 'blue'}
            selected_color = color_dict[user_input]        
#roi영역이동
            roi_h_waypoint = [-0.21202,-0.16066, 0.07402559377458653, 0.21408961102624507, -3.118485667801178, 0.0024653322433895682]
            rtde_c.moveL(roi_h_waypoint, 0.2, 0.2, False)
            time.sleep(0.8)
#목적지위

            purpose_h_waypoint = [-Y * 8.793675889328063e-4-0.21202, -0.000868708 * X-0.16066, 0.07402559377458653, 0.21408961102624507, -3.118485667801178, 0.0024653322433895682]
            rtde_c.moveL(purpose_h_waypoint, 0.2, 0.2, False) #목적지위로가기
            time.sleep(0.8)

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
                # print(actual_TCP_force)
                if abs(actual_TCP_force[2]) > max_force_threshold:  # Z축 힘이 임계값을 초과하면 멈춤
                    break

                rtde_c.waitPeriod(rtde_c.initPeriod())  # 주기 대기

            rtde_c.forceModeStop()  # 힘 모드 정지
            time.sleep(0.8)
            # 물체를 놓기 위해 Z축을 약간 올리기
            lift_position = rtde_r.getActualTCPPose()
            lift_position[2] += 0.001  
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
            
    else:
        print("Invalid input.")

# Start the input thread
threading.Thread(target=input_thread, daemon=True).start()
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image")
        break

    # 카메라 보정 적용
    h, w = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)

    # ROI 설정
    roi_x, roi_y, roi_w, roi_h = 252, 78, 209, 253  # ROI 좌표 및 크기
    roi = frame[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]
    
    # HSV 변환 및 색상별 상자 감지
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    detected_boxes = detect_boxes(hsv_roi, color_ranges, min_size)

    # 각 상자 정렬 및 번호 매기기
    sorted_boxes = sort_boxes(detected_boxes)
    labels, centers = mark_boxes(frame, sorted_boxes, roi_x, roi_y)
   
    # 그레이스케일 및 블러 처리
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # 이진화 처리
    blockSize = 11
    binary = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY, blockSize, 2)

    # 이진화된 이미지에서 상자 검출 및 좌표, 각도 계산
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        rect = cv2.minAreaRect(contour)
        width, height = rect[1]
        if width >= min_size and height >= min_size:
            box = cv2.boxPoints(rect)
            box = np.intp(box)  # np.int0를 np.intp로 변경
            center = rect[0]
            angle = rect[2]
            label = labels.get((int(center[0]) , int(center[1]) ))
            # 이진화된 이미지 기준의 좌표를 표시
            cv2.drawContours(binary, [box], 0, (0, 0, 255), 2)
            cv2.putText(binary, label, (int(center[0]), int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # ROI에 녹색 사각형 그리기
    cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_w, roi_y + roi_h), (0, 255, 0), 2)

    # 결과 프레임 표시
    cv2.imshow('Detected Boxes', frame)
    cv2.imshow('Binary', binary)

    if selected_color:
        for color, center, angle in centers:
            if color == selected_color:
                X, Y = center
                Z = angle
                print(f"{selected_color.capitalize()} box - X: {X}, Y: {Y}, Angle (Z): {Z}")
                selected_color = None
                break

    if exit_flag:
        break

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()