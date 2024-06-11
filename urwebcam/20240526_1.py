import cv2
import numpy as np
import time

# 색상 범위 설정
color_ranges = {
    'red': ([0, 120, 70], [10, 255, 255]),
    'yellow': ([20, 100, 100], [40, 255, 255]),
    'green': ([40, 40, 40], [80, 255, 255]),
    'blue': ([100, 150, 0], [140, 255, 255])
}

color_codes = {'red': 1, 'yellow': 2, 'green': 3, 'blue': 4}
min_size = 20  # 상자의 최소 크기 (너비와 높이의 최솟값)
#카메라 보정
mtx = np.array([[3.35363409e+03, 0.00000000e+00, 3.20398039e+02],
 [0.00000000e+00, 3.27143741e+03, 2.40404283e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist = np.array([[ 4.97362943e-01, -6.05541137e+01, -6.56743039e-03,  4.39409986e-03,
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
            center = (int(rect[0][0]) + roi_offset_x, int(rect[0][1]) + roi_offset_y)  # 전체 이미지 좌표로 변환
            centers.append((color, center, rect[2]))  # 색상, 중심 좌표 및 각도 추가
            labels[center] = label
            cv2.drawContours(image, [box + [roi_offset_x, roi_offset_y]], 0, (0, 255, 0), 2)
            x, y = rect[0]
            cv2.putText(image, label, (int(x) + roi_offset_x, int(y) + roi_offset_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return labels, centers

def save_detected_info(detected_boxes, labels, filename, roi_offset_x, roi_offset_y):
    with open(filename, 'w') as f:
        for color, boxes in detected_boxes.items():
            for rect, box in boxes:
                center = rect[0]
                angle = rect[2]
                center_x = center[0] + roi_offset_x
                center_y = center[1] + roi_offset_y
                label = labels.get((int(center[0]) + roi_offset_x, int(center[1]) + roi_offset_y), "else")
                f.write(f"{label} - Center X: {center_x}, Center Y: {center_y}, Angle: {angle}\n")

# 웹캠 초기화
cap = cv2.VideoCapture(1)  # 카메라 장치 번호 변경 가능
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

start_time = time.time()
detected_info = None
centers = []

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
    roi = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]

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
            cv2.drawContours(frame, [box + [roi_x, roi_y]], 0, (0, 0, 255), 2)
            cv2.putText(frame, label, (int(center[0]) + roi_x, int(center[1]) + roi_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # ROI에 녹색 사각형 그리기
    cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_w, roi_y + roi_h), (0, 255, 0), 2)

    # 결과 프레임 표시
    cv2.imshow('Detected Boxes', frame)
    cv2.imshow('Binary', binary)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # 사용자 입력 받기
    color_input = cv2.waitKey(1) & 0xFF
    if color_input == ord('q'):
        break
    elif color_input in [ord('1'), ord('2'), ord('3'), ord('4')]:
        color_dict = {ord('1'): 'red', ord('2'): 'yellow', ord('3'): 'green', ord('4'): 'blue'}
        selected_color = color_dict.get(color_input)

        if selected_color:
            for color, center, angle in centers:
                if color == selected_color:
                    x, y = center
                    z = angle
                    print(f"{selected_color.capitalize()} box - X: {x}, Y: {y}, Angle (Z): {z}")
                    break
        else:
            print("Invalid input.")
cap.release()
cv2.destroyAllWindows()

