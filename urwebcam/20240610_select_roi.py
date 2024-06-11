import cv2
import numpy as np

isDragging = False
x0, y0, w, h = -1, -1, -1, -1
blue, red = (255, 0, 0), (0, 0, 255)

# 보정 매트릭스와 왜곡 계수 (예시 값 사용, 실제 값으로 대체 필요)
mtx = np.array([[3.35363409e+03, 0.00000000e+00, 3.20398039e+02],
 [0.00000000e+00, 3.27143741e+03, 2.40404283e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist = np.array([[ 4.97362943e-01, -6.05541137e+01, -6.56743039e-03,  4.39409986e-03,
  -2.77528622e-01]])

def onMouse(event, x, y, flags, param):
    global isDragging, x0, y0, img
    if event == cv2.EVENT_LBUTTONDOWN:
        isDragging = True
        x0 = x
        y0 = y
    elif event == cv2.EVENT_MOUSEMOVE:
        if isDragging:
            img_draw = img.copy()
            cv2.rectangle(img_draw, (x0, y0), (x, y), blue, 2)
            cv2.imshow('img', img_draw)
    elif event == cv2.EVENT_LBUTTONUP:
        if isDragging:
            isDragging = False
            w = x - x0
            h = y - y0
            if w > 0 and h > 0:
                img_draw = img.copy()
                cv2.rectangle(img_draw, (x0, y0), (x, y), red, 2)
                cv2.imshow('img', img_draw)
                roi = img[y0:y0+h, x0:x0+w]
                cv2.imshow('cropped', roi)
                cv2.moveWindow('cropped', 0, 0)
                cv2.imwrite('./cropped.png', roi)
                print(f'x: {x0}, y: {y0}, w: {w}, h: {h}')  # ROI 영역의 좌표와 크기 출력
            else:
                cv2.imshow('img', img)
                print('drag should start from left-top side')

cap = cv2.VideoCapture(1)  # 웹캠을 사용

if not cap.isOpened():
    print("Error: Could not open video device")
    exit()

cv2.namedWindow('img')
cv2.setMouseCallback('img', onMouse)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image")
        break

    # 보정 적용
    h, w = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)

    # 보정된 이미지로 표시
    img = dst.copy()
    cv2.imshow('img', img)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC 키를 누르면 종료
        break

cap.release()
cv2.destroyAllWindows()
