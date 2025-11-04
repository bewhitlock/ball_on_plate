import cv2 as cv
import numpy as np

# --- Camera calibration ---
#mtx = np.array([[1231.53955, 0., 703.463229],
 #[0., 1235.03337, 426.147508],
 #[0., 0., 1.]])
#dist = np.array([[ 0.02869969, -0.24915749, -0.01115483, -0.00071546,  0.23742651]])

# --- Start camera ---
cap = cv.VideoCapture(1)
if not cap.isOpened():
    print("Camera not found!")
    exit()

# --- Undistortion map using only original mtx ---
ret, frame = cap.read()
#h, w = frame.shape[:2]
#mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, mtx, (w, h), cv.CV_32FC1)

# --- Color range (green) ---
lower = np.array([40, 130, 60]) #for ball: lower = np.array([40, 150, 60])
upper = np.array([100, 230, 255]) #for ball: upper = np.array([100, 255, 255])

while True:
    _, frame = cap.read()

    # Undistort and convert to hsv
    #frame = cv.remap(frame, mapx, mapy, cv.INTER_LINEAR)
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    x, y = -1, -1
    if contours:
        c = max(contours, key=cv.contourArea)
        m = cv.moments(c)
        if m["m00"] != 0:
            x = int(m["m10"] / m["m00"])
            y = int(m["m01"] / m["m00"])
            cv.circle(frame, (x, y), 5, (0, 0, 255), -1)

    print(f"({x},{y})")

    #cv.imshow("frame", frame)
    #cv.imshow("mask", mask)
    if cv.waitKey(1) == 27:  # ESC to quit
        break

cap.release()
#cv.destroyAllWindows()