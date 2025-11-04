import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Chessboard parameters
pattern_size = (6, 9)  # inner corners
square_size = 18.9  # mm

# Prepare object points
objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []
imgpoints = []

# Load images
images = glob.glob('*.jpg')

for fname in images:
    img = cv.imread(fname)
    if img is None:
        print(f"Failed to load {fname}")
        continue

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray, pattern_size, cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE)

    if ret:
        print(f"Corners found in {fname}")
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        cv.drawChessboardCorners(img, pattern_size, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)
    else:
        print(f"No corners found in {fname}")

cv.destroyAllWindows()

# Calibration
if len(objpoints) > 0:
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print("Calibration successful!" if ret else "Calibration failed.")
    print("\nCamera matrix:\n", mtx)
    print("\nDistortion coefficients:\n", dist)

    # Optional: undistort one example image
    img = cv.imread(images[0])
    h, w = img.shape[:2]
    new_camera_mtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    dst = cv.undistort(img, mtx, dist, None, new_camera_mtx)
    cv.imshow('Undistorted', dst)
    cv.waitKey(0)
    cv.destroyAllWindows()
else:
    print("No corners detected. Calibration cannot be performed.")
