import cv2
import numpy as np
import glob

# Calibration pattern settings
CHECKERBOARD = (10, 7)  # inner corners
SQUARE_SIZE = 0.020  # in meters (or whatever real unit)

# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (0,0,0), (1,0,0) ...
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints = []

def calculate_calibration(pattern, show_img=True):
    images = glob.glob(pattern)

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            if show_img:
                cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
                cv2.imshow("Corners", img)
                cv2.waitKey(100)

    cv2.destroyAllWindows()

    # Calibrate camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    return mtx, dist

subsets = {
    "RGB": "calib_img_rgb_*.png",
    "Left": "calib_img_left_*.png",
    "Right": "calib_img_right_*.png"
}

for label, pattern in subsets.items():
    print(f"\nCalibrating {label} camera using pattern: {pattern}")
    mtx, dist = calculate_calibration(pattern, show_img=True)
    if mtx is not None:
        print("Camera Matrix:\n", mtx)
        print("Distortion Coefficients:\n", dist)
        print(f"""
            fx = {mtx[0][0]}
            fy = {mtx[1][1]}
            cx = {mtx[0][2]}
            cy = {mtx[1][2]}     
        """)
    else:
        print(f"Calibration failed for {label} camera.")