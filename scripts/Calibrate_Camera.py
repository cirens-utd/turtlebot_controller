Great — calibrating your **OAK-D camera** (from Luxonis) on a **TurtleBot** is essential for tasks like pose estimation, SLAM, or accurate tag localization. The process uses a **checkerboard** and **OpenCV’s calibration tools**.

Here's how to do it step by step.

---

## 🎯 Goal

Get the **camera intrinsics**:
`fx`, `fy`, `cx`, `cy`
→ Needed for `camera_params = [fx, fy, cx, cy]` in AprilTag detection.

---

## 🧰 What You’ll Need

* Your **OAK-D camera** connected to the TurtleBot (or another host)
* A **checkerboard pattern** (e.g., 7×6 squares of known size)
* `depthai` SDK installed
* Python and OpenCV (`cv2`)
* Optionally: ROS (if you're capturing via ROS topics)

---

## ✅ Step-by-Step Calibration (Standalone via Python)

### 1. 📥 Print a Checkerboard

* Use a **7×6 or 9×6** grid (inner corners)
* Squares should be **accurately sized** (e.g., 25mm x 25mm)
* Fix it flat on a rigid surface (e.g., foam board)

---

### 2. 🐍 Capture Calibration Images with the OAK-D

Here's a simple script using `depthai` and OpenCV:

```python
import cv2
import depthai as dai

pipeline = dai.Pipeline()
camRgb = pipeline.create(dai.node.ColorCamera)
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("video")
camRgb.video.link(xout.input)

camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)

with dai.Device(pipeline) as device:
    video = device.getOutputQueue("video", maxSize=8, blocking=False)

    img_count = 0
    while True:
        inFrame = video.get()
        frame = inFrame.getCvFrame()
        cv2.imshow("Calib", frame)

        key = cv2.waitKey(1)
        if key == ord("c"):
            cv2.imwrite(f"calib_img_{img_count}.png", frame)
            print(f"Captured calib_img_{img_count}.png")
            img_count += 1
        elif key == 27:  # ESC
            break

cv2.destroyAllWindows()
```

* Press **C** to capture a calibration image
* Capture **15–20 images** from different angles and distances

---

### 3. 🎯 Calibrate Using OpenCV

You can use OpenCV’s calibration tools on the saved images:

```python
import cv2
import numpy as np
import glob

# Calibration pattern settings
CHECKERBOARD = (7, 6)  # inner corners
SQUARE_SIZE = 0.025  # in meters (or whatever real unit)

# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (0,0,0), (1,0,0) ...
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints = []

images = glob.glob("calib_img_*.png")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow("Corners", img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

# Calibrate camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("Camera Matrix:\n", mtx)
print("Distortion Coefficients:\n", dist)
```

---

### 📌 What You Need from This:

The printed **camera matrix `mtx`** is:

```
[[fx  0 cx]
 [0  fy cy]
 [0   0  1]]
```

So extract:

```python
fx = mtx[0][0]
fy = mtx[1][1]
cx = mtx[0][2]
cy = mtx[1][2]
```

Use this in your AprilTag detector like:

```python
camera_params = [fx, fy, cx, cy]
```

---

## 🚀 Optional: Use ROS for Capturing Images

If you're running ROS on the TurtleBot and publishing the OAK-D feed (e.g., via `oak-d-ros-driver`), you can record images from a topic like `/color/image_raw` using:

```bash
rosrun image_view image_view image:=/color/image_raw
```

Press **'s'** to save images for calibration.

---

## 🧪 Optional Tools

You can also use:

* [**Kalibr**](https://github.com/ethz-asl/kalibr) — for multi-sensor calibration
* [**Camera Calibration in ROS**](http://wiki.ros.org/camera_calibration) — `rosrun camera_calibration camera_calibration`

---

## ✅ Summary

| Task                   | Tool                               |
| ---------------------- | ---------------------------------- |
| Capture images         | OAK-D with Python/ROS              |
| Run calibration        | OpenCV (`cv2.calibrateCamera`)     |
| Get `[fx, fy, cx, cy]` | From the camera matrix             |
| Use in AprilTags       | `camera_params = [fx, fy, cx, cy]` |

---

Let me know if you want me to review your captured calibration images or help convert a ROS bag into intrinsics.
