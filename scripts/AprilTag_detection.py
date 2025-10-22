#!/usr/bin/env python3

import cv2
import numpy as np
import depthai as dai
from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation as R
import pdb

### python3 -m pip install --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-snapshot-local/ depthai
### python3 -m pip install pupil-apriltags, opencv-python
### Tags: https://github.com/AprilRobotics/apriltag-imgs/tree/master
'''
Doesn't Work...
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag/image

# Example: Generate tag36h11 family (IDs 0–9)
./tag36h11_print.py 0 9

Install Depthai:
https://github.com/luxonis/depthai.git

python3 install_requirements.py
python3 depthai_demo.py
'''

'''
robot1:
RBG - camera_params=([1486.56, 1489.024, 953.16, 560.76])
Left = camera_params=([4348.13, 3086.80, 164.31, 265.95])
Right = camera_params=([1487.80, 1340.60, 67.40, 447.64])

'''

# Create pipeline
pipeline = dai.Pipeline()

# Define source and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
xoutPreview = pipeline.create(dai.node.XLinkOut)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
xoutLeft = pipeline.create(dai.node.XLinkOut)
xoutRight = pipeline.create(dai.node.XLinkOut)
xoutLeft.setStreamName('left')
xoutRight.setStreamName('right')
xoutPreview.setStreamName("preview")

# Properties
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
camRgb.setPreviewSize(960, 540)
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(True)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

# Linking
camRgb.preview.link(xoutPreview.input)
monoRight.out.link(xoutRight.input)
monoLeft.out.link(xoutLeft.input)
# Connect to device and start pipeline


def draw_tags(
        image,
        tags):
    for tag in tags:
        tag_family = tag.tag_family
        tag_id = tag.tag_id
        center = tag.center
        corners = tag.corners

        center = (int(center[0]), int(center[1]))
        corner_01 = (int(corners[0][0]), int(corners[0][1]))
        corner_02 = (int(corners[1][0]), int(corners[1][1]))
        corner_03 = (int(corners[2][0]), int(corners[2][1]))
        corner_04 = (int(corners[3][0]), int(corners[3][1]))

        # 中心
        cv2.circle(image,
                   (center[0], center[1]), 5, (255, 0, 255), 2)

        # 各辺
        cv2.line(image, (corner_01[0], corner_01[1]),
                 (corner_02[0], corner_02[1]), (255, 0, 0), 2)
        cv2.line(image, (corner_02[0], corner_02[1]),
                 (corner_03[0], corner_03[1]), (255, 0, 0), 2)
        cv2.line(image, (corner_03[0], corner_03[1]),
                 (corner_04[0], corner_04[1]), (0, 255, 0), 2)
        cv2.line(image, (corner_04[0], corner_04[1]),
                 (corner_01[0], corner_01[1]), (0, 255, 0), 2)
        cv2.putText(image, str(tag_id), (center[0] - 50, center[1] - 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
    return image


at_detector = Detector(
    families='tag36h11',
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0)
with dai.Device(pipeline) as device:
    qLeft = device.getOutputQueue(name="left", maxSize=4, blocking=False)
    qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)
    preview = device.getOutputQueue('preview')
    count = 0
    while True:
        inLeft = qLeft.tryGet()
        inRight = qRight.tryGet()
        previewFrame = preview.get()
        # if inLeft is not None:
        #     mono_frame = inLeft.getCvFrame()
        # if inRight is not None:
        #     mono_frame = inRight.getCvFrame()
        if inLeft is not None:
            tags = at_detector.detect(
                inLeft.getCvFrame(),
                estimate_tag_pose=True,
                camera_params=([4348.13, 3086.80, 164.31, 265.95]),
                tag_size=0.1651,
            )
            debug_image = draw_tags(previewFrame.getCvFrame(), tags)
            # cv2.imshow("left", debug_image)
            if tags:
                # x = right, y = down, z =  away from camera
                x, y, z = tags[0].pose_t.flatten()
                quat = R.from_matrix(tags[0].pose_R).as_quat() # [x, y, z, w]
                angle = np.remainder((np.arctan2(2 * (quat[3] * quat[2] + quat[0] * quat[1]),1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2])) + np.pi) , 2 * np.pi)
                if count == 0:
                    print(f"Pose: {tags[0].pose_t.flatten()}")
                    print(f"Angle: {angle}")
                elif count == 60:
                    count = 0
                else:
                    count += 1
                # print(f"Left Pose: {tags[0].pose_t.flatten()}")
        if inRight is not None:
            tags = at_detector.detect(
                inRight.getCvFrame(),
                estimate_tag_pose=True,
                camera_params=([1487.80, 1340.60, 67.40, 447.64]),
                tag_size=0.05,
            )
            debug_image = draw_tags(previewFrame.getCvFrame(), tags)
            # cv2.imshow("right", debug_image)
            if tags:
                # pdb.set_trace()
                '''
                Tags:
                [
                Detection object:
                tag_family = b'tag36h11'
                tag_id = 0
                hamming = 1
                decision_margin = 79.80211639404297
                homography = [[ 2.06804203e+01  3.92855814e+00  3.24899243e+02]
                [ 1.14190255e+00  1.48818948e+01  1.98456930e+02]
                [ 1.70840783e-02 -4.25033722e-03  1.00000000e+00]]
                center = [324.89924257 198.4569301 ]
                corners = [[314.86483765 216.82270813]
                [345.07955933 211.76301575]
                [334.51443481 180.85842896]
                [304.19421387 184.80487061]]
                pose_R = None
                pose_t = None
                pose_err = None
                ]
                '''
                # x = right, y = down, z =  away from camera
                x, y, z = tags[0].pose_t.flatten()
                quat = R.from_matrix(tags[0].pose_R).as_quat() # [x, y, z, w]
                # print(f"Right Pose: {tags[0].pose_t.flatten()}")

        # Get BGR frame from NV12 encoded video frame to show with opencv
        #cv2.imshow("video", videoFrame.getCvFrame())
        # Show 'preview' frame as is (already in correct format, no copy is made)
        # cv2.imshow("preview", previewFrame.getCvFrame())
        # cv2.imshow("right", inRight.getCvFrame())
        if cv2.waitKey(1) == ord('q'):
            break



'''
[ Right is postive , Down is Postive, Positive Forward]
'''