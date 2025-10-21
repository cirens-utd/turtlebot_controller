import cv2
import depthai as dai

pipeline = dai.Pipeline()
camRgb = pipeline.create(dai.node.ColorCamera)
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("video")
camRgb.video.link(xout.input)

monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
xoutLeft = pipeline.create(dai.node.XLinkOut)
xoutRight = pipeline.create(dai.node.XLinkOut)
xoutLeft.setStreamName('left')
xoutRight.setStreamName('right')

# Properites
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)

# Linking
monoRight.out.link(xoutRight.input)
monoLeft.out.link(xoutLeft.input)

with dai.Device(pipeline) as device:
    video = device.getOutputQueue("video", maxSize=8, blocking=False)
    
    qLeft = device.getOutputQueue(name="left", maxSize=4, blocking=False)
    qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)

    img_count = 0
    while True:
        inFrame = video.get()
        inLeft = qLeft.tryGet()
        inRight = qRight.tryGet()

        frame = inFrame.getCvFrame()
        # print("Image Shown")
        cv2.imshow("Calib", frame)

        key = cv2.waitKey(1)
        if key == ord("c"):
            # RGB Frame
            cv2.imwrite(f"calib_img_rgb_{img_count}.png", frame)
            print(f"Captured calib_img_rgb_{img_count}.png")

            # Left MONO Frame
            if inLeft is not None:
                mono_frame = inLeft.getCvFrame()
                cv2.imwrite(f"calib_img_left_{img_count}.png", mono_frame)
                print(f"Captured calib_img_left_{img_count}.png")

            # Right MONO FRame
            if inRight is not None:
                mono_frame = inRight.getCvFrame()
                cv2.imwrite(f"calib_img_right_{img_count}.png", mono_frame)
                print(f"Captured calib_img_right_{img_count}.png")


            img_count += 1
        elif key == 27:  # ESC
            break

cv2.destroyAllWindows()