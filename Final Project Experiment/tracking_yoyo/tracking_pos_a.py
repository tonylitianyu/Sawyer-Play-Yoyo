import cv2
import apriltag
import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

print("[INFO] detecting AprilTags...")
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)
def apriltag_detection(gray_img):
    results = detector.detect(gray_img)
    print("[INFO] {} total AprilTags detected".format(len(results)))
    # loop over the AprilTag detection results

    yoyo_center = []
    for r in results:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(gray_img, ptA, ptB, (0, 255, 0), 2)
        cv2.line(gray_img, ptB, ptC, (0, 255, 0), 2)
        cv2.line(gray_img, ptC, ptD, (0, 255, 0), 2)
        cv2.line(gray_img, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))

        cv2.circle(gray_img, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the image
        tagID = str(r.tag_id)#tag_family.decode("utf-8")
        if r.tag_id == 1:
            yoyo_center.append((cX,cY))
            print(cX,cY)
        cv2.putText(gray_img, tagID, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        #print("[INFO] tag family: {}".format(tagFamily))

    return yoyo_center

def plot_pos(posx,posy):
    plt.plot(posx, posy)
    plt.xlim([0, 640])
    plt.ylim([0, 480])
    plt.gca().invert_yaxis()
    plt.show()


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        print('rgb source')
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    print('bgr source')
    exit(0)

# if device_product_line == 'L500':
#     config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 60)
# else:
#     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)

config.enable_stream(rs.stream.infrared, 1, 848, 100, rs.format.y8, 300)


# Start streaming
pipeline.start(config)

iter = 0

posx = []
posy = []
try:
    while iter < 1000:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_infrared_frame(1)#frames.get_color_frame()

        # # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        # color_image = np.swapaxes(color_image,0,1)
        # color_image = np.flip(color_image, 0)

        #gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        yoyo_center = apriltag_detection(color_image)
        if len(yoyo_center) == 1:
            posx.append(yoyo_center[0][0])
            posy.append(yoyo_center[0][1])

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        cv2.waitKey(1)

        im = Image.fromarray(color_image)
        im.save("frames/"+str(iter)+".png")

        iter += 1

finally:

    # Stop streaming
    pipeline.stop()
    plot_pos(posx, posy)