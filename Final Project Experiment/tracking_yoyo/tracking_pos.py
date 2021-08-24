# Standard imports
import cv2
import numpy as np
import pyrealsense2 as rs
import time as time
import matplotlib.pyplot as plt
from PIL import Image


def setBlobParam():
    params = cv2.SimpleBlobDetector_Params()                
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 1
    params.maxArea = 100

    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = 0.1

    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.87

    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.01

    params.filterByColor = True
    params.blobColor = 0

    return params

def findBlob(blob_params, img_frame):
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(blob_params)
    else : 
        detector = cv2.SimpleBlobDetector_create(blob_params)

    thresh = 100
    img = cv2.threshold(img_frame, thresh, 255, cv2.THRESH_BINARY)[1]

    #find keypoints
    detector.empty()
    keypoints = detector.detect(img)
    # if len(keypoints) > 0:
    #     print(keypoints[0].pt[0])
    #     print(keypoints[0].pt[1])


    blob_img = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DEFAULT)
    return blob_img, keypoints

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

blob_params = setBlobParam()




iter = 0
posx = []
posy = []
try:
    while iter < 1000:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_infrared_frame(1)#frames.get_color_frame()

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())



        # hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        # blueMin = (95,230,150)
        # blueMax = (110, 250, 160)
        # mask = 255 - cv2.inRange(hsv, blueMin, blueMax)

        # #for color calibration
        # #image_center = (int(color_image.shape[1]/2),int(color_image.shape[0]/2))

        # image_center = (365,315)
        # image = cv2.circle(color_image.copy(), image_center, radius=5, color=(0, 0, 255), thickness=-1)
        
        # image_origin = (365, 40)
        # image = cv2.circle(image.copy(), image_origin, radius=5, color=(255, 0, 0), thickness=-1)

        # #pixel distance 275 pixel
        # #actual distance 0.82 m
        # #meter per pixel 0.00298181818 meter per pixel

        # #print(hsv[image_center[1]][image_center[0]])


        # blob_img, keypoints = findBlob(blob_params, mask)

        # if len(keypoints) > 0:
        #     # curr_posx = - (365 - keypoints[0].pt[0])
        #     # curr_posy = keypoints[0].pt[1] - 40
        #     curr_posx = keypoints[0].pt[0]
        #     curr_posy= keypoints[0].pt[1]
        #     posx.append(curr_posx)
        #     posy.append(curr_posy)

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