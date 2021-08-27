import cv2
import numpy as np
import EasyPySpin
import apriltag
from PIL import Image
import time

print("[INFO] detecting AprilTags...")
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)


def apriltag_detection(gray_img):
    results = detector.detect(gray_img)
    #print("[INFO] {} total AprilTags detected".format(len(results)))
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
        if r.tag_id == 2:
            #yoyo_center.append((cX,cY))
            #print(cX,cY)
            yoyo_center = [str(cX), str(cY)]
        cv2.putText(gray_img, tagID, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        #print("[INFO] tag family: {}".format(tagFamily))

    return yoyo_center



cap = EasyPySpin.VideoCapture(0)

cap.set(cv2.CAP_PROP_FPS, 500)
fps  = cap.get_pyspin_value("AcquisitionFrameRate")
print('fps ' + str(fps))

width  = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
print("image width: " + str(width))
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print("image height: " + str(height))


iter = 0
file = open("yoyo_pos.txt","w")
#start = time.time()
while True:
    ret, frame = cap.read()

    frame.setflags(write=1)
    frame[0:450, 50:200] = np.array(np.fliplr(frame[0:450, 50:200]))
    

    yoyo_center = apriltag_detection(frame)


    # delim = ", "
    # if len(yoyo_center) == 0:
    #     yoyo_center = ['None', 'None']

    # file.write(delim.join(yoyo_center) + "\n")
    #print(yoyo_center)

    cv2.imshow('img', frame)
    key = cv2.waitKey(30)
    if key == ord("q"):
        break
    # im = Image.fromarray(frame)
    # im.save("frames/"+str(iter)+".png")


    iter += 1

print(iter)
file.close()

cap.release()