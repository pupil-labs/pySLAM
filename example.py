import pySLAM
import cv2
import numpy as np
from time import time,sleep
import os
undistorter = pySLAM.Slam_Undistorter('/home/michael/LSD_room/cameraCalibration.cfg')

files = [os.path.join('/home/michael/LSD_room/images',f) for f in os.listdir('/home/michael/LSD_room/images/')]
files.sort()

#K = np.float32(np.load('/home/michael/git/pupil/capture_settings/camera_matrix.npy'))
#dist_coef = np.float32(np.load('/home/michael/git/pupil/capture_settings/dist_coefs.npy'))

# compute undistort transformation for debug view    
fieldcam_res = (640, 480)
#newCameraMatrix, _ = cv2.getOptimalNewCameraMatrix(K, dist_coef, fieldcam_res, 1, fieldcam_res)
#map1, map2 = cv2.initUndistortRectifyMap(K, dist_coef, np.identity(3), newCameraMatrix, fieldcam_res, cv2.CV_16SC2)

K = np.zeros((3,3), dtype=np.float32)
K[0,0] = 254.326950
K[1,1] = 375.934387
K[2,0] = 266.881897
K[2,1] = 231.099091
K[2,2] = 1.0

cap = cv2.VideoCapture('/home/michael/git/pupil/recordings/2015_06_17/000/world.mkv')
cap.set(3,640)
cap.set(4,480)
f = files.pop(0)
print f
img = cv2.imread(f, cv2.CV_LOAD_IMAGE_GRAYSCALE)
#img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#img = undistorter.undistort(img)
print img.shape

system = pySLAM.Slam_Context(img.shape[1],img.shape[0], K.flatten())
system.init(img,0,0)
ts = time()
x = 0
for f in files:
#for x in range(1,9999):
    x = x+1
    print f
    img = cv2.imread(f, cv2.CV_LOAD_IMAGE_GRAYSCALE)
    #img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    #img = undistorter.undistort(img)
    system.track_frame(img,x,x/60,True) 
duration = time() - ts

print "DURATION"
print duration
print "FPS"
print x / duration
 
system.finalize()