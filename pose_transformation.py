import cv2
import numpy as np

p = np.matrix('487.608731712861 0.0 318.1162109375 0.0; 0.0 487.608731712861 249.44425201416 0.0; 0.0 0.0 1.0 0.0')
initialFrame = cv2.imread('frame0000.jpg',0)
cameraPosition = np.matrix('11.6458148956; -4.59024477005; -0.131090074778; 1')

imagePoint = p * cameraPosition
imagePoint = imagePoint/imagePoint[2,0]
print "Below is the Image point obtained"
print imagePoint
cv2.circle(initialFrame, (int(imagePoint[0,0]), int(imagePoint[1,0])), 15, (0, 0, 255), 3, 8, 0)
cv2.imshow('Initial frame',initialFrame)
cv2.waitKey(0)
