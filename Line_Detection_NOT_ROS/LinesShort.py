import cv2
import numpy as np

strong_lines = np.zeros([4,1,2])

image = cv2.imread('Ex.png')

# Grayscale and Canny Edges extracted
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

_,msk = cv2.threshold(gray,115,255, cv2.THRESH_BINARY_INV)

erode = cv2.erode(msk, (5, 5), iterations = 7)

Cutx = (int(3*gray.shape[0]/4),gray.shape[0]-1)
Cuty = (0,gray.shape[1])

edges = cv2.Canny(erode, 0, 60)
cv2.imshow('Canny', edges)

# Run HoughLines using a rho accuracy of 1 pixel
# theta accuracy aof np.pi / 180 which is 1 degree
# The line threshold is set to 240 (number of points on line)

linesP = cv2.HoughLinesP(edges[Cutx[0]:Cutx[1],Cuty[0]:Cuty[1]], 1, np.pi / 180, 100, 1000,100)

# Iterate through each line and convert it to the format
    
if linesP is not None:
    for i in range(0, len(linesP)):
        l = linesP[i][0]
        cv2.line(edges[Cutx[0]:Cutx[1],Cuty[0]:Cuty[1]], (l[0], l[1]), (l[2], l[3]), 127, 3, cv2.LINE_AA)

cv2.imshow('Hough Lines', edges[Cutx[0]:Cutx[1],Cuty[0]:Cuty[1]])
cv2.waitKey(0)
cv2.destroyAllWindows()
