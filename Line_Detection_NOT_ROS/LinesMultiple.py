import cv2
import numpy as np

image = cv2.imread('Ex.png')

# Grayscale and Canny Edges extracted
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

_,msk = cv2.threshold(gray,120,255, cv2.THRESH_BINARY)

erode = cv2.erode(msk, (5, 5), iterations = 6)

Cutx = (int(3*gray.shape[0]/4),gray.shape[0]-10)
Cuty = (int(gray.shape[1]/4),int((3*gray.shape[1])/4))

edges = cv2.Canny(erode, 0, 60)
cv2.imshow('Canny', edges)

# Run HoughLines using a rho accuracy of 1 pixel
# theta accuracy aof np.pi / 180 which is 1 degree
# The line threshold is set to 240 (number of points on line)
lines = cv2.HoughLines(edges[Cutx[0]:Cutx[1],Cuty[0]:Cuty[1]], 1, np.pi / 180, 50)

# Iterate through each line and convert it to the format
for i in range(len(lines)): 
    rho, theta = lines[i,0,:]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))
    cv2.line(edges[Cutx[0]:Cutx[1],Cuty[0]:Cuty[1]], (x1, y1), (x2, y2), (127), 2)

cv2.imshow('Hough Lines', edges[Cutx[0]:Cutx[1],Cuty[0]:Cuty[1]])
cv2.waitKey(0)
cv2.destroyAllWindows()
