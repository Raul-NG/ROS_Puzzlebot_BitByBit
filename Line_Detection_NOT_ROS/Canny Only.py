import cv2
import numpy as np

for j in range(8):
    image = cv2.imread('ExPics/Ex%d.png' % (j+1))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    GB = cv2.GaussianBlur(gray, (9,9), cv2.BORDER_DEFAULT)
    edges = cv2.Canny(GB, 10, 20)
    cv2.imshow('CannyEx%d' % (j+1), edges)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
