import cv2
import numpy as np

tem = cv2.imread('template.jpg')

glo = cv2.cvtColor(tem, cv2.COLOR_BGR2GRAY)

cv2.imshow("template",glo)

cv2.waitKey(0)
cv2.destroyAllWindows()

orb = cv2.ORB_create(1000)
orb.setScaleFactor(1.2)

kpt, dest = orb.detectAndCompute(glo, None)

if len(kpt) < 200:
    print("Not enough")

dma = cv2.drawKeypoints(tem, kpt, tem, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv2.imshow("Keypoints", dma)

cv2.waitKey(0)
cv2.destroyAllWindows()

tem1 = cv2.imread('template1.jpg')

glo1 = cv2.cvtColor(tem1, cv2.COLOR_BGR2GRAY)

cv2.imshow("template1",glo1)

cv2.waitKey(0)
cv2.destroyAllWindows()

kpt1, dest1 = orb.detectAndCompute(glo1, None)

if len(kpt1) < 200:
    print("Not enough")

dma1 = cv2.drawKeypoints(tem1, kpt1, tem1, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv2.imshow("Keypoints", dma1)

cv2.waitKey(0)
cv2.destroyAllWindows()

# Brute Force Matching

bf = cv2.BFMatcher_create(cv2.NORM_HAMMING, crossCheck=True)

matches = bf.match(dest, dest1)
matches = sorted(matches, key = lambda x:x.distance)
matching_result = cv2.drawMatches(tem, kpt, tem1, kpt1, matches[:100], None, 
flags=2)
cv2.imshow("Matches", matching_result)
cv2.waitKey(0)
cv2.destroyAllWindows()

cap= cv2.VideoCapture(0)
while True:
    ret, frame= cap.read()
    if cv2.waitKey(1) & 0xFF == 13 or ret == False:
        break
    gfr = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kpf, desf = orb.detectAndCompute(gfr, None)
    matches = bf.match(dest, desf)
    matches = sorted(matches, key = lambda x:x.distance)
    slm = len(matches)
    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (0,frame.shape[1] - 170)
    fontScale = 1
    color = (255,255,255)
    thickness = 1
    text1 = cv2.putText(frame, str(slm), org, font, fontScale, color, thickness, cv2.LINE_AA)
    org1 = (0,30)
    if slm > 200:
        text2 = cv2.putText(text1, "yes", org1, font, fontScale, color, thickness, cv2.LINE_AA)
    else:
        text2 = cv2.putText(text1, "no", org1, font, fontScale, color, thickness, cv2.LINE_AA)
    cv2.imshow('Matching', text2)
        
cap.release()
cv2.destroyAllWindows()

