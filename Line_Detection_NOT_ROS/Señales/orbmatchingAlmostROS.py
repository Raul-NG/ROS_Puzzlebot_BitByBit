import cv2
import numpy as np

tem = []
tml = 5

for i in range(tml):
    tem.append(cv2.imread('%d.jpeg' % i))
    tem[i] = np.pad(cv2.pyrDown(cv2.pyrDown(tem[i])), pad_width=[(50, 50),(50, 50),(0, 0)], mode='constant',constant_values=(255))

orb = cv2.ORB_create(500,2)
orb2 = cv2.ORB_create(1000)
flann = cv2.FlannBasedMatcher(dict(algorithm = 0, trees = 3), dict(checks = 1000))

kpt = []
dest = []
for i in range(tml):
    kptemp, destemp = orb.detectAndCompute(tem[i], None)
    kpt.append(kptemp)
    dest.append(np.float32(destemp))

#Just testing, not for ROS
"""
dma = []
for i in range(len(tem)):
    dma.append(cv2.drawKeypoints(tem[i], kpt[i], tem[i], flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS))
    cv2.imshow("Keypoints_%d" % (i+1), dma[i])
    print(len(kpt[i]))
cv2.waitKey(0)
cv2.destroyAllWindows()
"""

señales = ['stop', 'continue', 'round','turn', 'no speed limit','no']
cap= cv2.VideoCapture(0)
sizel = 49
slml = list(np.zeros(sizel))
il = list(np.ones(sizel)*-1)
while True:
    ret, frame= cap.read()
    if cv2.waitKey(1) & 0xFF == 13 or ret == False:
        break
    _, desf = orb2.detectAndCompute(frame, None)
    desf = np.float32(desf)
    slm = 0
    index = 0
    matches = []
    matchesMask = []
    for j in range(tml):
        matches.append(flann.knnMatch(dest[j], desf, k=2))
        matchesMask.append([[0,0] for k in range(len(matches[j]))])
        for k,(m,n) in enumerate(matches[j]):
            if m.distance < 0.7*n.distance:
                matchesMask[j][k]=[1,0]
        matchesMask[j] = np.array(matchesMask[j])
        if slm < np.sum(matchesMask[j][:,0]):
            slm = np.sum(matchesMask[j][:,0])
            index = j
        if slm == 0:
            index = -1
    slml.append(slm)
    il.append(index)
    if len(slml) > sizel:
        slml.pop(0)
        il.pop(0)
    slmp = np.mean(np.array(slml))
    vals, counts = np.unique(il, return_counts=True)
    ila = señales[int(vals[np.argwhere(counts == np.max(counts))][0])]
    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (0,frame.shape[1] - 170)
    fontScale = 1
    color = (255,255,255)
    thickness = 1
    text2 = cv2.putText(frame, str(round(slmp,2)), org, font, fontScale, color, thickness, cv2.LINE_AA)
    org1 = (0,30)
    if slmp > 2:
        text3 = cv2.putText(text2, ila, org1, font, fontScale, color, thickness, cv2.LINE_AA)
    else:
        text3 = cv2.putText(text2, "no", org1, font, fontScale, color, thickness, cv2.LINE_AA)
    cv2.imshow('Matching', text3)
        
cap.release()
cv2.destroyAllWindows()

