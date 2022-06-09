import cv2
import numpy as np

tem = []
tml = 3
padn = 100

for i in range(1,tml+1):
    tem.append(cv2.imread('semaforo_t_%d.png' % i))
    i -= 1
    tem[i] = np.pad(cv2.pyrDown(cv2.pyrDown(tem[i])), pad_width=[(padn, padn),(padn, padn),(0, 0)], mode='constant',constant_values=(255))

orb = cv2.ORB_create(500)
orb.setScaleFactor(2)
orb2 = cv2.ORB_create(1000)

bf = cv2.BFMatcher()

kpt = []
dest = []
for i in range(tml):
    kptemp, destemp = orb.detectAndCompute(tem[i], None)
    kpt.append(kptemp)
    dest.append(destemp)

#Just testing, not for ROS

dma = []
for i in range(len(tem)):
    dma.append(cv2.drawKeypoints(tem[i], kpt[i], tem[i], flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS))
    cv2.imshow("Keypoints_%d" % (i+1), dma[i])
    print(len(kpt[i]))
cv2.waitKey(0)
cv2.destroyAllWindows()


señales = ['centro', 'lado izq', 'lado der','no']
cap= cv2.VideoCapture(0)
sizel = 21
slml = list(np.zeros(sizel))
il = list(np.ones(sizel)*-1)
while True:
    ret, frame= cap.read()
    if cv2.waitKey(1) & 0xFF == 13 or ret == False:
        break
    _, desf = orb2.detectAndCompute(frame, None)
    slm = 0
    index = -1
    for j in range(tml):
        slmc = 0
        matches = bf.knnMatch(dest[j], desf, k=2)
        for m,n in matches:
            if m.distance < 0.75*n.distance:
                slmc += 1
        if slm < slmc:
            slm = slmc
            index = j
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
    if slmp > 6:
        text3 = cv2.putText(text2, ila, org1, font, fontScale, color, thickness, cv2.LINE_AA)
    else:
        text3 = cv2.putText(text2, "no", org1, font, fontScale, color, thickness, cv2.LINE_AA)
    cv2.imshow('Matching', text3)
        
cap.release()
cv2.destroyAllWindows()

