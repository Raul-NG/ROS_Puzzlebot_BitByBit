import cv2
import numpy as np

tem = []
glo = []
tml = 3
for i in range(1,tml+1):
    tem.append(np.pad(cv2.imread('semaforo_t_%d.png' % i), pad_width=[(250, 250),(250, 250),(0, 0)], mode='constant',constant_values=(255)))
    #tem.append(cv2.imread('semaforo_t_%d.png' % i))

orb = cv2.ORB_create()
orb.setScaleFactor(1.5)
flann = cv2.FlannBasedMatcher(dict(algorithm = 0, trees = 3), dict(checks = 100))

dest = []
for i in range(tml):
    _, destemp = orb.detectAndCompute(tem[i], None)
    dest.append(np.float32(destemp))

colores = ['centro','lado','lado']
cap= cv2.VideoCapture(0)
slml = []
il = []
slma = 0
ila = 0
while True:
    ret, frame= cap.read()
    if cv2.waitKey(1) & 0xFF == 13 or ret == False:
        break
    _, desf = orb.detectAndCompute(frame, None)
    desf = np.float32(desf)
    slm = 0
    index = 0
    matches = []
    matchesMask = []
    if desf.any():
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
    slml.append(slm)
    il.append(index)
    if len(slml) > 9:
        slml.pop(0)
        il.pop(0)
        slma = round(np.median(np.array(slml)))
        slmm = np.max(np.array(slml))
        ila = il[slml.index(slmm)]
    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (0,frame.shape[1] - 170)
    fontScale = 1
    color = (255,255,255)
    thickness = 1
    text1 = cv2.putText(frame, str(slma), org, font, fontScale, color, thickness, cv2.LINE_AA)
    org1 = (0,30)
    if slma > 4:
        text2 = cv2.putText(text1, colores[ila], org1, font, fontScale, color, thickness, cv2.LINE_AA)
    else:
        text2 = cv2.putText(text1, "no", org1, font, fontScale, color, thickness, cv2.LINE_AA)
    cv2.imshow('Matching', text2)
        
cap.release()
cv2.destroyAllWindows()

