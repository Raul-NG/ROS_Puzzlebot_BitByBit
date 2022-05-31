import cv2
import numpy as np

tem = []
glo = []
for i in range(1,6):
    tem.append(cv2.imread('semaforo_t_%d.png' % i))
    glo.append(cv2.cvtColor(tem[i-1], cv2.COLOR_BGR2GRAY))

orb = cv2.ORB_create(1000)
orb.setScaleFactor(1.2)
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 3)
search_params = dict(checks = 100)
flann = cv2.FlannBasedMatcher(index_params, search_params)

kpt = []
dest = []
for i in range(5):
    kptemp, destemp = orb.detectAndCompute(glo[i], None)
    kpt.append(kptemp)
    dest.append(np.float32(destemp))

colores = ['verde', 'amarillo', 'rojo','apagado 1', 'apagado 2']
cap= cv2.VideoCapture(0)
slml = []
il = []
slma = 0
ila = 0
while True:
    ret, frame= cap.read()
    if cv2.waitKey(1) & 0xFF == 13 or ret == False:
        break
    gfr = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kpf, desf = orb.detectAndCompute(gfr, None)
    desf = np.float32(desf)
    slm = 0
    index = 0
    matches = []
    matchesMask = []
    for j in range(5):
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
    if len(slml) > 10:
        slml.pop(0)
        il.pop(0)
        slma = round(np.median(np.array(slml)))
        ila = round(np.median(np.array(il)))
    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (0,frame.shape[1] - 170)
    fontScale = 1
    color = (255,255,255)
    thickness = 1
    text1 = cv2.putText(frame, str(slma), org, font, fontScale, color, thickness, cv2.LINE_AA)
    org1 = (0,30)
    if slma > 2:
        text2 = cv2.putText(text1, colores[ila], org1, font, fontScale, color, thickness, cv2.LINE_AA)
    else:
        text2 = cv2.putText(text1, "no", org1, font, fontScale, color, thickness, cv2.LINE_AA)
    cv2.imshow('Matching', text2)
        
cap.release()
cv2.destroyAllWindows()

