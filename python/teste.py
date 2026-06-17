import cv2
import time

cap = cv2.VideoCapture("http://esp32cam.local:81/stream")

while True:
    t0 = time.time()

    success, img = cap.read()

    if not success:
        continue

    fps = 1/(time.time()-t0)

    cv2.putText(img, f"FPS: {fps:.1f}",
                (20,40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0,255,0),
                2)

    cv2.imshow("Teste", img)

    if cv2.waitKey(1) == ord("q"):
        break