# arquivo de teste para a câmera ESP32-CAM
# Não utilizado no projeto final, apenas para testes rápidos 
import cv2
import time

cap = cv2.VideoCapture("http://esp32cam.local:81/stream")

while True:
    t0 = time.time()

    success, img = cap.read()

    if not success:
        continue

    cv2.imshow("Teste", img)

    if cv2.waitKey(1) == ord("q"):
        break