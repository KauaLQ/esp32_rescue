import cv2
from ultralytics import YOLO
import socket
import numpy as np

# --- Configuração do ESP de Controle PID ---
ESP32_IP = "192.168.1.104"
ESP32_PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- Configuração do ESP32-CAM ---
cap = cv2.VideoCapture("http://esp32cam.local:81/stream", cv2.CAP_FFMPEG)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
if not cap.isOpened():
    print("Erro ao acessar ESP32-CAM")
    exit()

model = YOLO("yolov8n.pt")
tracked_id = None

while True:
    success, img = cap.read()

    if not success or img is None:
        continue

    hs, ws, _ = img.shape
    centerX = ws // 2
    centerY = hs // 2

    # --- Byte Track para detecção e rastreamento de pessoas por id ---
    results = model.track(img, persist=True, tracker="bytetrack.yaml", verbose=False)
    persons = []

    # --- Processa resultados para extrair pessoas e seus ids ---
    for r in results:
        if r.boxes is None:
            continue

        for box in r.boxes:
            cls = int(box.cls[0])

            # Classe 0 = pessoa
            if cls != 0:
                continue

            conf = float(box.conf[0])

            if conf < 0.50:
                continue

            if box.id is None:
                continue

            person_id = int(box.id[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)

            persons.append({"id": person_id, "bbox": (x1, y1, x2, y2), "area": area, "conf": conf})

    # --- Escolhe o alvo a ser rastreado ---
    target = None
    if persons:
        # ainda não existe alvo
        if tracked_id is None:
            target = max(persons, key=lambda p: p["area"])
            tracked_id = target["id"]

        else:
            # tenta achar a pessoa já rastreada
            target = next(
                (
                    p for p in persons
                    if p["id"] == tracked_id
                ),
                None
            )

            # perdeu o alvo
            if target is None:
                tracked_id = None
                target = max(persons, key=lambda p: p["area"])
                tracked_id = target["id"]

    # --- Processamento do alvo e envio UDP ---
    if target is not None:
        x1, y1, x2, y2 = target["bbox"]

        fx = (x1 + x2) // 2
        fy = (y1 + y2) // 2

        altura = y2 - y1
        erroX = fx - centerX
        erroY = fy - centerY

        message = (f"{erroX}," f"{erroY}," f"{altura}," f"{tracked_id}")
        sock.sendto(message.encode(), (ESP32_IP, ESP32_PORT))

        # --- Visualização do alvo e informações ---
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(img, (fx, fy), 6, (0, 0, 255), -1)
        cv2.line(img, (centerX, centerY), (fx, fy), (255, 0, 0), 2)
        cv2.putText(img, f"ID: {tracked_id}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(img, f"H: {altura}px", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(img, "ALVO RASTREADO", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(img, f"ErroX: {erroX}", (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(img, f"ErroY: {erroY}", (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    else:
        tracked_id = None
        cv2.putText(img, "SEM ALVO", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # --- Desenho de cruz central para referência ---
    cv2.circle(img, (centerX, centerY), 15, (255, 0, 255), 2)
    cv2.line(img, (centerX, 0), (centerX, hs), (255, 0, 255), 1)
    cv2.line(img, (0, centerY), (ws, centerY), (255, 0, 255), 1)

    cv2.imshow("Rescue Tracker", img)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()