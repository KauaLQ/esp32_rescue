import cv2
from ultralytics import YOLO
import socket
import numpy as np
import time
import threading

# --- Configuração do ESP de Controle PID ---
ESP32_IP = "192.168.1.101"
ESP32_PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


# --- Thread dedicada de captura: sempre entrega o frame MAIS RECENTE ---
# Isso evita que o buffer interno do FFmpeg/stream acumule frames antigos
# enquanto o loop principal está ocupado fazendo inferência.
class CameraStream:
    def __init__(self, src):
        self.cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError("Erro ao acessar ESP32-CAM")

        self.ret, self.frame = self.cap.read()
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue
            with self.lock:
                self.ret = ret
                self.frame = frame

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return self.ret, self.frame.copy()

    def stop(self):
        self.running = False
        self.thread.join(timeout=2)
        self.cap.release()


# --- Inicializa captura via thread ---
cam = CameraStream("http://esp32cam.local:81/stream")

model = YOLO("yolov8n.pt")
tracked_id = None

# --- Variáveis para otimização e FPS ---
frame_count = 0
last_results = None
fps_suave = 0.0  # Inicializa o FPS estabilizado
alpha = 0.1      # Fator de suavização (quanto menor, mais suave fica o contador)

try:
    while True:
        frame_inicio = time.time()  # para cálculo de FPS

        success, img = cam.read()

        if not success or img is None:
            continue

        hs, ws, _ = img.shape
        centerX = ws // 2
        centerY = hs // 2

        # --- ByteTrack para detecção e rastreamento de pessoas por id ---
        frame_count += 1
        if frame_count % 3 == 0 or last_results is None:
            # Removido stream=True + list(): aqui é frame único, não generator.
            last_results = model.track(
                img, persist=True, tracker="bytetrack.yaml", verbose=False
            )

        results = last_results
        persons = []

        # --- Cálculo de FPS Suave e Seguro ---
        tempo_decorrido = time.time() - frame_inicio
        if tempo_decorrido > 0:  # Evita o ZeroDivisionError
            fps_instantaneo = 1 / tempo_decorrido
            if fps_suave == 0.0:
                fps_suave = fps_instantaneo
            else:
                fps_suave = (alpha * fps_instantaneo) + ((1 - alpha) * fps_suave)

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

                persons.append(
                    {"id": person_id, "bbox": (x1, y1, x2, y2), "area": area, "conf": conf}
                )

        # --- Escolhe o alvo a ser rastreado ---
        target = None
        if persons:
            if tracked_id is None:
                target = max(persons, key=lambda p: p["area"])
                tracked_id = target["id"]
            else:
                target = next((p for p in persons if p["id"] == tracked_id), None)

                if target is None:
                    tracked_id = None
                    target = max(persons, key=lambda p: p["area"])
                    tracked_id = target["id"]

        # --- Processamento do alvo e envio UDP ---
        if target is not None:
            x1, y1, x2, y2 = target["bbox"]

            fx = (x1 + x2) // 2
            fy = y1 + int((y2 - y1) * 0.20)

            area = (x2 - x1) * (y2 - y1)
            erroX = fx - centerX
            erroY = fy - centerY

            message = f"{erroX},{erroY},{area},{tracked_id}"
            sock.sendto(message.encode(), (ESP32_IP, ESP32_PORT))

            # --- Visualização do alvo e informações ---
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(img, (fx, fy), 6, (0, 0, 255), -1)
            cv2.line(img, (centerX, centerY), (fx, fy), (255, 0, 0), 2)
            cv2.putText(img, f"ID: {tracked_id}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(img, f"A: {area}px", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(img, "ALVO RASTREADO", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(img, f"ErroX: {erroX}", (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(img, f"ErroY: {erroY}", (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(img, f"FPS: {fps_suave:.1f}", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        else:
            tracked_id = None
            cv2.putText(img, "SEM ALVO", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(img, f"FPS: {fps_suave:.1f}", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        # --- Desenho de cruz central para referência ---
        cv2.circle(img, (centerX, centerY), 15, (255, 0, 255), 2)
        cv2.line(img, (centerX, 0), (centerX, hs), (255, 0, 255), 1)
        cv2.line(img, (0, centerY), (ws, centerY), (255, 0, 255), 1)

        cv2.imshow("Rescue Tracker", img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    cam.stop()
    cv2.destroyAllWindows()