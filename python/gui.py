"""
Rescue Tracker - GUI
=====================
Interface Tkinter para o sistema de rastreamento YOLO + ESP32-CAM.

ARQUITETURA (importante entender antes de modificar):

  Thread 1 (CameraStream._update) -> só captura frames do stream HTTP,
            sempre mantém o frame MAIS RECENTE em memória.

  Thread 2 (TrackerWorker.run)    -> pega o frame mais recente, roda
            YOLO + ByteTrack, desenha as anotações, envia UDP pro ESP32,
            e guarda o resultado anotado em self.display_frame.
            Essa thread roda em loop livre, na velocidade que conseguir,
            SEM esperar a GUI.

  Thread principal (Tkinter)      -> só faz dois trabalhos, ambos leves:
            1) a cada ~33ms (30 FPS de display), pega display_frame e
               desenha no Label da câmera.
            2) lê/escreve os campos de IP, porta e variáveis extras.

Como a inferência roda numa thread própria e a GUI só "olha" o resultado
através de um lock (sem bloquear a thread de tracking), o desempenho do
YOLO não é afetado pela interface. A GUI nunca espera o YOLO, e o YOLO
nunca espera a GUI.
"""

import socket
import threading
import time
import tkinter as tk
from tkinter import ttk
import cv2
import numpy as np
from PIL import Image, ImageTk
from ultralytics import YOLO

# -----------------------------------------------------------
# CONFIGURAÇÃO PADRÃO (pode ser alterada na própria interface)
# -----------------------------------------------------------
DEFAULT_ESP32_IP = "192.168.1.104"
DEFAULT_ESP32_PORT = 4210
DEFAULT_STREAM_URL = "http://esp32cam.local:81/stream"
CAM_DISPLAY_W = 640
CAM_DISPLAY_H = 480
GUI_REFRESH_MS = 33  # ~30 FPS para o display (a thread de tracking roda livre, independente disso)

# -----------------------------------------------------------
# THREAD DE CAPTURA
# -----------------------------------------------------------
class CameraStream:
    def __init__(self, src):
        self.src = src
        self.cap = None
        self.ret = False
        self.frame = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.error = None

    def start(self):
        self.cap = cv2.VideoCapture(self.src, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.error = "Erro ao acessar ESP32-CAM (stream não abriu)."
            return False

        self.ret, self.frame = self.cap.read()
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        return True

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
        if self.thread is not None:
            self.thread.join(timeout=2)
        if self.cap is not None:
            self.cap.release()


# -----------------------------------------------------------
# THREAD DE PROCESSAMENTO - YOLO + ByteTrack + envio UDP
# -----------------------------------------------------------
class TrackerWorker:
    def __init__(self, cam: CameraStream, get_esp_target, log_callback):
        self.cam = cam
        self.get_esp_target = get_esp_target  # função -> (ip, porta) atuais, lidos da GUI
        self.log_callback = log_callback       # função para mandar mensagens pro Monitor/Erros da GUI

        self.model = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.tracked_id = None
        self.frame_count = 0
        self.last_results = None
        self.fps_suave = 0.0
        self.alpha = 0.1

        self.running = False
        self.thread = None

        # Frame anotado mais recente, pronto para exibição (protegido por lock)
        self.display_lock = threading.Lock()
        self.display_frame = None

        # Estatísticas para exibir na GUI (erroX, erroY, area, id, fps)
        self.stats_lock = threading.Lock()
        self.stats = {"erroX": 0, "erroY": 0, "area": 0, "id": None, "fps": 0.0, "alvo": False}

    def load_model(self, weights_path="yolov8n.pt"):
        self.model = YOLO(weights_path)

    def start(self):
        if self.model is None:
            self.load_model()
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2)

    def get_display_frame(self):
        with self.display_lock:
            if self.display_frame is None:
                return None
            return self.display_frame.copy()

    def run(self):
        try:
            while self.running:
                frame_inicio = time.time()

                success, img = self.cam.read()
                if not success or img is None:
                    continue

                hs, ws, _ = img.shape
                centerX = ws // 2
                centerY = hs // 2

                # --- ByteTrack para detecção e rastreamento de pessoas por id ---
                self.frame_count += 1
                if self.frame_count % 3 == 0 or self.last_results is None:
                    self.last_results = self.model.track(
                        img, persist=True, tracker="bytetrack.yaml", verbose=False
                    )

                results = self.last_results
                persons = []

                # --- Cálculo de FPS Suave e Seguro ---
                tempo_decorrido = time.time() - frame_inicio
                if tempo_decorrido > 0:
                    fps_instantaneo = 1 / tempo_decorrido
                    if self.fps_suave == 0.0:
                        self.fps_suave = fps_instantaneo
                    else:
                        self.fps_suave = (self.alpha * fps_instantaneo) + ((1 - self.alpha) * self.fps_suave)

                # --- Processa resultados para extrair pessoas e seus ids ---
                for r in results:
                    if r.boxes is None:
                        continue
                    for box in r.boxes:
                        cls = int(box.cls[0])
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
                    if self.tracked_id is None:
                        target = max(persons, key=lambda p: p["area"])
                        self.tracked_id = target["id"]
                    else:
                        target = next((p for p in persons if p["id"] == self.tracked_id), None)
                        if target is None:
                            self.tracked_id = None
                            target = max(persons, key=lambda p: p["area"])
                            self.tracked_id = target["id"]

                # --- Processamento do alvo e envio UDP ---
                if target is not None:
                    x1, y1, x2, y2 = target["bbox"]
                    fx = (x1 + x2) // 2
                    fy = y1 + int((y2 - y1) * 0.35)
                    area = (x2 - x1) * (y2 - y1)
                    erroX = fx - centerX
                    erroY = fy - centerY

                    esp_ip, esp_port = self.get_esp_target()
                    message = f"{erroX},{erroY},{area},{self.tracked_id}"
                    try:
                        self.sock.sendto(message.encode(), (esp_ip, esp_port))
                    except OSError as e:
                        self.log_callback("erro", f"Falha ao enviar UDP: {e}")

                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(img, (fx, fy), 6, (0, 0, 255), -1)
                    cv2.line(img, (centerX, centerY), (fx, fy), (255, 0, 0), 2)
                    cv2.putText(img, f"ID: {self.tracked_id}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(img, f"A: {area}px", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(img, "ALVO RASTREADO", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    cv2.putText(img, f"ErroX: {erroX}", (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    cv2.putText(img, f"ErroY: {erroY}", (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    cv2.putText(img, f"FPS: {self.fps_suave:.1f}", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

                    with self.stats_lock:
                        self.stats = {
                            "erroX": erroX, "erroY": erroY, "area": area,
                            "id": self.tracked_id, "fps": self.fps_suave, "alvo": True,
                        }
                else:
                    self.tracked_id = None
                    cv2.putText(img, "SEM ALVO", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    cv2.putText(img, f"FPS: {self.fps_suave:.1f}", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

                    with self.stats_lock:
                        self.stats = {
                            "erroX": 0, "erroY": 0, "area": 0,
                            "id": None, "fps": self.fps_suave, "alvo": False,
                        }

                # --- Desenho de cruz central para referência ---
                cv2.circle(img, (centerX, centerY), 15, (255, 0, 255), 2)
                cv2.line(img, (centerX, 0), (centerX, hs), (255, 0, 255), 1)
                cv2.line(img, (0, centerY), (ws, centerY), (255, 0, 255), 1)

                with self.display_lock:
                    self.display_frame = img

        except Exception as e:
            self.log_callback("erro", f"Thread de tracking parou: {e}")


# -----------------------------------------------------------
# INTERFACE GRÁFICA - estilo "tabela"
# -----------------------------------------------------------
class RescueTrackerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Rescue Tracker | ESP32-CAM Controller")
        self.root.geometry("1000x720")
        self.root.minsize(900, 650)

        self.cam = None
        self.worker = None
        self.connected = False

        self._build_layout()

    # -------------------- LAYOUT --------------------
    def _build_layout(self):
        style = ttk.Style()
        style.theme_use("default")
        style.configure("Connect.TButton", background="#90EE90")
        style.configure("Disconnect.TButton", background="#FFB3B3")

        # ---- Linha 1: câmera (esquerda) + painel de conexão/variáveis (direita) ----
        top_frame = tk.Frame(self.root)
        top_frame.pack(fill="both", expand=True, padx=8, pady=8)

        # --- Painel da câmera ---
        cam_frame = tk.LabelFrame(top_frame, text="Câmera (ESP32-CAM)")
        cam_frame.pack(side="left", fill="both", expand=True, padx=(0, 6))

        self.cam_label = tk.Label(
            cam_frame, text="Sem sinal", bg="black", fg="white",
            width=CAM_DISPLAY_W // 8, height=CAM_DISPLAY_H // 16,
        )
        self.cam_label.pack(fill="both", expand=True, padx=4, pady=4)

        # --- Painel lateral: tabela de configuração ---
        side_frame = tk.Frame(top_frame, width=340)
        side_frame.pack(side="right", fill="y")

        config_table = tk.Frame(side_frame, highlightbackground="black", highlightthickness=1)
        config_table.pack(fill="x", pady=(0, 8))

        self._add_row(config_table, 0, "IP ESP32:", default=DEFAULT_ESP32_IP, attr="ip_entry")
        self._add_row(config_table, 1, "Porta UDP:", default=str(DEFAULT_ESP32_PORT), attr="port_entry")
        self._add_row(config_table, 2, "Stream URL:", default=DEFAULT_STREAM_URL, attr="stream_entry")

        # Botão Conectar / Desconectar
        self.connect_btn = tk.Button(
            config_table, text="Conectar", bg="#90EE90",
            command=self.toggle_connection,
        )
        self.connect_btn.grid(row=3, column=0, columnspan=2, sticky="nsew", padx=1, pady=1)
        config_table.grid_columnconfigure(0, weight=0)
        config_table.grid_columnconfigure(1, weight=1)

        # --- Tabela de variáveis a enviar para o ESP (genéricas, expansível) ---
        vars_frame = tk.LabelFrame(side_frame, text="Variáveis para o ESP32 (futuro)")
        vars_frame.pack(fill="x", pady=(0, 8))

        self.var_rows = []  # cada item: (nome_entry, valor_entry)
        self.vars_table = tk.Frame(vars_frame)
        self.vars_table.pack(fill="x", padx=4, pady=4)

        for _ in range(4):
            self._add_var_row()

        btns_frame = tk.Frame(vars_frame)
        btns_frame.pack(fill="x", padx=4, pady=(0, 4))
        tk.Button(btns_frame, text="+ Variável", command=self._add_var_row).pack(side="left", expand=True, fill="x", padx=2)
        tk.Button(btns_frame, text="Enviar Variáveis", bg="#ADD8E6", command=self.send_variables).pack(side="left", expand=True, fill="x", padx=2)

        # --- Estatísticas de rastreamento em tempo real ---
        stats_frame = tk.LabelFrame(side_frame, text="Status do Rastreamento")
        stats_frame.pack(fill="x")

        self.stats_labels = {}
        for i, key in enumerate(["ID Alvo", "ErroX", "ErroY", "Área (px)", "FPS"]):
            tk.Label(stats_frame, text=f"{key}:", anchor="w", width=12).grid(row=i, column=0, sticky="w", padx=4, pady=2)
            lbl = tk.Label(stats_frame, text="--", anchor="w", fg="#0a6b0a", font=("Consolas", 10, "bold"))
            lbl.grid(row=i, column=1, sticky="w", padx=4, pady=2)
            self.stats_labels[key] = lbl

        # ---- Linha 2: Monitor / Erros ----
        bottom_frame = tk.Frame(self.root)
        bottom_frame.pack(fill="both", expand=False, padx=8, pady=(0, 8))

        monitor_frame = tk.LabelFrame(bottom_frame, text="Monitor:")
        monitor_frame.pack(side="left", fill="both", expand=True, padx=(0, 4))
        self.monitor_text = tk.Text(monitor_frame, height=8, state="disabled", bg="white")
        self.monitor_text.pack(fill="both", expand=True, padx=2, pady=2)

        erros_frame = tk.LabelFrame(bottom_frame, text="Erros:")
        erros_frame.pack(side="left", fill="both", expand=True, padx=(4, 0))
        self.erros_text = tk.Text(erros_frame, height=8, state="disabled", bg="white", fg="#a30000")
        self.erros_text.pack(fill="both", expand=True, padx=2, pady=2)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _add_row(self, parent, row, label_text, default, attr):
        tk.Label(parent, text=label_text, anchor="w").grid(row=row, column=0, sticky="nsew", padx=4, pady=2)
        entry = tk.Entry(parent)
        entry.insert(0, default)
        entry.grid(row=row, column=1, sticky="nsew", padx=4, pady=2)
        setattr(self, attr, entry)

    def _add_var_row(self):
        row_idx = len(self.var_rows)
        name_entry = tk.Entry(self.vars_table, width=12)
        name_entry.insert(0, f"var{row_idx + 1}")
        name_entry.grid(row=row_idx, column=0, padx=2, pady=1, sticky="we")

        value_entry = tk.Entry(self.vars_table, width=12)
        value_entry.insert(0, "0")
        value_entry.grid(row=row_idx, column=1, padx=2, pady=1, sticky="we")

        self.vars_table.grid_columnconfigure(0, weight=1)
        self.vars_table.grid_columnconfigure(1, weight=1)

        self.var_rows.append((name_entry, value_entry))

    # -------------------- LOG HELPERS --------------------
    def log(self, kind, msg):
        """Thread-safe: agenda a escrita no widget de texto na thread principal do Tkinter."""
        self.root.after(0, self._log_ui, kind, msg)

    def _log_ui(self, kind, msg):
        target = self.monitor_text if kind == "info" else self.erros_text
        target.config(state="normal")
        timestamp = time.strftime("%H:%M:%S")
        target.insert("end", f"[{timestamp}] {msg}\n")
        target.see("end")
        target.config(state="disabled")

    # -------------------- CONEXÃO --------------------
    def toggle_connection(self):
        if not self.connected:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        stream_url = self.stream_entry.get().strip()
        self.log("info", f"Conectando ao stream: {stream_url}")

        self.cam = CameraStream(stream_url)
        ok = self.cam.start()
        if not ok:
            self.log("erro", self.cam.error or "Falha ao conectar na câmera.")
            return

        self.log("info", "Câmera conectada. Carregando modelo YOLO...")
        self.connect_btn.config(text="Carregando...", state="disabled")
        self.root.update_idletasks()

        # Carregamento do modelo pode demorar alguns segundos -> feito antes de iniciar a thread,
        # mas sem travar a percepção do usuário pois já avisamos via log.
        def load_and_start():
            self.worker = TrackerWorker(self.cam, self.get_esp_target, self.log)
            try:
                self.worker.load_model("yolov8n.pt")
            except Exception as e:
                self.log("erro", f"Falha ao carregar modelo YOLO: {e}")
                self.root.after(0, self._reset_connect_button)
                return
            self.worker.start()
            self.root.after(0, self._on_connected)

        threading.Thread(target=load_and_start, daemon=True).start()

    def _on_connected(self):
        self.connected = True
        self.connect_btn.config(text="Desconectar", bg="#FFB3B3", state="normal")
        self.log("info", "Rastreamento iniciado.")
        self._schedule_gui_update()

    def _reset_connect_button(self):
        self.connect_btn.config(text="Conectar", bg="#90EE90", state="normal")

    def disconnect(self):
        self.connected = False
        if self.worker is not None:
            self.worker.stop()
            self.worker = None
        if self.cam is not None:
            self.cam.stop()
            self.cam = None
        self.connect_btn.config(text="Conectar", bg="#90EE90")
        self.cam_label.config(image="", text="Sem sinal")
        self.log("info", "Desconectado.")

    def get_esp_target(self):
        """Lido pela thread de tracking a cada envio UDP — permite mudar IP/porta em tempo real."""
        ip = self.ip_entry.get().strip() or DEFAULT_ESP32_IP
        try:
            port = int(self.port_entry.get().strip())
        except ValueError:
            port = DEFAULT_ESP32_PORT
        return ip, port

    def send_variables(self):
        """
        Placeholder para envio futuro de variáveis (ex.: Kp, Ki, Kd) ao ESP32.
        Por enquanto apenas monta a mensagem e mostra no Monitor; quando você
        definir o protocolo, troque o `self.log(...)` por um sock.sendto(...).
        """
        pares = []
        for name_entry, value_entry in self.var_rows:
            nome = name_entry.get().strip()
            valor = value_entry.get().strip()
            if nome:
                pares.append(f"{nome}={valor}")

        if not pares:
            self.log("erro", "Nenhuma variável definida para enviar.")
            return

        payload = ";".join(pares)
        ip, port = self.get_esp_target()
        self.log("info", f"[PLACEHOLDER] Variáveis prontas para envio a {ip}:{port} -> {payload}")
        # Quando o protocolo no ESP32 estiver definido, basta:
        # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # sock.sendto(payload.encode(), (ip, port))

    # -------------------- ATUALIZAÇÃO DA GUI (não bloqueante) --------------------
    def _schedule_gui_update(self):
        if not self.connected or self.worker is None:
            return

        frame = self.worker.get_display_frame()
        if frame is not None:
            self._update_camera_label(frame)

        with self.worker.stats_lock:
            stats = dict(self.worker.stats)

        self.stats_labels["ID Alvo"].config(text=str(stats["id"]) if stats["id"] is not None else "--")
        self.stats_labels["ErroX"].config(text=str(stats["erroX"]))
        self.stats_labels["ErroY"].config(text=str(stats["erroY"]))
        self.stats_labels["Área (px)"].config(text=str(stats["area"]))
        self.stats_labels["FPS"].config(text=f"{stats['fps']:.1f}")

        # Reagenda — isso é o "loop" da GUI, leve e não bloqueante (não interfere no tracking)
        self.root.after(GUI_REFRESH_MS, self._schedule_gui_update)

    def _update_camera_label(self, frame_bgr):
        # Redimensiona para caber no painel sem distorcer
        h, w, _ = frame_bgr.shape
        scale = min(CAM_DISPLAY_W / w, CAM_DISPLAY_H / h)
        new_w, new_h = int(w * scale), int(h * scale)
        resized = cv2.resize(frame_bgr, (new_w, new_h))

        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(rgb)
        img_tk = ImageTk.PhotoImage(image=img_pil)

        self.cam_label.config(image=img_tk, text="")
        self.cam_label.image = img_tk  # mantém referência (evita garbage collection)

    # -------------------- FECHAMENTO --------------------
    def on_close(self):
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = RescueTrackerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()