import cv2
import time
from threading import Thread
import socket

# === CONFIGURAÇÕES DO SISTEMA ===
W_REAL = 0.15           # Largura real do rosto (m)
FOCAL_LENGTH = 800      # Calibração focal
BASELINE = 0.055        # Distância entre câmeras (m)
SEND_INTERVAL = 2     # Enviar dados a cada 200ms

# === CONFIG TCP ===
TCP_IP = "172.20.10.8"
TCP_PORT = 3333
BUFFER_SIZE = 1024

print(f"Conectando ao ESP32 em {TCP_IP}:{TCP_PORT}...")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
print("Conectado!")

# Se o ESP mandar mensagem inicial, lê:
try:
    welcome = s.recv(BUFFER_SIZE).decode()
    print("ESP32:", welcome)
except:
    pass

# === CLASSIFICADORES ===
path = cv2.data.haarcascades
face_cascade = cv2.CascadeClassifier(path + "haarcascade_frontalface_default.xml")
profile_cascade = cv2.CascadeClassifier(path + "haarcascade_profileface.xml")

print("=== INICIANDO SISTEMA ===")

# ==========================================================
# CLASS DA THREAD DE VÍDEO
# ==========================================================
class VideoStream:
    def __init__(self, src=0):
        self.src = src
        self.stream = cv2.VideoCapture(src)
        self.ret, self.frame = self.stream.read()
        self.stopped = False
        if not self.ret:
            self.frame = None

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                self.stream.release()
                return

            grabbed, frame = self.stream.read()

            if grabbed:
                self.frame = frame
                self.ret = True
            else:
                self.ret = False
                try:
                    self.stream.release()
                    time.sleep(1)
                    self.stream = cv2.VideoCapture(self.src)
                    print(f"Tentando reconectar a {self.src}...")
                except:
                    pass

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.stopped = True

# ==========================================================
# DETECÇÃO DE UM ROSTO (FRONTAL, PERFIL, PERFIL INVERTIDO)
# ==========================================================
def detect_one_face(img):
    if img is None:
        return None

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 1. Frontal
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    if len(faces) > 0:
        return faces[0]

    # 2. Perfil
    faces_prof = profile_cascade.detectMultiScale(gray, 1.3, 5)
    if len(faces_prof) > 0:
        return faces_prof[0]

    # 3. Perfil invertido
    flipped = cv2.flip(gray, 1)
    faces_flip = profile_cascade.detectMultiScale(flipped, 1.3, 5)
    if len(faces_flip) > 0:
        (x, y, w, h) = faces_flip[0]
        x = img.shape[1] - x - w
        return (x, y, w, h)

    return None

# ==========================================================
# INICIALIZA CÂMERAS
# ==========================================================
cam_left = VideoStream("http://172.20.10.6").start()
cam_right = VideoStream("http://172.20.10.5").start()
time.sleep(2.0)

last_send_time = 0

# ==========================================================
# LOOP PRINCIPAL
# ==========================================================
while True:
    retL, frameL = cam_left.read()
    retR, frameR = cam_right.read()

    if frameL is None or frameR is None:
        print("Aguardando sinal das câmeras...")
        time.sleep(0.5)
        continue

    h, w, _ = frameL.shape
    frame_center = (w // 2, h // 2)
    cv2.circle(frameL, frame_center, 5, (0, 255, 0), -1)

    rectL = detect_one_face(frameL)
    rectR = detect_one_face(frameR)

    text_info = "Procurando..."

    # ======================================================
    # SE ACHOU ROSTO NAS DUAS CÂMERAS
    # ======================================================
    if rectL is not None and rectR is not None:
        (xL, yL, wL, hL) = rectL
        face_center_L = (xL + wL//2, yL + hL//2)

        (xR, yR, wR, hR) = rectR
        cxR = xR + wR//2

        # 1. Erros X e Y
        dx = face_center_L[0] - frame_center[0]
        dy = -(face_center_L[1] - frame_center[1])

        # 2. Profundidade Z
        cxL = face_center_L[0]
        disparity = abs(cxL - cxR)

        if disparity > 2:
            Z = (FOCAL_LENGTH * BASELINE) / disparity
        else:
            Z = 0.0

        cv2.circle(frameL, face_center_L, 5, (0, 0, 255), -1)
        cv2.rectangle(frameL, (xL, yL), (xL+wL, yL+hL), (255, 0, 0), 2)

        text_info = f"X:{dx} Y:{dy} Z:{Z:.2f}"

        # ==================================================
        # ENVIO TCP A CADA 200ms
        # ==================================================
        current_time = time.time()
        if (current_time - last_send_time) > SEND_INTERVAL:
            try:
                msg = f"posX:{int(dx)}, posY:{int(dy)}, posZ:{Z:.2f}"
                s.send(msg.encode())

                # Lê resposta (não trava)
                try:
                    resp = s.recv(1024).decode()
                    print("ESP32:", resp)
                except:
                    pass

                print(f"✅ ENVIADO >> {msg}")
                last_send_time = current_time

            except Exception as e:
                print("Erro ao enviar TCP:", e)

    # ======================================================
    # EXIBIÇÃO
    # ======================================================
    cv2.putText(frameL, text_info, (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

    try:
        frameL_s = cv2.resize(frameL, (0,0), fx=0.6, fy=0.6)
        frameR_s = cv2.resize(frameR, (0,0), fx=0.6, fy=0.6)
        combined = cv2.hconcat([frameL_s, frameR_s])
        cv2.imshow("Sistema Stereo XYZ", combined)
    except Exception as e:
        print("Erro ao exibir:", e)
        pass

    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break

# ==========================================================
# FINALIZAÇÃO
# ==========================================================
cam_left.stop()
cam_right.stop()
cv2.destroyAllWindows()
s.close()
print("Conexão encerrada.")
