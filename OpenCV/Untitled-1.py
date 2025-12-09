import cv2
import time
import requests
from threading import Thread

# ================= CONFIGURAÇÕES =================
W_REAL = 0.15           # Largura real do rosto (m)
FOCAL_LENGTH = 800      # Calibração focal
BASELINE = 0.055        # Distância entre câmeras (m)

# URL do ESP32 (Substitua pelo IP correto do seu ESP)
ESP_URL = "http://172.20.10.5" 

# Carrega classificadores
path = cv2.data.haarcascades
face_cascade = cv2.CascadeClassifier(path + "haarcascade_frontalface_default.xml")
profile_cascade = cv2.CascadeClassifier(path + "haarcascade_profileface.xml")

print("=== INICIANDO SISTEMA COMPLETO (XYZ) ===")

# === CLASSE PARA LEITURA RÁPIDA (THREAD) ===
class VideoStream:
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        self.ret, self.frame = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            if self.stopped: return
            self.ret, self.frame = self.stream.read()

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()

# Função para detectar rosto (Frente ou Perfil)
def detect_one_face(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 1. Tenta frontal
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    if len(faces) > 0: return faces[0]
    
    # 2. Tenta perfil
    faces_prof = profile_cascade.detectMultiScale(gray, 1.3, 5)
    if len(faces_prof) > 0: return faces_prof[0]

    # 3. Tenta perfil invertido (flip)
    flipped = cv2.flip(gray, 1)
    faces_flip = profile_cascade.detectMultiScale(flipped, 1.3, 5)
    if len(faces_flip) > 0:
        (x, y, w, h) = faces_flip[0]
        x = img.shape[1] - x - w # Desinverter X
        return (x, y, w, h)
        
    return None

# === INICIALIZA CÂMERAS ===
# Certifique-se que os IPs estão corretos
cam_left = VideoStream("http://172.20.10.6:81/stream").start()
cam_right = VideoStream("http://172.20.10.5:81/stream").start()
time.sleep(2.0) # Tempo para estabilizar

while True:
    retL, frameL = cam_left.read()
    retR, frameR = cam_right.read()

    if not retL or not retR:
        continue

    # Pega dimensões e centro da imagem (Baseado na Esquerda)
    h, w, _ = frameL.shape
    frame_center = (w // 2, h // 2)

    # Desenha o centro da tela (alvo)
    cv2.circle(frameL, frame_center, 5, (0, 255, 0), -1)

    # Detecta rosto nas duas câmeras
    rectL = detect_one_face(frameL)
    rectR = detect_one_face(frameR)

    text_info = "Procurando..."

    # Se detectou nas duas, faz o cálculo completo
    if rectL is not None and rectR is not None:
        # Câmera Esquerda (Referência para X e Y)
        (xL, yL, wL, hL) = rectL
        face_center_L = (xL + wL//2, yL + hL//2)
        
        # Câmera Direita (Apenas para Z)
        (xR, yR, wR, hR) = rectR
        cxR = xR + wR//2

        # === 1. CÁLCULO DE POSIÇÃO (X, Y) ===
        # Baseado na lógica que você pediu
        dx = face_center_L[0] - frame_center[0]
        dy = -(face_center_L[1] - frame_center[1]) # Invertido pois Y cresce pra baixo

        # === 2. CÁLCULO DE PROFUNDIDADE (Z) ===
        cxL = face_center_L[0]
        disparity = abs(cxL - cxR)

        if disparity > 2:
            Z = (FOCAL_LENGTH * BASELINE) / disparity
        else:
            Z = 0.0

        # Desenhos na tela
        cv2.circle(frameL, face_center_L, 5, (0, 0, 255), -1)
        cv2.rectangle(frameL, (xL, yL), (xL+wL, yL+hL), (255, 0, 0), 2)
        
        text_info = f"X:{dx} Y:{dy} Z:{Z:.2f}"

        # === 3. ENVIO PARA O ESP32 ===
        # Envia no formato que você quer capturar com sscanf ou parser
        try:
            # Payload ajustado para posX, posY, posZ
            payload = {
                "posX": dx, 
                "posY": dy, 
                "posZ": Z
            }
            # Timeout curto para não travar o vídeo
            requests.get(ESP_URL, params=payload, timeout=0.02)
        except:
            pass

    # Exibição
    cv2.putText(frameL, text_info, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
    
    # Redimensiona para mostrar lado a lado
    frameL_s = cv2.resize(frameL, (0,0), fx=0.6, fy=0.6)
    frameR_s = cv2.resize(frameR, (0,0), fx=0.6, fy=0.6)
    
    combined = cv2.hconcat([frameL_s, frameR_s])
    cv2.imshow("Stereo Tracking XYZ", combined)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cam_left.stop()
cam_right.stop()
cv2.destroyAllWindows()