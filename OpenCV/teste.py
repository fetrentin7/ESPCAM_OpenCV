import cv2
import time
import requests
from threading import Thread
import socket
W_REAL = 0.15           # Largura real do rosto (m)
FOCAL_LENGTH = 800      # Calibração focal
BASELINE = 0.055        # Distância entre câmeras (m)
SEND_INTERVAL = 1.0     # Enviar dados a cada 200ms

ESP_URL = "http://172.20.10.8:3333" 
TCP_IP = "172.20.10.8"
TCP_PORT = 3333
BUFFER_SIZE = 1024

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
print("Conectado!")
# Carrega classificadores
path = cv2.data.haarcascades
face_cascade = cv2.CascadeClassifier(path + "haarcascade_frontalface_default.xml")
profile_cascade = cv2.CascadeClassifier(path + "haarcascade_profileface.xml")

print("===INICIANDO SISTEMA===")

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

            # Tenta ler o frame
            grabbed, frame = self.stream.read()

            if grabbed:
                self.frame = frame
                self.ret = True
            else:
                # Se falhar, marca como erro e tenta reconectar
                self.ret = False
                # Não printa toda hora para não poluir, mas tenta reabrir
                try:
                    self.stream.release()
                    time.sleep(1) # Espera um pouco antes de tentar de novo
                    self.stream = cv2.VideoCapture(self.src)
                    print(f"Tentando reconectar a {self.src}...")
                except:
                    pass

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.stopped = True

# Função auxiliar para detectar rostos (Frente, Perfil ou Invertido)
def detect_one_face(img):
    if img is None: return None
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 1. Frontal
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    if len(faces) > 0: return faces[0]
    
    # 2. Perfil
    faces_prof = profile_cascade.detectMultiScale(gray, 1.3, 5)
    if len(faces_prof) > 0: return faces_prof[0]

    # 3. Perfil Invertido
    flipped = cv2.flip(gray, 1)
    faces_flip = profile_cascade.detectMultiScale(flipped, 1.3, 5)
    if len(faces_flip) > 0:
        (x, y, w, h) = faces_flip[0]
        x = img.shape[1] - x - w 
        return (x, y, w, h)
        
    return None

# === INICIALIZAÇÃO ===
cam_left = VideoStream("http://172.20.10.6").start()
cam_right = VideoStream("http://172.20.10.5").start()
time.sleep(2.0) 

last_send_time = 0 

while True:
    retL, frameL = cam_left.read()
    retR, frameR = cam_right.read()

    # Se alguma câmera caiu, mostra aviso mas NÃO TRAVA O LOOP
    if frameL is None or frameR is None:
        print("Aguardando sinal das câmeras...")
        time.sleep(0.5)
        continue

    # Setup da imagem (Esquerda é a Mestre)
    h, w, _ = frameL.shape
    frame_center = (w // 2, h // 2)
    cv2.circle(frameL, frame_center, 5, (0, 255, 0), -1) # Centro da tela (Verde)

    # Detecção
    rectL = detect_one_face(frameL)
    rectR = detect_one_face(frameR)

    text_info = "Procurando..."

    # === LÓGICA PRINCIPAL (Só se achar rosto nas duas) ===
    if rectL is not None and rectR is not None:
        
        # Coordenadas Esquerda
        (xL, yL, wL, hL) = rectL
        face_center_L = (xL + wL//2, yL + hL//2)
        
        # Coordenadas Direita
        (xR, yR, wR, hR) = rectR
        cxR = xR + wR//2

        # 1. CÁLCULO DE ERRO (X, Y)
        dx = face_center_L[0] - frame_center[0]
        dy = -(face_center_L[1] - frame_center[1])

        # 2. CÁLCULO DE PROFUNDIDADE (Z)
        cxL = face_center_L[0]
        disparity = abs(cxL - cxR)

        if disparity > 2: # Filtro de ruído
            Z = (FOCAL_LENGTH * BASELINE) / disparity
        else:
            Z = 0.0

        # Desenho Visual
        cv2.circle(frameL, face_center_L, 5, (0, 0, 255), -1) # Centro do rosto (Vermelho)
        cv2.rectangle(frameL, (xL, yL), (xL+wL, yL+hL), (255, 0, 0), 2)
        
        text_info = f"X:{dx} Y:{dy} Z:{Z:.2f}"

        # 3. ENVIO TEMPORIZADO (A cada 200ms)
        current_time = time.time()
        if (current_time - last_send_time) > SEND_INTERVAL:

            try:
                msg = f"posX:{int(dx)}, posY:{int(dy)}, posZ:{Z:.2f}"
                s.send(msg.encode())

        # Ler resposta do ESP32 (não trava se não vier)
                try:
                    resp = s.recv(1024).decode()
                    print("ESP32:", resp)
                except:
                    pass

                print(f"✅ ENVIADO! >> {msg}")
                last_send_time = current_time

            except Exception as e:
                print("Erro ao enviar TCP:", e)        
    # === EXIBIÇÃO ===
    cv2.putText(frameL, text_info, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
    
    try:
        frameL_s = cv2.resize(frameL, (0,0), fx=0.6, fy=0.6)
        frameR_s = cv2.resize(frameR, (0,0), fx=0.6, fy=0.6)
        combined = cv2.hconcat([frameL_s, frameR_s])
        cv2.imshow("Sistema Stereo XYZ", combined)
    except Exception as e:
        print(f" ERRO ENVIO: O ESP32 não respondeu. {e}")
        pass

    if cv2.waitKey(1) & 0xFF == 27: # ESC para sair
        break

# Limpeza
cam_left.stop()
cam_right.stop()
cv2.destroyAllWindows()