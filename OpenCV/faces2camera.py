import cv2
import time
import numpy as np

FOCAL_LENGTH = 800      # Fator de calibração (pixels)
W_REAL = 0.06         # Distância entre as câmeras em METROS (ex: 6cm = 0.06)

# Inicializa as DUAS câmeras
cap_left = cv2.VideoCapture(0)
cap_right = cv2.VideoCapture(1)

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

def find_face_center(frame):
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    
    # Pega o maior rosto encontrado (para evitar ruído de fundo)
    if len(faces) > 0:
        # Ordena para pegar o maior retângulo (w * h)
        faces = sorted(faces, key=lambda f: f[2]*f[3], reverse=True)
        (x, y, w, h) = faces[0]
        
        center_x = x + w // 2
        center_y = y + h // 2
        
        # Desenha na imagem para visualização
        cv2.rectangle(frame, (x,y), (x+w,y+h), (255,0,0), 2)
        cv2.circle(frame, (center_x, center_y), 5, (0,255,0), -1)
        
        return (center_x, center_y)
    
    return None

while True:
    retL, frameL = cap_left.read()
    retR, frameR = cap_right.read()

    if not retL or not retR:
        print("Erro ao ler câmeras")
        break

    # 2. Encontrar rostos em ambas as câmeras
    centerL = find_face_center(frameL)
    centerR = find_face_center(frameR)

    distance_Z = 0
    text_info = "Procurando..."

    #centro da esquerda e centoro da direita
    if centerL is not None and centerR is not None:
        
        x_right = centerR[0]
        x_left = centerL[0]
        disparity = x_left - x_right

        if disparity > 0:
            distance_Z = (FOCAL_LENGTH * W_REAL) / disparity
            
            #text_info = f"Dist: {distance_Z:.2f} m | Disp: {disparity}px"
            
            # Coordenadas X e Y médias (para centralizar o tracking)
            avg_x = (x_left + x_right) / 2
            avg_y = (centerL[1] + centerR[1]) / 2
            h, w, _ = frameL.shape
            
            errorX = x_left - (w // 2)
            errorY = -centerL[1] + (h // 2)
            
            msg = f"X:{errorX},Y:{errorY},Z:{distance_Z:.2f}\n"
            print(msg)
            text_info = msg
        else:
            text_info = "Erro: Disparidade Negativa (Cameras trocadas?)"


    vis = np.hstack((frameL, frameR))
    
    cv2.putText(vis, text_info, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                0.8, (0, 255, 255), 2)

    cv2.imshow("Stereo Vision - Left / Right", vis)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap_left.release()
cap_right.release()
cv2.destroyAllWindows()