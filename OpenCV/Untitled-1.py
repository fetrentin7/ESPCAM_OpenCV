import cv2
import time
import socket



W_REAL = 0.15 #largura do rosto em metros
FOCAL_LENGTH = 800
ESP_IP = " "  #
ESP_PORT = 5000

#sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#sock.connect((ESP_IP, ESP_PORT))

#print("Conectado ao ESP32 via TCP!")


#posx: numero pox, pos y e a distancia, valor posicao x = tal, z = tal
#QUADRADO

#class PID:
#c   def __init__(self, kp, ki, kd):
#c       self.kp = kp
#c       self.ki = ki
#c       self.kd = kd
#c       self.integral = 0
#c       self.prev_error = 0
#c       self.prev_time = time.time()
#c
#c   def update(self, error):
#c       current_time = time.time()
#c       dt = current_time - self.prev_time
#c
#c       if dt <= 0:
#c           dt = 1e-6
#c
#c       self.integral += error * dt
#c       derivative = (error - self.prev_error) / dt
#c
#c       output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
#c
#c       self.prev_error = error
#c       self.prev_time = current_time
#c
#c       return output
#c

# Inicializa vídeo e face detector
cap = cv2.VideoCapture(0)
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

#pid_yaw = PID(kp=0.025, ki=0.0, kd=0.003)

#SERVO_CENTER = 90
#servo_angle_yaw = SERVO_CENTER


while True:
    ret, frame = cap.read()
    if not ret:
        break
    

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    h, w, _ = frame.shape
    frame_center = (w // 2, h // 2)
    cv2.circle(frame, frame_center, 5, (0,255,0), -1)

    yaw_output = 0
    text = "Nenhuma face detectada"

    for (x, y, fw, fh) in faces:
        face_center = (x + fw//2, y + fh//2)

        cv2.circle(frame, face_center, 5, (0,0,255), -1)
        cv2.rectangle(frame, (x,y), (x+fw,y+fh), (255,0,0), 2)

        dx = face_center[0] - frame_center[0]
        dy = -face_center[1] + frame_center[1]

        distance = (FOCAL_LENGTH * W_REAL)/fw
        # Mostrar só a posição relativa ao centro
        msg = []

        errorX = dx 
        errorY = dy 



        #text = f"Posicao ({dx},  {dy})"
        
        text = f"X:{dx},Y:{dy},Z:{distance:.2f}\n"

        #sock.sendall(msg.encode())

        break  

    cv2.putText(frame, text, (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                (0, 255, 255), 2)

    # MOSTRAR NA JANELA — ESSENCIAL
    cv2.imshow("Face Tracking + PID (Simulado)", frame)

    # ESC PRA SAIR
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
