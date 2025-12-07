import cv2
import time

# THRESHOLDS QUE VOCÊ PEDIU
THRESH_X = 40
THRESH_Y = 40


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.prev_time = time.time()

    def update(self, error):
        current_time = time.time()
        dt = current_time - self.prev_time

        if dt <= 0:
            dt = 1e-6

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.prev_error = error
        self.prev_time = current_time

        return output


# Inicializa vídeo e face detector
cap = cv2.VideoCapture(0)
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

pid_yaw = PID(kp=0.025, ki=0.0, kd=0.003)

SERVO_CENTER = 90
servo_angle_yaw = SERVO_CENTER



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

    for (x, y, fw, fh) in faces:
        face_center = (x + fw//2, y + fh//2)

        cv2.circle(frame, face_center, 5, (0,0,255), -1)
        cv2.rectangle(frame, (x,y), (x+fw,y+fh), (255,0,0), 2)

        dx = face_center[0] - frame_center[0]
        dy = face_center[1] - frame_center[1]

        # THRESH LOGIC ORIGINAL
        msg = []

        errorX = dx - THRESH_X
        errorY = dy - THRESH_Y

        if dx > THRESH_X:
            msg.append("Move direita " + str(errorX))
        elif dx < -THRESH_X:
            msg.append("Move esquerda " + str(errorX))

        if dy > THRESH_Y:
            msg.append("Move cima " + str(errorY))
        elif dy < -THRESH_Y:
            msg.append("Move baixo " + str(errorY))

        if not msg:
            text = "centralizada"
        else:
            text = " e ".join(msg)

        cv2.putText(frame, text, (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    (0, 255, 255), 2)

        # PID → ÂNGULO (só cálculo, sem enviar nada)
        error_yaw = frame_center[0] - face_center[0]

        yaw_output = pid_yaw.update(error_yaw)

        servo_angle_yaw = SERVO_CENTER + yaw_output
        servo_angle_yaw = max(0, min(180, servo_angle_yaw))

        break  # usa só a primeira face


    # DEBUG
    cv2.putText(frame, f"Yaw PID: {yaw_output:.2f}", (20,90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

    cv2.putText(frame, f"Servo Yaw (simulado): {servo_angle_yaw:.1f} graus", (20,120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

    cv2.imshow("Face Tracking + PID (Simulado)", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
