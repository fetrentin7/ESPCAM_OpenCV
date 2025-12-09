import socket
import time

# === CONFIGURAÇÃO ===
# Use o IP que apareceu no seu log
TCP_IP = '172.20.10.8'
TCP_PORT = 3333
BUFFER_SIZE = 1024

print(f"Conectando ao ESP32 em {TCP_IP}:{TCP_PORT}...")

try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    
    # Recebe a mensagem de boas-vindas "Conexao estabelecida!"
    data = s.recv(BUFFER_SIZE)
    print("Recebido:", data.decode())

    # === TESTE 1: PARADO (Z=1.0 pois o target é 1.0, logo erro=0) ===
    # O PID de Z busca chegar em 1.0. Se enviamos 1.0, o erro é 0.
    msg = "posX:0.0, posY:0.0, posZ:1.0"
    print(f"Enviando PARAR: {msg}")
    s.send(msg.encode())
    print("Resposta:", s.recv(BUFFER_SIZE).decode())
    time.sleep(2)

    # === TESTE 2: ACELERAR (Z=0.0) ===
    # Target é 1.0. Enviamos 0.0. Erro = 1.0. O PID vai acelerar os motores.
    msg = "posX:0.0, posY:0.0, posZ:0.0"
    print(f"Enviando ACELERAR: {msg}")
    s.send(msg.encode())
    print("Resposta:", s.recv(BUFFER_SIZE).decode())
    time.sleep(3)

    # === TESTE 3: VIRAR (X altera a diferença entre motores) ===
    msg = "posX:50.0, posY:0.0, posZ:0.0"
    print(f"Enviando VIRAR: {msg}")
    s.send(msg.encode())
    print("Resposta:", s.recv(BUFFER_SIZE).decode())
    time.sleep(3)

    # === TESTE 4: SERVO (Y controla o servo) ===
    # No seu código: angulo = 90 + (y * 0.3)
    # Se Y=100 -> 90 + 30 = 120 graus
    msg = "posX:0.0, posY:100.0, posZ:1.0"
    print(f"Enviando SERVO: {msg}")
    s.send(msg.encode())
    print("Resposta:", s.recv(BUFFER_SIZE).decode())

    s.close()
    print("Teste finalizado.")

except Exception as e:
    print(f"Erro: {e}")
    print("Verifique se o computador está na mesma rede Wi-Fi (iPhone) que o ESP32.")