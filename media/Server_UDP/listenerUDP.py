import socket

UDP_IP = "0.0.0.0"  # Escuchar en todas las interfaces
UDP_PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Escuchando en {UDP_IP}:{UDP_PORT}")

while True:
    data, addr = sock.recvfrom(1024)
    print(f"Mensaje recibido de {addr}: {data.decode()}")
    # Enviar confirmación
    sock.sendto(b"Datos recibidos", addr)