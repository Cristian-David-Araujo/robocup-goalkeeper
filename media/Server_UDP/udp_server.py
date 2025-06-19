from flask import Flask, render_template, request, jsonify
import socket
import threading

app = Flask(__name__)

# Configuración UDP
UDP_PORT = 3333  # Puerto que usaremos para enviar los datos
socket_timeout = 2  # Timeout en segundos

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/send_udp', methods=['POST'])
def send_udp():
    try:
        data = request.json
        ip_address = data['ip']
        option = data['option']
        number1 = data['numbers'][0]
        number2 = data['numbers'][1]
        angle = data['angle']
        
        # Validar la dirección IP
        try:
            socket.inet_aton(ip_address)
        except socket.error:
            return jsonify({'status': 'error', 'message': 'Dirección IP inválida'}), 400
        
        # Crear el mensaje a enviar
        message = f"{option},{number1},{number2},{angle}"
        
        # Configurar el socket UDP
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect((ip_address, UDP_PORT))  # Reemplaza con la IP del servidor
        sock.settimeout(socket_timeout)
        
        # Enviar los datos
        sock.sendto(message.encode('utf-8'), (ip_address, UDP_PORT))
        
        # Opcional: esperar respuesta (si el servidor UDP responde)
        try:
            response, _ = sock.recvfrom(1024)
            return jsonify({
                'status': 'success',
                'message': 'Datos enviados correctamente',
                'response': response.decode('utf-8')
            })
        except socket.timeout:
            return jsonify({
                'status': 'success',
                'message': 'Datos enviados (sin confirmación del servidor)'
            })
        
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': f'Error al enviar datos: {str(e)}'
        }), 500

if __name__ == '__main__':
    # Configuración para desarrollo
    app.run(host='0.0.0.0', port=5000, debug=True)