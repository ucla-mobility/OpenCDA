import socket
import time

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('localhost', 5000))
tick = 0

while True:
    tick += 1
    message = f"Sythetic llm Message at tick {tick}"
    client_socket.sendall(message.encode())
    print(f"Sent: {message}")
    time.sleep(1)  # Wait for 1 second before the next tick

client_socket.close()

