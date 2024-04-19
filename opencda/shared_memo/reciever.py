import socket

# Create a socket object using the Internet address family and socket type as TCP
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bind the socket to address ('localhost', 5000) specifies the server's interface and port
server_socket.bind(('localhost', 5000))
# Enable the server to accept connections; specifies the number of unaccepted connections that the system will allow before refusing new connections
server_socket.listen()

# Wait for a connection
print('Waiting for a connection...')
conn, addr = server_socket.accept()  # Accept a connection; conn is a new socket object usable to send and receive data on the connection

print(f'Connection established with {addr}')
while True:
    data = conn.recv(1024)  # Receive data from the client
    if not data:
        break
    print("Received..")  # Decode and print the data
    print(data.decode())
    print('---------------------')
    conn.sendall("ACK".encode())  # Send an acknowledgment back to the client

conn.close()  # Close the connection

