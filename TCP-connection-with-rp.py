import socket
import json

# TCP/IP server configuration
SERVER_IP = '0.0.0.0'  # Listen on all available network interfaces
SERVER_PORT = 5000  # Choose a desired port number

# Create a TCP/IP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the server IP and port
server_socket.bind((SERVER_IP, SERVER_PORT))

# Listen for incoming connections
server_socket.listen(1)

print(f"Waiting for incoming connection on {SERVER_IP}:{SERVER_PORT}...")

# Accept a connection request from the Raspberry Pi
client_socket, client_address = server_socket.accept()
print(f"Connection established with {client_address[0]}:{client_address[1]}")

try:
    while True:
        # Receive the data from the client
        data = client_socket.recv(1024).decode()
        
        # If data is empty, the client has closed the connection
        if not data:
            print("Connection closed by the client.")
            break
        
        # Parse the received JSON data
        sensor_data = json.loads(data)
        
        # Process the sensor data as needed
        # You can access the individual sensor values using the dictionary keys
        distance1 = sensor_data['distance1']
        distance2 = sensor_data['distance2']
        rear_left = sensor_data['rear_left']
        rear_right = sensor_data['rear_right']
        front_right = sensor_data['front_right']
        front_left = sensor_data['front_left']
        
        # Print the received sensor data
        print(f"Distance 1: {distance1} cm")
        print(f"Distance 2: {distance2} cm")
        print(f"Rear Left Distance: {rear_left} cm")
        print(f"Rear Right Distance: {rear_right} cm")
        print(f"Front Right Distance: {front_right} cm")
        print(f"Front Left Distance: {front_left} cm")

except KeyboardInterrupt:
    print("Server terminated by user.")

finally:
    # Close the client socket and server socket
    client_socket.close()
    server_socket.close()
