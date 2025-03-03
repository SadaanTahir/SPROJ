import socket
import threading
import time
from advanced_simulation.utils import *

# UDP Server (Drone Communication Center)
def udp_server(host='0.0.0.0', port=5005):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    print(f"UDP server is running on {host}:{port}")
    
    while True:
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        print(f"Received message: {data.decode()} from {addr}")
        response = f"Message received from {addr}: {data.decode()}"
        # Send response back to the client (drone)
        sock.sendto(response.encode(), addr)

# Generic UDP Client (Drone)
def udp_client(client_port, server_ip, server_port, drone_id):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Bind the client to its specific port
    sock.bind(('localhost', client_port))
    
    while True:
        message = f"Hello from {drone_id}"
        sock.sendto(message.encode(), (server_ip, server_port))
        print(f"{drone_id} (port {client_port}) sent message: {message}")
        
        # Receive response from the server
        data, _ = sock.recvfrom(1024)
        print(f"{drone_id} received response: {data.decode()}")
        time.sleep(1)

# Main function to run the server and multiple clients (drones)
if __name__ == "__main__":
    connections = connect_n_drones(2)

    server_ip = '127.0.0.1'
    server_port = 5501
    num_drones = 2  # Adjust this value to create more drone clients

    # Client ports for each drone
    drone_ports = [14550, 14560]  # Add more ports if you have more drones

    # Create and start server thread
    server_thread = threading.Thread(target=udp_server, args=(server_ip, server_port))
    server_thread.daemon = True
    server_thread.start()

    # Create and start drone client threads
    drone_threads = []
    for i in range(num_drones):
        drone_id = f"Drone {i+1}"
        client_port = drone_ports[i]
        drone_thread = threading.Thread(target=udp_client, args=(client_port, server_ip, server_port, drone_id))
        drone_thread.daemon = True
        drone_thread.start()
        drone_threads.append(drone_thread)

    # Keep the main thread alive
    while True:
        time.sleep(1)
