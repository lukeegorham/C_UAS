# Imports
import socket
import time

# Global Variables
node = socket.socket()          # Declare Socket
ip = "192.168.1.50"                # Declare ip to send packet
port = 46555                    # Declare port
listen_time = 10  # Number of seconds to listen

# Function Definitions


# Main Function
def main(name):
    print(f'Hi, {name}')
    # node.connect((ip, port))
    node.bind((ip, port))
    node.listen(1)
    message, addr = node.accept()
    end_time = time.time() + listen_time
    print(f'Starting Receiving Messages')

    while time.time() < end_time:
        print(message.recv(1024))

    node.close()


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main('Receiver')
