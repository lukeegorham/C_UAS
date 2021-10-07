# Imports
import socket
import time

# Global Variables
node = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Declare Socket
ip = "192.168.1.50"  # Declare ip to send packet
port = 46555         # Declare port
interval = 3         # Seconds between packets
num = 5              # Number of packets to send
sent = 0             # Number of packets sent


# Function Definitions


# Main Function
def main(name):
    global sent, num, interval, node
    print(f'Hi, {name}')

    while sent < num:
        message = "Received Packet " + str(sent)
        time.sleep(interval)
        node.connect((ip, port))
        node.sendto(bytes(message, 'utf8'), (ip, port))
        sent = sent + 1

    node.close()
    print(f'Finished Sending Packets!')


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main('Sender')
